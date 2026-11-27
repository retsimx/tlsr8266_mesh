#!/usr/bin/env python3
"""Secret-free HIL acceptance driver for the TLSR8266 firmware."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
MATRIX_PATH = REPO_ROOT / "tests" / "fixtures" / "hil_matrix.json"
PATCHED_TOOLCHAIN = Path("/home/lewis/Projects/lights/llvm/build-native/bin")


class HilError(RuntimeError):
    pass


def load_matrix(path: str | Path = MATRIX_PATH) -> dict:
    matrix_path = Path(path)
    with matrix_path.open("r", encoding="utf-8") as handle:
        matrix = json.load(handle)
    validate_matrix(matrix)
    return matrix


def validate_matrix(matrix: dict) -> None:
    if not isinstance(matrix, dict):
        raise HilError("HIL matrix must be a JSON object")

    bench = matrix.get("bench")
    if not isinstance(bench, dict):
        raise HilError("HIL matrix bench section is required")
    required_bench = {
        "host": "root@10.0.0.150",
        "artifacts_dir": "/root/hil-accept/artifacts",
        "runner": "/root/hil-accept/run.sh",
    }
    for key, expected in required_bench.items():
        if bench.get(key) != expected:
            raise HilError(f"unexpected bench {key}: {bench.get(key)!r}")

    images = matrix.get("images")
    if not isinstance(images, dict):
        raise HilError("HIL matrix images section is required")
    for name in ("rtt", "base", "next"):
        image = images.get(name)
        if not isinstance(image, dict):
            raise HilError(f"missing {name} image metadata")
        if image.get("bin") != f"{name}.bin" or image.get("elf") != f"{name}.elf":
            raise HilError(f"invalid {name} image names")

    preflight = matrix.get("preflight")
    if not isinstance(preflight, dict):
        raise HilError("HIL matrix preflight section is required")
    if preflight.get("dut_mesh_addr_range") != [1, 63]:
        raise HilError("DUT mesh address range must be [1, 63]")
    if preflight.get("allow_silent_fallback") is not False:
        raise HilError("silent fallback must be disabled")

    stages = matrix.get("stages")
    if stages != list(range(6)):
        raise HilError("HIL stages must be [0, 1, 2, 3, 4, 5]")

    profiles = matrix.get("profiles")
    if not isinstance(profiles, dict):
        raise HilError("HIL profiles section is required")
    for name in ("12mhz", "16mhz"):
        if name not in profiles:
            raise HilError(f"missing {name} profile")
        profile_stages = profiles[name].get("stages")
        if not isinstance(profile_stages, list) or any(
            stage not in range(6) for stage in profile_stages
        ):
            raise HilError(f"invalid stages in {name} profile")

    if matrix.get("secret_free") is not True:
        raise HilError("HIL matrix must declare secret_free=true")


def parse_stages(value: str | None, default: Iterable[int]) -> list[int]:
    if value is None:
        return list(default)
    stages: list[int] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            raise HilError("empty stage in --stages")
        try:
            stage = int(item)
        except ValueError as exc:
            raise HilError(f"invalid stage: {item}") from exc
        if stage not in range(6):
            raise HilError(f"stage out of range: {stage}")
        if stage not in stages:
            stages.append(stage)
    return stages


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("12mhz", "16mhz"), default="12mhz")
    parser.add_argument("--stages", metavar="CSV", default=None)
    parser.add_argument("--toolchain-bin", default=str(PATCHED_TOOLCHAIN), metavar="DIR")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def validate_toolchain(toolchain_bin: str) -> Path:
    toolchain = Path(toolchain_bin)
    if not toolchain.is_dir():
        raise HilError(f"patched TC32 toolchain directory not found: {toolchain}")

    required = ("llc", "llvm-link", "opt", "ld.lld", "llvm-objcopy")
    missing = [name for name in required if not (toolchain / name).is_file()]
    if missing:
        raise HilError(
            "patched TC32 toolchain is incomplete; missing: " + ", ".join(missing)
        )
    return toolchain


def run(command: list[str], cwd: Path | None = None, dry_run: bool = False) -> subprocess.CompletedProcess[str]:
    if dry_run:
        print("DRY-RUN:", " ".join(str(part) for part in command))
        return subprocess.CompletedProcess(command, 0, "", "")
    return subprocess.run(command, cwd=str(cwd) if cwd else None, check=True)


def add_temp_worktree(repo: Path, parent: Path, label: str) -> Path:
    worktree = parent / label
    subprocess.run(
        ["git", "-C", str(repo), "worktree", "add", "--detach", str(worktree), "HEAD"],
        check=True,
    )
    return worktree


def bump_version_in_worktree(worktree: Path) -> tuple[int, int]:
    version_path = worktree / "rust" / "src" / "version.rs"
    text = version_path.read_text(encoding="utf-8")
    import re

    match = re.search(r"pub const BUILD_VERSION: u32 = (\d+);", text)
    if not match:
        raise HilError("could not find BUILD_VERSION in temporary worktree")
    old = int(match.group(1))
    new = old + 1
    version_path.write_text(
        text[: match.start(1)] + str(new) + text[match.end(1) :],
        encoding="utf-8",
    )
    return old, new


def build_image(
    worktree: Path,
    toolchain: Path,
    features: str,
    staging: Path,
    image_name: str,
) -> dict[str, Path]:
    command = [
        "make",
        *((["FEATURES=rtt"] if features == "rtt" else [])),
        f"LLC={toolchain / 'llc'}",
        f"LLVM_LINK={toolchain / 'llvm-link'}",
        f"OPT={toolchain / 'opt'}",
        f"LD={toolchain / 'ld.lld'}",
        f"OBJCOPY={toolchain / 'llvm-objcopy'}",
    ]
    subprocess.run(command, cwd=str(worktree), check=True)

    elf = worktree / "_build" / "lightblemesh"
    bin_path = worktree / "_build" / "lightblemesh.bin"
    if not elf.is_file() or not bin_path.is_file():
        raise HilError(f"build did not produce {image_name} ELF and BIN")

    staged_elf = staging / f"{image_name}.elf"
    staged_bin = staging / f"{image_name}.bin"
    shutil.copy2(elf, staged_elf)
    shutil.copy2(bin_path, staged_bin)
    return {"elf": staged_elf, "bin": staged_bin}


def find_prebuilt(repo: Path, image_name: str) -> dict[str, Path]:
    candidates = (
        repo / "_build" / "hil",
        repo / "_build",
    )
    for directory in candidates:
        elf = directory / f"{image_name}.elf"
        bin_path = directory / f"{image_name}.bin"
        if elf.is_file() and bin_path.is_file():
            return {"elf": elf, "bin": bin_path}
    raise HilError(
        f"--skip-build could not find prebuilt {image_name}.elf and {image_name}.bin "
        f"under {repo / '_build'}"
    )


def stage_images(bench: dict, staged: dict[str, dict[str, Path]], dry_run: bool = False) -> None:
    paths = [
        str(staged[name][kind])
        for name in ("rtt", "base", "next")
        for kind in ("bin", "elf")
    ]
    command = [
        "scp",
        *paths,
        f"{bench['host']}:{bench['artifacts_dir']}/",
    ]
    run(command, dry_run=dry_run)


def invoke_runner(bench: dict, stages: list[int], dry_run: bool = False) -> None:
    full_run = stages == list(range(6))
    if full_run:
        run(["ssh", bench["host"], bench["runner"]], dry_run=dry_run)
        return
    for stage in stages:
        run(
            ["ssh", bench["host"], bench["runner"], "--stage", str(stage)],
            dry_run=dry_run,
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        matrix = load_matrix()
        stages = parse_stages(args.stages, matrix["profiles"][args.profile]["stages"])

        print(
            f"HIL profile={args.profile} stages={','.join(str(stage) for stage in stages)}"
        )

        if args.dry_run:
            staged = {
                name: {
                    "bin": Path("/tmp") / f"{name}.bin",
                    "elf": Path("/tmp") / f"{name}.elf",
                }
                for name in ("rtt", "base", "next")
            }
            if not args.skip_build:
                print("DRY-RUN: build rtt, base, and next in temporary git worktrees")
            else:
                print("DRY-RUN: skip-build; use prebuilt rtt/base/next images")
            stage_images(matrix["bench"], staged, dry_run=True)
            invoke_runner(matrix["bench"], stages, dry_run=True)
            return 0

        toolchain = None
        if not args.skip_build:
            toolchain = validate_toolchain(args.toolchain_bin)

        if args.skip_build:
            staged = {
                name: find_prebuilt(REPO_ROOT, name)
                for name in ("rtt", "base", "next")
            }
        else:
            staged: dict[str, dict[str, Path]] = {}
            with tempfile.TemporaryDirectory(prefix="hil_acceptance_") as temp:
                parent = Path(temp)
                worktrees: list[Path] = []
                try:
                    rtt = add_temp_worktree(REPO_ROOT, parent, "rtt")
                    worktrees.append(rtt)
                    staged["rtt"] = build_image(rtt, toolchain, "rtt", parent, "rtt")

                    base = add_temp_worktree(REPO_ROOT, parent, "base")
                    worktrees.append(base)
                    staged["base"] = build_image(base, toolchain, "", parent, "base")

                    next_worktree = add_temp_worktree(REPO_ROOT, parent, "next")
                    worktrees.append(next_worktree)
                    old_version, new_version = bump_version_in_worktree(next_worktree)
                    print(f"temporary next image BUILD_VERSION: {old_version} -> {new_version}")
                    staged["next"] = build_image(next_worktree, toolchain, "", parent, "next")

                    stage_images(matrix["bench"], staged)
                    invoke_runner(matrix["bench"], stages)
                finally:
                    for worktree in reversed(worktrees):
                        subprocess.run(
                            ["git", "-C", str(REPO_ROOT), "worktree", "remove", "--force", str(worktree)],
                            check=False,
                        )

        if args.skip_build:
            stage_images(matrix["bench"], staged)
            invoke_runner(matrix["bench"], stages)
        return 0

    except HilError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except (OSError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
