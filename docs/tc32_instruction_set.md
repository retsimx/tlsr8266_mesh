# TLSR8266 TC32 Instruction Set (Recovered)

- **Target scope:** TLSR8266 firmware build mode (`.code 16`)
- **Toolchain:** Telink GNU assembler/objdump 2.20, GCC 4.5.1
  (`tc32-elf-1.5`)
- **Evidence date:** 2026-11-20
- **Status:** executable evidence from the shipped vendor tools; not an
  official Telink architecture manual

## Scope and Limits

This reference reconstructs the TC32 syntax and encodings that the shipped
Telink binaries advertise for the TLSR8266 firmware's 16-bit build mode.
`sdk/cstartup_8266.S` contains `.code 16`, and the custom LLVM backend emits
Thumb-like 16-bit TC32 code for that same target.

The vendor tools expose no `TLSR8266`, `8266`, or `-mcpu=...` selector. They
expose one architecture name, `tc32`, plus a generic `-mtc32` GCC switch.
Consequently, the binaries do **not** identify a distinct instruction subset
for TLSR8253, TLSR8258, TLSR8266, or another TC32 MCU.

The strongest available 8266-specific scope is therefore:

1. instructions accepted in `.code 16` by the shipped assembler;
2. encodings recognized by the shipped TC32 disassembler;
3. instructions observed in `sdk/cstartup_8266.S` and the previously supplied
   TC32 `libgcc.a` (since removed from the repository).

This is an executable syntax/encoding specification, not a complete
architectural manual. The binaries do not establish:

- cycle counts, pipeline behavior, or interrupt latency;
- memory-ordering or alignment guarantees;
- privilege/fault behavior;
- the semantics of raw reserved or undocumented encodings;
- silicon support for an instruction omitted from the tool tables.

An absent mnemonic proves only that the shipped assembler/disassembler has no
supported spelling for it. It cannot prove that the silicon contains no
undocumented opcode.

## Security and Licensing Note

The file records facts recovered from binaries already present in this
repository. It does not supply the GPL-corresponding assembler/source package
and is not a substitute for it.

## Evidence

The reconstructed tables come from unstripped x86-64 vendor executables:

| Binary | SHA-256 | Relevant symbol / result |
|---|---|---|
| `toolchain/tc32/bin/tc32-elf-as` | `4787d1e6ee1284439a470ca9bb9802dd892f2d86a575bc38ce6ffd5cc52ce4e1` | `insns`, 6,272 bytes, 112 generic parser records |
| `toolchain/tc32/bin/tc32-elf-objdump` | `8f5c7879a32b82475714d9cef8fbf7bf94d8a685b377ca3c07dc9f6193713f8f` | `tc32_opcodes`, 1,608 bytes; `thumb32_opcodes`, 128 bytes |
| `toolchain/tc32/bin/tc32-elf-gcc` | `a37f0013e2a3359624384d0188c3dce8f65ff83421c8422d24de6d54118a1dc8` | 4.5.1.tc32-elf-1.5 host driver |
| `toolchain/tc32/lib/gcc/tc32-elf/4.5.1.tc32-elf-1.5/libgcc.a` (removed) | `064fec41e011be6c05d6bc9253e706425628f597134e2eec0ceb4e7990c3d348` | 139 TC32 runtime objects used as emitted-code evidence; the archive was removed after the analysis |

The disassembler advertises `tc32` as both `elf32-littletc32` and
`elf32-little`. GCC defines:

```text
__tc32_16__ 1
__tc32__ 1
__TC32BEL__ 1
__TC32EL__ 1
```

These are target-wide definitions, not TLSR8266-specific CPU feature flags.

## Executive Findings

1. The 8266 firmware mode is a 16-bit TC32 ISA, with one used 32-bit
   instruction family, `tjl` (long call). The decoder also recognizes
   `tjlex` (long call/exchange), but the assembler's 8266 mnemonic table does
   not expose `tjlex`.
2. There is no `WFI`, `WFE`, `SEV`, `YIELD`, `sleep`, `halt`, `suspend`,
   `LDREX`, `STREX`, `CLREX`, `SWP`, `CAS`, or other atomic/wait mnemonic in
   the TC32 decoder or 16-bit assembler set.
3. The ordinary tool-supported memory instructions are `tloadr*`, `tstorer*`,
   `tloadm`, and `tstorem`. They are not documented by the tools as exclusive
   or atomic operations.
4. The tool tables support a single CPU status interface through
   `tmcsr`/`tmrcs` and `tmssr`/`tmrss`. None of those instructions is a CPU-idle
   primitive.
5. Do not blindly paste `tc32-elf-objdump` output back into `tc32-elf-as`.
   Several decoder spellings use an `s` suffix (`tands`, `tadds`, `tmuls`,
   etc.) that the assembler rejects in `.code 16`.
6. `tserv imm8` is the vendor tools' SWI/SVC-class service instruction. The
   assembler internally routes it through `do_t_swi`, but the shipped binaries
   disagree on its opcode: objdump recognizes `0xCFxx` as `tserv`, while the
   assembler emits `0xDFxx`, which that objdump decodes as `tloadm`. The
   matching silicon opcode is therefore unresolved.

## Table Extraction

`tc32_opcodes` is 1,608 bytes: 67 records of 24 bytes. The first 66 records
are actual or explicit-undefined patterns; the final record is the zero
terminator.

Each record is laid out as:

```text
u64  architecture/feature word
u16  value
u16  mask
u64  pointer to disassembly format
```

The 16-bit match rule, in table order, is:

```text
(word & mask) == value
```

`thumb32_opcodes` is 128 bytes: four 32-byte records consisting of two
patterns, an undefined catch-all, and a zero terminator. A logical 32-bit
word is the first halfword followed by the second halfword in instruction
stream order; both halfwords are little-endian on disk.

The decoder tables are preferable to the 112-entry assembler `insns` table
for target scoping. `insns` contains inherited ARM/Thumb syntax; many entries
are rejected under `.code 16` and some emit encodings that this TC32 decoder
cannot recognize.

### Reproducing the Symbols

```sh
nm -a -S -n toolchain/tc32/bin/tc32-elf-as \
  | grep ' insns$'
nm -a -S -n toolchain/tc32/bin/tc32-elf-objdump \
  | grep -E ' tc32_opcodes$| thumb32_opcodes$'
```

The complete 16-bit and 32-bit patterns are listed below. A changed hash
means the tables must be re-extracted before trusting this document.

## Register and Operand Model

| Name | Encoding use |
|---|---|
| `r0`-`r7` | low-register fields (3 bits each) |
| `r8`-`r15` | available through high-register `tadd`/`tcmp`/`tmov` and `tjex` |
| `sp` | alias of `r13` |
| `lr` | alias of `r14`; push list and link target |
| `pc` / `r15` | available through high-register operations, branches, and pop/return |

Stack/list encodings:

- `tpush` permits a subset of `r0`-`r7` plus `lr`;
- `tpop` and `treti` encode a subset of low registers plus `pc`;
- `tloadm` and `tstorem` encode a subset of `r0`-`r7` plus a low-register
  base with `!`.

The decoder-table format notation is:

| Marker | Meaning |
|---|---|
| `%a-br` | unsigned register field bits `a..b` |
| `%D`, `%S` | encoded destination/source register, including high registers |
| `%0-7d` | raw 8-bit unsigned immediate |
| `%6-10d` | raw 5-bit unsigned immediate |
| `%...W` | word-scaled offset (`field * 4`) |
| `%...H` | halfword-scaled offset (`field * 2`) |
| `%...B` | signed branch displacement; target is halfword-aligned |
| `%N`, `%M`, `%O` | register-list encodings |
| `%C` | decoder output includes `s` |
| `%c` | condition suffix from surrounding decoder state, or nothing |

Decoder spellings containing `%C` are not automatically valid assembler
input. For example, assembling `tand r1, r2` emits an encoding that objdump
prints as `tands r1, r2`, but `tands` itself is rejected in `.code 16`.

## TLSR8266 `.code 16` Instruction Families

### Status and Return

| Assembly form | Size | Operand range | Encoding note |
|---|---:|---|---|
| `tmcsr Rd` | 16 | `Rd = r0`-`r7` | move to current processor status |
| `tmrcs Rd` | 16 | `Rd = r0`-`r7` | read current processor status |
| `tmssr Rd` | 16 | `Rd = r0`-`r7` | write saved processor status |
| `tmrss Rd` | 16 | `Rd = r0`-`r7` | read saved processor status |
| `treti {list}` | 16 | low-register subset plus `pc` | startup/IRQ epilogue uses `treti {r15}` |
| `tserv imm8` | 16 | `0..255` | SWI/SVC-style service trap; assembler/disassembler opcode disagreement means do not use without HIL validation |

`tserv` is not an idle, atomic, or synchronization instruction. As with an
SWI/SVC, the encoded 8-bit value is a service/exception selector; exception
entry, saved state, and dispatch are processor-platform behavior. A handler may
recover the selector from the instruction at the saved return PC. The shipped
tools do not reveal the TLSR8266 service-vector address or handler contract,
and the image format occupies offset `0x8` with the `TLNK` marker. No current
project code uses `tserv`.

The status semantics are corroborated by `sdk/cstartup_8266.S`: reset uses
`tmcsr` while selecting IRQ/SVC stack state; the IRQ wrapper saves with
`tmrss` and restores with `tmssr`.

### Register ALU

All registers in these two-operand encodings are `r0`-`r7`.

| Assembly form | Exact behavior class | Size |
|---|---|---:|
| `tand Rd, Rs` | bitwise AND, updates represented flags | 16 |
| `txor Rd, Rs` | XOR, updates represented flags | 16 |
| `tshftl Rd, Rs` | logical left shift `Rd` by register value in `Rs` | 16 |
| `tshftr Rd, Rs` | logical right shift by register | 16 |
| `tasr Rd, Rs` | arithmetic right shift by register | 16 |
| `taddc Rd, Rs` | add with carry | 16 |
| `tsubc Rd, Rs` | subtract with carry/borrow | 16 |
| `trotr Rd, Rs` | rotate right by register | 16 |
| `tnand Rd, Rs` | NAND | 16 |
| `tneg Rd, Rs` | negate `Rs` into `Rd` | 16 |
| `tcmp Rd, Rs` | compare | 16 |
| `tcmpn Rd, Rs` | compare negative / CMN-class operation | 16 |
| `tor Rd, Rs` | OR | 16 |
| `tmul Rd, Rs` | multiply | 16 |
| `tbclr Rd, Rs` | bit clear | 16 |
| `tmovn Rd, Rs` | bitwise NOT / move-not | 16 |

`tnands`, `tcmps`, `tcmpns`, and `tmovs` are accepted aliases for their base
forms. Other explicit `s` suffixes, including `tands`, `txors`, `taddcs`,
`tadds`, `tmuls`, `tmovns`, and `tnegs`, are rejected in `.code 16`.

### Add, Subtract, and Immediate Operations

| Assembly form | Operand domains | Size |
|---|---|---:|
| `tadd Rd, Rs` | `Rd,Rs = r0`-`r7`; assembler forms `Rd += Rs` | 16 |
| `tsub Rd, Rs` | `Rd,Rs = r0`-`r7`; assembler forms `Rd -= Rs` | 16 |
| `tadd Rd, Rn, Rm` | all `r0`-`r7` | 16 |
| `tsub Rd, Rn, Rm` | all `r0`-`r7` | 16 |
| `tadd Rd, Rn, #imm` | `Rd,Rn = r0`-`r7`; `imm = 0..7` | 16 |
| `tsub Rd, Rn, #imm` | `Rd,Rn = r0`-`r7`; `imm = 0..7` | 16 |
| `tadd Rd, #imm` | `Rd = r0`-`r7`; `imm = 0..255` | 16 |
| `tsub Rd, #imm` | `Rd = r0`-`r7`; `imm = 0..255` | 16 |
| `tcmp Rd, #imm` | `Rd = r0`-`r7`; `imm = 0..255` | 16 |
| `tmov Rd, #imm` | `Rd = r0`-`r7`; `imm = 0..255` | 16 |
| `tadd sp, #imm` | `imm` aligned to 4, `0..508`; negative input may select `tsub` | 16 |
| `tsub sp, #imm` | `imm` aligned to 4, `0..508` | 16 |
| `tadd Rd, pc, #imm` | `Rd = r0`-`r7`; `imm = 0..1020`, aligned to 4 | 16 |
| `tadd Rd, sp, #imm` | `Rd = r0`-`r7`; `imm = 0..1020`, aligned to 4 | 16 |
| `tadd Rd, Rs` high-register form | all `r0`-`r15` combinations accepted by assembler | 16 |
| `tcmp Rd, Rs` high-register form | all `r0`-`r15` combinations accepted by assembler | 16 |
| `tmov Rd, Rs` high-register form | all `r0`-`r15` combinations accepted by assembler | 16 |

The two-operand `tmov Rd, Rs` spelling may be assembled as
`tadd Rd, Rs, #0`; this is why objdump output is not a one-to-one spelling of
assembler source.

### Shifts and Rotation

| Assembly form | Immediate domain | Size |
|---|---|---:|
| `tshftl Rd, Rs` | register-shift form, low registers | 16 |
| `tshftl Rd, Rs, #n` | `n = 0..31`; zero may canonicalize to move | 16 |
| `tshftr Rd, Rs` | register-shift form, low registers | 16 |
| `tshftr Rd, Rs, #n` | `n = 0..32`; decoder prints zero field as `#32` | 16 |
| `tasr Rd, Rs` | register-shift form, low registers | 16 |
| `tasr Rd, Rs, #n` | `n = 0..32`; decoder prints zero field as `#32` | 16 |
| `trotr Rd, Rs` | register rotation amount, low registers | 16 |

There is no immediate-rotate form in the recovered table.

### Loads and Stores

All register-index/base operands below are low registers unless shown as
`pc`/`sp`.

| Assembly form | Offset range | Size |
|---|---|---:|
| `tloadr Rt, [Rb, Ri]` | register index | 16 |
| `tstorer Rt, [Rb, Ri]` | register index | 16 |
| `tloadrb Rt, [Rb, Ri]` | register index | 16 |
| `tstorerb Rt, [Rb, Ri]` | register index | 16 |
| `tloadrh Rt, [Rb, Ri]` | register index | 16 |
| `tstorerh Rt, [Rb, Ri]` | register index | 16 |
| `tloadrsb Rt, [Rb, Ri]` | signed byte load | 16 |
| `tloadrsh Rt, [Rb, Ri]` | signed halfword load | 16 |
| `tloadr Rt, [Rb, #n]` | `n = 0..124`, aligned to 4 | 16 |
| `tstorer Rt, [Rb, #n]` | `n = 0..124`, aligned to 4 | 16 |
| `tloadrb Rt, [Rb, #n]` | `n = 0..31` | 16 |
| `tstorerb Rt, [Rb, #n]` | `n = 0..31` | 16 |
| `tloadrh Rt, [Rb, #n]` | `n = 0..62`, aligned to 2 | 16 |
| `tstorerh Rt, [Rb, #n]` | `n = 0..62`, aligned to 2 | 16 |
| `tloadr Rt, [pc, #n]` | `n = 0..1020`, aligned to 4 | 16 |
| `tloadr Rt, symbol` | assembler-generated PC-relative literal form when the symbol is in range | 16 |
| `tloadr Rt, [sp, #n]` | `n = 0..1020`, aligned to 4 | 16 |
| `tstorer Rt, [sp, #n]` | `n = 0..1020`, aligned to 4 | 16 |

The assembler table contains conditional signed-load spellings such as
`tloadreqsh` and `tloadrnesb`. They are rejected in `.code 16` with "Thumb
does not support conditional execution" and are not part of the 8266 set.

### Multiple Registers and Stack

| Assembly form | Valid registers | Size |
|---|---|---:|
| `tpush {list}` | subset of `r0`-`r7`, optionally `lr` | 16 |
| `tpop {list}` | subset of `r0`-`r7`, optionally `pc` | 16 |
| `tstorem Rb!, {list}` | `Rb = r0`-`r7`; list subset of `r0`-`r7` | 16 |
| `tloadm Rb!, {list}` | same | 16 |
| `tstoremea Rb!, {list}` | accepted alias of `tstorem` | 16 |
| `ldmfd Rb!, {list}` | accepted alias of `tloadm` | 16 |

### Branch, Call, and Exchange

| Assembly form | Encoded displacement/target | Size |
|---|---|---:|
| `tj label` | unconditional, 11-bit signed halfword displacement | 16 |
| `tjeq/ne/cs/cc/mi/pl/vs/vc/hi/ls/ge/lt/gt/le label` | condition, 8-bit signed halfword displacement | 16 |
| `tjex Rn` | indirect branch/exchange, `Rn = r0`-`r15` | 16 |
| `tjl label` | link + 22-bit split signed halfword displacement | 32 |
| raw `tjlex` encoding | decoder-recognized link/exchange; no 8266 assembler mnemonic | 32 |

Assembler aliases `tjhs` and `tjlo` select the same encodings as `tjcs` and
`tjcc`. `tjal` assembles as plain `tj`.

Condition-code values for the conditional branch family are:

| Bits 11:8 | Mnemonic | Equivalent |
|---:|---|---|
| 0 | `tjeq` | equal |
| 1 | `tjne` | not equal |
| 2 | `tjcs` | `tjhs` |
| 3 | `tjcc` | `tjlo` |
| 4 | `tjmi` | negative |
| 5 | `tjpl` | non-negative |
| 6 | `tjvs` | overflow |
| 7 | `tjvc` | no overflow |
| 8 | `tjhi` | unsigned greater |
| 9 | `tjls` | unsigned less/equal |
| A | `tjge` | signed greater/equal |
| B | `tjlt` | signed less |
| C | `tjgt` | signed greater |
| D | `tjle` | signed less/equal |
| E | explicit undefined in the conditional-branch range | do not emit |
| F | `tserv` decoder entry takes priority | do not assume unconditional branch |

Assembler `tjal` is accepted as an alias for plain `tj`, which uses the
separate unconditional-branch encoding table (row 64), not condition value E.

## Complete 16-Bit Decoder Table

The counts are the number of 16-bit words assigned to that row after applying
the table in order. They show overlaps and priority, rather than merely
counting unmasked bits.

| # | Value / mask | Unique words | Decoder form / operand fields |
|---:|---|---:|---|
| 00 | `0780 / ff87` | 16 | `tjex%c %3-6r%x` |
| 01 | `06c0 / ffff` | 1 | `tnop%c` |
| 02 | `0000 / ffc0` | 64 | `tand%C %0-2r, %3-5r` |
| 03 | `0040 / ffc0` | 64 | `txor%C %0-2r, %3-5r` |
| 04 | `0080 / ffc0` | 64 | `tshftl%C %0-2r, %3-5r` |
| 05 | `00c0 / ffc0` | 64 | `tshftr%C %0-2r, %3-5r` |
| 06 | `0100 / ffc0` | 64 | `tasr%C %0-2r, %3-5r` |
| 07 | `0140 / ffc0` | 64 | `taddc%C %0-2r, %3-5r` |
| 08 | `0180 / ffc0` | 64 | `tsubc%C %0-2r, %3-5r` |
| 09 | `01c0 / ffc0` | 64 | `trotr%C %0-2r, %3-5r` |
| 10 | `0200 / ffc0` | 64 | `tnand%c %0-2r, %3-5r` |
| 11 | `0240 / ffc0` | 64 | `tneg%C %0-2r, %3-5r` |
| 12 | `0280 / ffc0` | 64 | `tcmp%c %0-2r, %3-5r` |
| 13 | `02c0 / ffc0` | 64 | `tcmpn%c %0-2r, %3-5r` |
| 14 | `0300 / ffc0` | 64 | `tor%C %0-2r, %3-5r` |
| 15 | `0340 / ffc0` | 64 | `tmul%C %0-2r, %3-5r` |
| 16 | `0380 / ffc0` | 64 | `tbclr%C %0-2r, %3-5r` |
| 17 | `03c0 / ffc0` | 64 | `tmovn%C %0-2r, %3-5r` |
| 18 | `6bc0 / fff8` | 8 | `tmcsr%c %0-2r` |
| 19 | `6bc8 / fff8` | 8 | `tmrcs%c %0-2r` |
| 20 | `6bd0 / fff8` | 8 | `tmssr%c %0-2r` |
| 21 | `6bd8 / fff8` | 8 | `tmrss%c %0-2r` |
| 22 | `6800 / fe00` | 512 | `treti %O` |
| 23 | `6000 / ff80` | 128 | `tadd%c sp, #%0-6W` |
| 24 | `6080 / ff80` | 128 | `tsub%c sp, #%0-6W` |
| 25 | `0700 / ff80` | 128 | `tjex%c %S%x` |
| 26 | `0400 / ff00` | 256 | `tadd%c %D, %S` |
| 27 | `0500 / ff00` | 256 | `tcmp%c %D, %S` |
| 28 | `0600 / ff00` | 255 | `tmov%c %D, %S` |
| 29 | `6400 / fe00` | 512 | `tpush%c %N` |
| 30 | `6c00 / fe00` | 512 | `tpop%c %O` |
| 31 | `e800 / fe00` | 512 | `tadd%C %0-2r, %3-5r, %6-8r` |
| 32 | `ea00 / fe00` | 512 | `tsub%C %0-2r, %3-5r, %6-8r` |
| 33 | `ec00 / fe00` | 512 | `tadd%C %0-2r, %3-5r, #%6-8d` |
| 34 | `ee00 / fe00` | 512 | `tsub%C %0-2r, %3-5r, #%6-8d` |
| 35 | `1200 / fe00` | 512 | `tstorerh%c %0-2r, [%3-5r, %6-8r]` |
| 36 | `1a00 / fe00` | 512 | `tloadrh%c %0-2r, [%3-5r, %6-8r]` |
| 37 | `1600 / f600` | 1,024 | `tloadrs%11?hb%c %0-2r, [%3-5r, %6-8r]` |
| 38 | `1000 / fa00` | 1,024 | `tstorer%10'b%c %0-2r, [%3-5r, %6-8r]` |
| 39 | `1800 / fa00` | 1,024 | `tloadr%10'b%c %0-2r, [%3-5r, %6-8r]` |
| 40 | `f000 / ffc0` | 64 | `tmov%C %0-2r, %3-5r` |
| 41 | `f000 / f800` | 1,984 | `tshftl%C %0-2r, %3-5r, #%6-10d` |
| 42 | `f800 / f800` | 2,048 | `tshftr%C %0-2r, %3-5r, %s` |
| 43 | `e000 / f800` | 2,048 | `tasr%C %0-2r, %3-5r, %s` |
| 44 | `a000 / f800` | 2,048 | `tmov%C %8-10r, #%0-7d` |
| 45 | `a800 / f800` | 2,048 | `tcmp%c %8-10r, #%0-7d` |
| 46 | `b000 / f800` | 2,048 | `tadd%C %8-10r, #%0-7d` |
| 47 | `b800 / f800` | 2,048 | `tsub%C %8-10r, #%0-7d` |
| 48 | `0800 / f800` | 2,048 | `tloadr%c %8-10r, [pc, #%0-7W]` |
| 49 | `5000 / f800` | 2,048 | `tstorer%c %0-2r, [%3-5r, #%6-10W]` |
| 50 | `5800 / f800` | 2,048 | `tloadr%c %0-2r, [%3-5r, #%6-10W]` |
| 51 | `4000 / f800` | 2,048 | `tstorerb%c %0-2r, [%3-5r, #%6-10d]` |
| 52 | `4800 / f800` | 2,048 | `tloadrb%c %0-2r, [%3-5r, #%6-10d]` |
| 53 | `2000 / f800` | 2,048 | `tstorerh%c %0-2r, [%3-5r, #%6-10H]` |
| 54 | `2800 / f800` | 2,048 | `tloadrh%c %0-2r, [%3-5r, #%6-10H]` |
| 55 | `3000 / f800` | 2,048 | `tstorer%c %8-10r, [sp, #%0-7W]` |
| 56 | `3800 / f800` | 2,048 | `tloadr%c %8-10r, [sp, #%0-7W]` |
| 57 | `7000 / f800` | 2,048 | `tadd%c %8-10r, pc, #%0-7W` |
| 58 | `7800 / f800` | 2,048 | `tadd%c %8-10r, sp, #%0-7W` |
| 59 | `d000 / f800` | 2,048 | `tstorem%c %8-10r!, %M` |
| 60 | `d800 / f800` | 2,048 | `tloadm%c %8-10r!, %M` |
| 61 | `cf00 / ff00` | 256 | `tserv%c %0-7d` |
| 62 | `ce00 / fe00` | 256 | explicit undefined instruction range |
| 63 | `c000 / f000` | 3,584 | `tj%8-11c %0-7B%X` |
| 64 | `8000 / f800` | 2,048 | `tj%c %0-10B%x` |
| 65 | `0000 / 0000` | 8,528 | catch-all undefined |

Rows 00-65 partition all 65,536 16-bit words in decoder-table priority
order. Rows 00-61 and 63-64 are 56,752 decoder-recognized non-error words;
rows 62 and 65 are undefined.

## Complete 32-Bit Decoder Table

| Value | Mask | Logical 32-bit form | Decoder result |
|---|---|---|---|
| `9000c000` | `f800d000` | first half `90xx`, second half `cxxx` | `tjlex target` |
| `90009800` | `f800f800` | first half `90xx`/`9xxx`, second half `98xx`/`9fxx` | `tjl target` |
| `00000000` | `00000000` | catch-all | undefined |

Raw round-trip examples:

```text
stream bytes  00 90 00 c0       ->  tjlex 0x4
stream bytes  00 90 00 98       ->  tjl   0x8
```

The assembler emits `tjl` with two halfwords. `tjlex` is decoder-only in the
examined 8266 build and should not be used until an actual TLSR8266 image or
HIL execution proves the path is required and valid.

## Assembler-Table Entries Excluded from the 8266 Set

The generic assembler parser inherited many ARM/Thumb entries. Do not treat
them as 8266 instructions:

| Family | `.code 16` result |
|---|---|
| explicit `s` ALU spellings (`tands`, `txors`, `taddcs`, `tadds`, `tmuls`, etc.) | rejected as unsupported in Thumb16 |
| `trsub` / `trsubs` | assembler emits unrecognized 32-bit bytes |
| `tcmpp`, `tcmpnp`, `tnandp`, `taddrl` | processor/encoding unsupported |
| `tmrs` / `tmsr` ARM status-register syntax | assembler reaches an unhandled operand path |
| `tloadreqsh`, `tloadrnesb`, and other conditional signed loads | rejected: conditional execution unsupported in Thumb |

`taddr` appears in the generic parser but could not produce a valid
PC-relative object from the tested 8266 assembly forms. Use explicit
`tadd Rd, pc, #aligned_offset` where that form is intended.

## Atomicity, Waiting, and Sleep

For this 8266 build:

- no assembler- or disassembler-recognized atomic instruction exists;
- no 16-bit or 32-bit decoder pattern names an exclusive load/store or CAS
  operation;
- `tloadr`/`tstorer` are ordinary load/store encodings;
- no `WFI`-class or CPU-idle instruction is recognized.

This supports, but does not mathematically prove, the firmware rule that
compound atomic updates must mask interrupts, perform the operation, and
restore interrupts:

```text
irq_disable()
read/modify/write
irq_restore(previous)
```

An aligned 32-bit load/store may still be indivisible at the bus/instruction
level, but that fact does not make a compound read-modify-write atomic. The
only tool-visible portable mechanism for compound state is the project's
critical-section implementation.

CPU wait behavior, if any, must not be inferred from a guessed instruction.
The TLSR8266 power-management sequence is a separate register/MMIO contract;
test it on HIL before using it as an executor idle mechanism.

## Validation Performed

- All 67 16-bit table records were parsed from the unstripped objdump binary.
- All four 32-bit table records were parsed.
- Representative encodings were round-tripped through
  `tc32-elf-objdump -D -b binary -m tc32`.
- More than 100 likely assembler spellings were tried under `.code 16`;
  unsupported inherited ARM/Thumb aliases were excluded.
- All 256 pairs of `r0`-`r15` were accepted for the high-register forms of
  `tadd`, `tcmp`, and `tmov`; `tmcsr`/`tmrcs`/`tmssr`/`tmrss` were limited
  to `r0`-`r7`.
- Immediate boundaries and alignment were probed for 3-bit, 8-bit, PC/SP
  word, byte, halfword, and stack offsets.
- The 139 objects in the previously supplied `libgcc.a` (since removed from
  the repository) were checked for emitted mnemonics. They use the same 16-bit
  TC32 family; none emits a wait or atomic instruction.
- The repository startup file was preprocessed/assembled with the actual
  Makefile `-B .../tc32-elf-` tool selection and produced `tj`, `tjl`,
  `tloadr`, `tmcsr`, `tmrss`, `tmssr`, stack, branch, and store encodings
  consistent with the recovered tables.

Silicon-level validation remains outstanding for decoder-only instructions,
power/sleep behavior, ordering, timing, and any hidden opcode not represented
in the supplied tools.
