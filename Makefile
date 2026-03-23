
TARGET = lightblemesh

CC  = ./toolchain/tc32/bin/tc32-elf-gcc -B ./toolchain/tc32/bin/tc32-elf-
AS  = ./toolchain/tc32/bin/tc32-elf-as
LD  = ./toolchain/tc32/bin/tc32-elf-ld
CP  = ./toolchain/tc32/bin/tc32-elf-objcopy

LLC ?= ../../llvm/build/bin/llc
LLVM_LINK ?= ../../llvm/build/bin/llvm-link
OPT ?= ../../llvm/build/bin/opt

# Whole-module IR optimization pipeline.
#
# Phase 1 (module-level): ipsccp (inter-procedural sparse conditional constant
#   propagation) propagates constant arguments across call boundaries, making
#   branches deterministic and enabling downstream dead code elimination.
#   deadargelim then removes any function parameters made dead by ipsccp.
#
# Phase 2 (function-level): SROA, mem-to-reg, instcombine, CFG simplification,
#   DCE, loop rotation + LICM, aggressive DCE, dead store elimination.
#   Running before inlining shrinks functions so more become eligible.
#
# Phase 3 (module-level): always-inline (honours #[inline(always)]), then
#   cgscc(inline) with threshold=50 IR instructions. On Cortex-M0/TC32 every
#   call costs ~10+ cycles (no branch predictor, no return-address stack), so
#   inlining functions up to ~50 IR instructions recovers that overhead while
#   staying net-positive on code size. Above threshold=50 size starts growing.
#
# Phase 4 (module-level): second ipsccp+deadargelim round after inlining.
#   Inlining exposes new constants at call sites that weren't visible before —
#   a second ipsccp pass propagates these and enables further dead-code removal.
#
# Phase 5 (function-level): repeat the same safe passes on the fully inlined
#   and constant-propagated code so merged bodies are fully cleaned up.
OPT_INLINE_THRESHOLD = 50
OPT_SAFE = sroa,mem2reg,instcombine<no-verify-fixpoint>,simplifycfg,dce,loop-simplify,lcssa,loop-rotate,loop-mssa(licm),instcombine<no-verify-fixpoint>,simplifycfg,adce,dse
OPT_PASSES = ipsccp,deadargelim,function($(OPT_SAFE)),always-inline,cgscc(inline),ipsccp,deadargelim,function($(OPT_SAFE))

CCFLAGS = -O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -DMCU_STARTUP_8266 -D__PROJECT_LIGHT_8266__=1 -DPROVISIONING_ENABLE -I ./sdk/ -ffunction-sections -fdata-sections

LLC_FLAGS = -march=thumb -mcpu=arm9 -mattr=+soft-float,-v6

LDFLAGS = --gc-sections -T ./sdk/boot.link

LIB = ./toolchain/tc32/lib/gcc/tc32-elf/4.5.1.tc32-elf-1.5/libgcc.a

BUILD_DIR  = _build
RUST_DEPS  = rust/target/thumbv6m-none-eabi/release/deps

STARTUP_SRC = ./sdk/cstartup_8266.S
STARTUP_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(STARTUP_SRC:%.S=%.o)))

RUST_SRCS = $(shell find rust/src -name '*.rs') rust/Cargo.toml rust/Cargo.lock

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET): $(STARTUP_OBJ) $(RUST_SRCS)
	@[ -f .env ] && . .env || :; \
	 _LLC=$${LLC:-$(LLC)}; \
	 _LLVM_LINK=$${LLVM_LINK:-$(LLVM_LINK)}; \
	 _OPT=$${OPT:-$(OPT)}; \
	 cd rust && \
	 RUSTFLAGS="--emit=llvm-ir" cargo build --lib --color=always --release -Z build-std=core --all-features --target thumbv6m-none-eabi && \
	 echo "Linking IR modules..." && \
	 $$_LLVM_LINK target/thumbv6m-none-eabi/release/deps/*.ll -o target/thumbv6m-none-eabi/release/deps/merged.bc && \
	 echo "Optimising IR..." && \
	 $$_OPT -passes="$(OPT_PASSES)" -inline-threshold=$(OPT_INLINE_THRESHOLD) target/thumbv6m-none-eabi/release/deps/merged.bc -o target/thumbv6m-none-eabi/release/deps/merged_opt.bc && \
	 echo "Compiling optimised IR to assembly..." && \
	 $$_LLC $(LLC_FLAGS) target/thumbv6m-none-eabi/release/deps/merged_opt.bc -o target/thumbv6m-none-eabi/release/deps/merged.s && \
	 echo "Assembling..." && \
	 ../$(AS) -o target/thumbv6m-none-eabi/release/deps/merged.o target/thumbv6m-none-eabi/release/deps/merged.s
	$(LD) $(LDFLAGS) -o $@ $(RUST_DEPS)/merged.o $(STARTUP_OBJ) $(LIB)

# Startup.
$(BUILD_DIR)/asm/cstartup_8266.o : $(STARTUP_SRC)
	bash sdk/increment_version.sh
	mkdir -p $(BUILD_DIR)/asm
	$(CC) -c $(CCFLAGS) $< -o $@

.PHONY: all
all: $(BUILD_DIR)/$(TARGET).bin

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(RUST_DEPS)/*.s $(RUST_DEPS)/*.o $(RUST_DEPS)/merged.bc $(RUST_DEPS)/merged_opt.bc

.PHONY: distclean
distclean: clean
	cd rust && cargo clean
	rm -f $(RUST_DEPS)/*.ll
