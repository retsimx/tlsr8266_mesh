
TARGET = lightblemesh

CC  = ./toolchain/tc32/bin/tc32-elf-gcc
AS  = ./toolchain/tc32/bin/tc32-elf-as
LD  = ./toolchain/tc32/bin/tc32-elf-ld
CP  = ./toolchain/tc32/bin/tc32-elf-objcopy

LLC ?= ../../llvm/build/bin/llc

CCFLAGS = -O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -DMCU_STARTUP_8266 -D__PROJECT_LIGHT_8266__=1 -DPROVISIONING_ENABLE -I ./sdk/ -ffunction-sections -fdata-sections

LLC_FLAGS = -march=thumb -mcpu=arm9 -mattr=+soft-float,-v6

LDFLAGS = --gc-sections -T ./sdk/boot.link

LIB = ./toolchain/tc32/lib/gcc/tc32-elf/4.5.1.tc32-elf-1.5/libgcc.a

BUILD_DIR  = _build
RUST_DEPS  = rust/target/thumbv6m-none-eabi/release/deps

STARTUP_SRC = ./sdk/cstartup_8266.S
STARTUP_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(STARTUP_SRC:%.S=%.o)))

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET): $(STARTUP_OBJ)
	@[ -f .env ] && . .env || :; \
	 _LLC=$${LLC:-$(LLC)}; \
	 cd rust && \
	 RUSTFLAGS="--emit=llvm-ir" cargo build --color=always --release -Z build-std=core --all-features --target thumbv6m-none-eabi && \
	 rm -f target/thumbv6m-none-eabi/release/deps/*.s target/thumbv6m-none-eabi/release/deps/*.o && \
	 for i in target/thumbv6m-none-eabi/release/deps/*.ll; do \
	   bname="$${i%.*}"; \
	   echo "Compiling $${bname}.ll"; \
	   ($$_LLC $(LLC_FLAGS) $$bname.ll -o $$bname.s && ../$(AS) -o $$bname.o $$bname.s) & \
	 done; \
	 wait
	$(LD) $(LDFLAGS) -o $@ $(RUST_DEPS)/*.o $(STARTUP_OBJ) $(LIB)

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
	rm -f $(RUST_DEPS)/*.s $(RUST_DEPS)/*.o
