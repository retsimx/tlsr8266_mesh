
TARGET=lightblemesh

CC = ./toolchain/tc32/bin/tc32-elf-gcc
LD = ./toolchain/tc32/bin/tc32-elf-ld
CP = ./toolchain/tc32/bin/tc32-elf-objcopy

CCFLAGS = -O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -DMCU_STARTUP_8266 -D__PROJECT_LIGHT_8266__=1 -DPROVISIONING_ENABLE -I ./sdk/ -ffunction-sections -fdata-sections

LDFLAGS = --gc-sections -T ./sdk/boot.link

LIB = ./toolchain/tc32/lib/gcc/tc32-elf/4.5.1.tc32-elf-1.5/libgcc.a

BUILD_DIR = _build

STARTUP_SRC = ./sdk/cstartup_8266.S
STARTUP_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(STARTUP_SRC:%.S=%.o)))

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET): $(STARTUP_OBJ)
	bash toolchain/rust2c.sh
	$(LD) $(LDFLAGS) -o $@ rust/target/thumbv6m-none-eabi/release/deps/*.o $(CPPOBJS) $(OBJS) $(DRIVERS_OBJS) $(COMMON_OBJS) $(VENDORS_OBJS) $(SDK_COMMON_OBJS) $(STARTUP_OBJ) $(DIVMOD_OBJ) $(LIB)

# Startup.
$(BUILD_DIR)/asm/cstartup_8266.o : $(STARTUP_SRC)
	bash sdk/increment_version.sh
	mkdir -p $(BUILD_DIR)/asm
	$(CC) -c $(CCFLAGS) $< -o $@

.PHONE: all
all: $(STARTUP_OBJ)
	@echo $(DRIVERS_OBJ)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
