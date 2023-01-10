
TARGET=lightblemesh

CBE = /home/lewis/Projects/llvm-cbe/build/tools/llvm-cbe/llvm-cbe
CC = ./toolchain/tc32/bin/tc32-elf-gcc
LD = ./toolchain/tc32/bin/tc32-elf-ld
CP = ./toolchain/tc32/bin/tc32-elf-objcopy

CCFLAGS = -O2 -fshort-wchar -fms-extensions -finline-small-functions -fpack-struct -fshort-enums -Wall -std=gnu99 -DMCU_STARTUP_8266 -D__PROJECT_LIGHT_8266__=1 -DPROVISIONING_ENABLE -I ./sdk/ -ffunction-sections -fdata-sections

LDFLAGS = --gc-sections -T ./sdk/boot.link

LIB = ./sdk/proj_lib/libble_app_8266.a ./sdk/proj_lib/libsoft-fp.a

BUILD_DIR = _build

DRIVERS_SRC = \
	./sdk/proj/mcu/clock.c \
	./sdk/proj/drivers/flash.c \
	./sdk/proj/drivers/flash_mesh_extend.c
DRIVERS_OBJS = $(addprefix $(BUILD_DIR)/drivers/, $(notdir $(DRIVERS_SRC:%.c=%.o)))

SDK_COMMON_SRC = \
	./sdk/proj/common/compatibility.c \
	./sdk/proj/common/string.c
#	./sdk/proj/common/printf.c
SDK_COMMON_OBJS = $(addprefix $(BUILD_DIR)/sdk-common/, $(notdir $(SDK_COMMON_SRC:%.c=%.o)))

VENDORS_SRC = \
	./sdk/vendor/common/app_att_light.c \
	./sdk/vendor/common/common.c
VENDORS_OBJS = $(addprefix $(BUILD_DIR)/vendor/common/, $(notdir $(VENDORS_SRC:%.c=%.o)))

STARTUP_SRC = ./sdk/proj/mcu_spec/cstartup_8266.S
STARTUP_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(STARTUP_SRC:%.S=%.o)))

DIVMOD_SRC = #./sdk/div_mod.S
DIVMOD_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(DIVMOD_SRC:%.S=%.o)))

SRC = \
	src/main_light.c \
	src/vendor_att_light.c
OBJS = $(addprefix $(BUILD_DIR)/, $(SRC:%.c=%.o))

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET): $(OBJS) $(DRIVERS_OBJS) $(COMMON_OBJS) $(VENDORS_OBJS) $(SDK_COMMON_OBJS) $(STARTUP_OBJ) $(DIVMOD_OBJ) $(OBJS)
	bash toolchain/rust2c.sh
	$(LD) $(LDFLAGS) -o $@ rust/target/i686-unknown-linux-gnu/release/deps/*.o $(CPPOBJS) $(OBJS) $(DRIVERS_OBJS) $(COMMON_OBJS) $(VENDORS_OBJS) $(SDK_COMMON_OBJS) $(STARTUP_OBJ) $(DIVMOD_OBJ) $(LIB)

# Drivers.
$(BUILD_DIR)/drivers/%.o: ./sdk/proj/drivers/%.c
	mkdir -p $(BUILD_DIR)/drivers
	$(CC) -c $(CCFLAGS) -o $@ $<

$(BUILD_DIR)/drivers/%.o: ./sdk/proj/mcu/%.c
	mkdir -p $(BUILD_DIR)/drivers
	$(CC) -c $(CCFLAGS) -o $@ $<

# SDK common stuff.
$(BUILD_DIR)/sdk-common/%.o: ./sdk/proj/common/%.c
	mkdir -p $(BUILD_DIR)/sdk-common
	$(CC) -c $(CCFLAGS) -o $@ $<

# Vendor stuff.
$(BUILD_DIR)/vendor/common/%.o: ./sdk/vendor/common/%.c
	mkdir -p $(BUILD_DIR)/vendor/common
	$(CC) -c $(CCFLAGS) -o $@ $<

# Startup.
$(BUILD_DIR)/asm/cstartup_8266.o : $(STARTUP_SRC)
	mkdir -p $(BUILD_DIR)/asm
	$(CC) -c $(CCFLAGS) $< -o $@

# div_mod.
$(BUILD_DIR)/asm/div_mod.o : $(DIVMOD_SRC)
	mkdir -p $(BUILD_DIR)/asm
	$(CC) -c $(CCFLAGS) $< -o $@

# App sources.
$(BUILD_DIR)/%.o: %.c
	mkdir -p $(BUILD_DIR)/src
	$(CC) -c $(CCFLAGS) -o $@ $<

.PHONE: all
all: $(DRIVERS_OBJS) $(STARTUP_OBJ)
	@echo $(DRIVERS_OBJ)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)