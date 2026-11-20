# Firmware build front end.
#
# Build flags (Cargo features, the LLVM optimisation pipeline, llc/ld arguments),
# tool resolution, exact IR module selection, startup-object caching and the
# incremental link manifest all live in sdk/build_firmware.py. This Makefile
# stays a thin wrapper so there is exactly one source of truth for the build.

TARGET = lightblemesh

BUILD_DIR  = _build

# Cargo feature selection, e.g. `make FEATURES=rtt`. Forwarded verbatim to the
# driver, which normalises it and keeps one coherent IR module set per selection.
FEATURES ?=

# $(BUILD_DIR)/$(TARGET) is not a file make can reason about: always
# re-evaluate so a selection change can never leave a stale $(TARGET) in place.
.PHONY: all $(BUILD_DIR)/$(TARGET) $(BUILD_DIR)/$(TARGET).bin bump-version
all: $(BUILD_DIR)/$(TARGET).bin
$(BUILD_DIR)/$(TARGET):
	python3 sdk/build_firmware.py $(if $(FEATURES),--features "$(FEATURES)",) $(if $(LLC),--llc "$(LLC)",) $(if $(LLVM_LINK),--llvm-link "$(LLVM_LINK)",) $(if $(OPT),--opt "$(OPT)",)
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	@:
bump-version:
	bash sdk/increment_version.sh

.PHONY: stack-check
# Static stack/RAM gate: reads the linked ELF's symbols and fails on call cycles
# or a main/IRQ margin below 400 B. Does not rebuild the firmware itself.
stack-check:
	@if [ ! -f $(BUILD_DIR)/$(TARGET) ]; then \
		echo "stack-check: $(BUILD_DIR)/$(TARGET) not found; run 'make' first" >&2; \
		exit 2; \
	fi
	python3 sdk/stack_analysis.py --elf $(BUILD_DIR)/$(TARGET)

# Flashable image must fit the 128 KiB (131072 B) application slot.
IMAGE_SIZE_LIMIT = 131072

.PHONY: image-size-check
# Fails if the built .bin exceeds IMAGE_SIZE_LIMIT. Does not rebuild.
image-size-check:
	@if [ ! -f $(BUILD_DIR)/$(TARGET).bin ]; then \
		echo "image-size-check: $(BUILD_DIR)/$(TARGET).bin not found; run 'make' first" >&2; \
		exit 2; \
	fi
	@size=$$(wc -c < $(BUILD_DIR)/$(TARGET).bin); \
	echo "firmware image: $$size bytes (limit $(IMAGE_SIZE_LIMIT))"; \
	if [ $$size -gt $(IMAGE_SIZE_LIMIT) ]; then \
		echo "ERROR: firmware image $$size B exceeds $(IMAGE_SIZE_LIMIT) B" >&2; \
		exit 1; \
	fi

.PHONY: check
# All non-rebuilding static gates.
check: stack-check image-size-check

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -rf rust/target/features

.PHONY: distclean
distclean: clean
	cd rust && cargo clean
	rm -rf rust/target/features
