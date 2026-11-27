# Firmware build front end.
#
# Build flags (Cargo features, the LLVM optimisation pipeline, llc/ld arguments),
# tool resolution, exact IR module selection and the incremental link manifest
# all live in sdk/build_firmware.py. This Makefile stays a thin wrapper so there
# is exactly one source of truth for the build.

TARGET = lightblemesh

BUILD_DIR  = _build

# Cargo feature selection, e.g. `make FEATURES=rtt`. Forwarded verbatim to the
# driver, which normalises it and keeps one coherent IR module set per selection.
FEATURES ?=

# Normalise the Make-side feature list exactly like the build driver. This makes
# RTT a single exact feature selection, rather than a substring or mixed-feature match.
comma := ,
empty :=
space := $(empty) $(empty)
FEATURE_WORDS := $(sort $(subst $(comma), ,$(strip $(FEATURES))))
FEATURE_SLUG := $(if $(FEATURE_WORDS),$(subst $(space),-,$(FEATURE_WORDS)),default)

ifeq ($(strip $(FEATURES)),)
FEATURE_PROFILE = default
else
ifeq ($(FEATURE_WORDS),rtt)
FEATURE_PROFILE = rtt
else
ifeq ($(FEATURE_WORDS),default)
FEATURE_PROFILE = default
else
FEATURE_PROFILE = $(FEATURE_SLUG)
endif
endif
endif

# $(BUILD_DIR)/$(TARGET) is not a file make can reason about: always
# re-evaluate so a selection change can never leave a stale $(TARGET) in place.
.PHONY: all $(BUILD_DIR)/$(TARGET) $(BUILD_DIR)/$(TARGET).bin bump-version
all: $(BUILD_DIR)/$(TARGET).bin
$(BUILD_DIR)/$(TARGET):
	python3 sdk/build_firmware.py $(if $(FEATURES),--features "$(FEATURES)",) $(if $(LLC),--llc "$(LLC)",) $(if $(LLVM_LINK),--llvm-link "$(LLVM_LINK)",) $(if $(OPT),--opt "$(OPT)",) $(if $(filter-out default undefined,$(origin LD)),--ld "$(LD)",) $(if $(OBJCOPY),--objcopy "$(OBJCOPY)",)
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	@:
bump-version:
	bash sdk/increment_version.sh

STACK_BASELINE = $(BUILD_DIR)/stack-baseline-$(FEATURE_PROFILE).json

.PHONY: stack-check
# Static stack/RAM gate: reads the linked ELF's symbols and fails on call cycles
# or a main/IRQ margin below the analyzer's 400 B minimum. It also retains a
# separate baseline JSON for default and RTT feature selections. Does not rebuild.
stack-check:
	@if [ ! -f $(BUILD_DIR)/$(TARGET) ]; then \
		echo "stack-check: $(BUILD_DIR)/$(TARGET) not found; run 'make' first" >&2; \
		exit 2; \
	fi
	python3 sdk/stack_analysis.py --strict --elf $(BUILD_DIR)/$(TARGET) --baseline-json $(STACK_BASELINE)

# Every feature selection retains the 128 KiB (131072 B) application-slot ceiling.
IMAGE_SIZE_LIMIT = 131072

.PHONY: image-size-check
# Fails if any build's application-slot image exceeds the flash limit. Does not rebuild.
image-size-check:
	@if [ ! -f $(BUILD_DIR)/$(TARGET).bin ]; then \
		echo "image-size-check: $(BUILD_DIR)/$(TARGET).bin not found; run 'make' first" >&2; \
		exit 2; \
	fi
	@size=$$(wc -c < $(BUILD_DIR)/$(TARGET).bin); \
	echo "firmware image: $$size bytes (profile $(FEATURE_PROFILE), application-slot limit $(IMAGE_SIZE_LIMIT) B)"; \
	if [ $$size -gt $(IMAGE_SIZE_LIMIT) ]; then \
		echo "ERROR: firmware image $$size B exceeds $(IMAGE_SIZE_LIMIT) B application-slot ceiling" >&2; \
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
