include $(TOP_DIR)/common/base.mk

WEST_BUILD := $(WEST) $(VERBOSE_OPTION) \
	build --pristine --build-dir=$(BUILD_DIR)

TRACE_ARGS = \
	-DOVERLAY_CONFIG=overlay-ztrace.conf

RAW_ARGS = \
	-DPM_STATIC_YML_FILE=pm_raw.yml \
	-D"CONFIG_BOARD_HAS_NRF5_BOOTLOADER=n"

OTA_ARGS = \
	-DOVERLAY_CONFIG=overlay-fota.conf \
	-DPM_STATIC_YML_FILE=pm_ota.yml \
	-D"CONFIG_BOARD_HAS_NRF5_BOOTLOADER=n" \
	-D"mcuboot_CONFIG_BOOT_SIGNATURE_KEY_FILE=\"$(ZIGBEE_SIGNING_KEY)\"" \
	-D"CONFIG_MCUBOOT_IMAGE_VERSION=\"$(IMAGE_VERSION)\"" \
	-D"CONFIG_DATE_CODE=\"$(IMAGE_DATE)\""

SHELL_ARGS = \
	-DCONFIG_ZIGBEE_SHELL=y \
	-DCONFIG_ZIGBEE_DEBUG_FUNCTIONS=y \
	-DCONFIG_ZIGBEE_SHELL_ENDPOINT=10

.DEFAULT_GOAL := build
.PHONY: build
build:
	cd $(BUILD_DIR) && $(NINJA)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: dk
dk:
	# FIXME: zigbee-avc crashes on startup when using TRACE_ARGS
	#$(WEST_BUILD) --board nrf52840dk_nrf52840 -- $(TRACE_ARGS)
	$(WEST_BUILD) --board nrf52840dk_nrf52840 -- $(SHELL_ARGS)

.PHONY: dongle
dongle:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840

.PHONY: raw
raw:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840 -- $(RAW_ARGS)

ifeq ($(MODULE_TYPE),zigbee)
.PHONY: ota
ota:
	@$(TOP_DIR)/scripts/check-signing-key.py $(ZIGBEE_SIGNING_KEY)
	$(WEST_BUILD) --board nrf52840dongle_nrf52840 -- $(OTA_ARGS)

.PHONY: raw-trace
raw-trace:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840 -- $(RAW_ARGS) $(TRACE_ARGS)

.PHONY: test
test:
	$(MAKE) clean
	$(MAKE) ota
	$(MAKE) dk
	$(MAKE) dongle
	$(MAKE) raw
	$(MAKE) raw-trace

.PHONY: release
release:
	$(MAKE) ota ZIGBEE_SIGNING_KEY=$(ZIGBEE_RELEASE_KEY)
	cp $(BUILD_DIR)/zephyr/*.zigbee $(BUILD_DIR)/nrfhome-$(MODULE_NAME).zigbee
	cp $(BUILD_DIR)/zephyr/merged.hex $(BUILD_DIR)/nrfhome-$(MODULE_NAME).hex
	scp $(BUILD_DIR)/nrfhome-$(MODULE_NAME).* $(ZIGBEE_RELEASE_PATH)

else

.PHONY: test
test:
	$(MAKE) clean
	$(MAKE) dk
	$(MAKE) dongle
	$(MAKE) raw

endif

.PHONY: flash
flash:
	$(NCS_ENV) $(TOP_DIR)/scripts/nrf-flash.sh
