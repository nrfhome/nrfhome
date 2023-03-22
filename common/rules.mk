include $(TOP_DIR)/common/base.mk

WEST_BUILD := $(WEST) $(VERBOSE_OPTION) \
	build --pristine --build-dir=$(BUILD_DIR)

TRACE_ARGS := \
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

.DEFAULT_GOAL := build
.PHONY: build
build:
	cd $(BUILD_DIR) && $(NINJA)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: dongle
dongle:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840

.PHONY: ota
ota:
	$(if $(ZIGBEE_SIGNING_KEY),,$(error Please set $$ZIGBEE_SIGNING_KEY))
	@if [ ! -e "$(ZIGBEE_SIGNING_KEY)" ]; then \
		echo "Please run 'make setup' from the top level, or manually generate $$ZIGBEE_SIGNING_KEY"; \
		exit 1; \
	fi
	$(WEST_BUILD) --board nrf52840dongle_nrf52840 -- $(OTA_ARGS)

.PHONY: raw
raw:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840 -- $(RAW_ARGS)

.PHONY: raw-trace
raw-trace:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840 -- $(RAW_ARGS) $(TRACE_ARGS)

.PHONY: dk
dk:
	$(WEST_BUILD) --board nrf52840dk_nrf52840 -- $(TRACE_ARGS)

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
	$(MAKE) ota
	cp $(BUILD_DIR)/zephyr/*.zigbee $(BUILD_DIR)/nrfhome-$(MODULE_NAME).zigbee
	cp $(BUILD_DIR)/zephyr/merged.hex $(BUILD_DIR)/nrfhome-$(MODULE_NAME).hex
	scp $(BUILD_DIR)/nrfhome-$(MODULE_NAME).* $(ZIGBEE_RELEASE_PATH)
