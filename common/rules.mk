BUILD_DIR := build

ifeq ($(V),1)
VERBOSE_OPTION := -v
endif
WEST_BUILD := west $(VERBOSE_OPTION) build --pristine --build-dir=$(BUILD_DIR)

IMAGE_VERSION = $(shell $(TOP_DIR)/scripts/unix-time-to-version.sh)
IMAGE_DATE = $(shell date +%Y%m%d-%H%M)

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

.PHONY: build
build:
	cd $(BUILD_DIR) && ninja

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: dongle
dongle:
	$(WEST_BUILD) --board nrf52840dongle_nrf52840

.PHONY: ota
ota:
	$(if $(ZIGBEE_SIGNING_KEY),,$(error Please set $$ZIGBEE_SIGNING_KEY))
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
	$(if $(ZIGBEE_RELEASE_PATH),,$(error Please set $$ZIGBEE_RELEASE_PATH))
	$(MAKE) ota
	scp build/zephyr/*.zigbee $(ZIGBEE_RELEASE_PATH)/nrfhome-$(MODULE_NAME).zigbee
	scp build/zephyr/merged.hex $(ZIGBEE_RELEASE_PATH)/nrfhome-$(MODULE_NAME).hex
