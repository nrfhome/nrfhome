TOP_DIR := $(shell pwd)
include $(TOP_DIR)/common/base.mk

.PHONY: all
all:
	$(MAKE) -C zigbee-avc dongle
	$(MAKE) -C zigbee-switch dongle

.PHONY: clean
clean:
	$(MAKE) -C zigbee-avc clean
	$(MAKE) -C zigbee-switch clean

.PHONY: test
test:
	$(MAKE) -C zigbee-avc test
	$(MAKE) -C zigbee-switch test

.PHONY: setup
setup:
	mkdir -p $(ZIGBEE_RELEASE_PATH)
	mkdir -p $(dir $(ZIGBEE_SIGNING_KEY))
	$(IMGTOOL) keygen -k $(ZIGBEE_SIGNING_KEY) -t ecdsa-p256
