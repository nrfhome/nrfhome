TOP_DIR := $(shell pwd)
include $(TOP_DIR)/common/base.mk

foreach_project = \
	$(MAKE) -C zigbee-avc $(1) && \
	$(MAKE) -C zigbee-switch $(1) && \
	$(MAKE) -C timebeacon $(1)

.DEFAULT_GOAL := all
.PHONY: all
all:
	$(call foreach_project,dongle)

.PHONY: clean
clean:
	$(call foreach_project,clean)

.PHONY: test
test:
	$(call foreach_project,test)

.PHONY: setup
setup:
	mkdir -p $(ZIGBEE_RELEASE_PATH)
	mkdir -p $(dir $(ZIGBEE_SIGNING_KEY))
	$(IMGTOOL) keygen -k $(ZIGBEE_SIGNING_KEY) -t ecdsa-p256
