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
