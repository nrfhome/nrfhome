.PHONY: all
all:
	$(MAKE) -C avc dongle
	$(MAKE) -C switch dongle

.PHONY: clean
clean:
	$(MAKE) -C avc clean
	$(MAKE) -C switch clean

.PHONY: test
test:
	$(MAKE) -C avc test
	$(MAKE) -C switch test
