NCS_VERSION := 2.3.0
NCS_ENV := $(TOP_DIR)/scripts/ncs-env.py $(NCS_VERSION)

# ZIGBEE_SIGNING_KEY is used for `make ota`
# ZIGBEE_RELEASE_KEY is used for `make release`
# This is done to simplify development and testing. `make ota` sometimes
# breaks due to NCS updates, so it's useful to be able to test that build
# configuration without having access to the "official" release key
# used for actual OTA deployment to installed units.
ZIGBEE_SIGNING_KEY ?= $(HOME)/.nrfhome/zigbee-signing.pem
ZIGBEE_RELEASE_KEY ?= $(ZIGBEE_SIGNING_KEY)
ZIGBEE_RELEASE_PATH ?= $(HOME)/.nrfhome/zrel

BUILD_DIR := build

ifeq ($(V),1)
VERBOSE_OPTION := -v
endif

WEST := $(NCS_ENV) west
NINJA := $(NCS_ENV) ninja
IMGTOOL := $(NCS_ENV) imgtool

IMAGE_VERSION = $(shell $(TOP_DIR)/scripts/unix-time-to-version.sh)
IMAGE_DATE = $(shell date +%Y%m%d-%H%M)

# start an interactive shell with NCS/Zephyr variables set up
# and all of the SDK utilities in $PATH
# Note: make's $(SHELL) is "sh" rather than the user's actual $SHELL
# https://www.gnu.org/software/make/manual/make.html#Choosing-the-Shell
.PHONY: sh
sh:
	@echo ""
	@echo "=================="
	@echo "Entering NCS shell"
	@echo "=================="
	@echo ""
	$(NCS_ENV) $(shell echo "$$SHELL")
