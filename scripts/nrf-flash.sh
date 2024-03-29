#!/bin/bash

set -ex

if [ -d build ]; then
	BUILDDIR="$(pwd)/build"
elif [ "$(basename `pwd`)" = "build" ]; then
	BUILDDIR="$(pwd)"
fi

if [ -n "${1:-}" ]; then
	HEXFILE="${1}"
else
	HEXFILE="${BUILDDIR}/zephyr/merged.hex"
	if [ ! -e "${HEXFILE}" ]; then
		HEXFILE="${BUILDDIR}/zephyr/zephyr.hex"
		if [ ! -e "${HEXFILE}" ]; then
			echo "Can't find zephyr/merged.hex or zephyr.hex"
			exit 1
		fi
	fi
fi

function flash_dongle() {
	nrfutil pkg generate \
		--debug-mode --hw-version 52 --sd-req 0 \
		--application "${HEXFILE}" dfu.zip

	DEV=$(ls /dev/cu.* | awk '/usbmodem[0-9A-F]{13}/ { print }')
	if [ -z "${DEV}" ]; then
		echo "can't find device in DFU mode"
		exit 1
	fi

	exec nrfutil dfu usb-serial -pkg dfu.zip -p "${DEV}" -b 115200
}

if grep -q "^CONFIG_BOARD_HAS_NRF5_BOOTLOADER=y" ${BUILDDIR}/zephyr/.config; then
	flash_dongle
else
	exec pyocd flash -e sector -t nrf52840 "${HEXFILE}"
fi
