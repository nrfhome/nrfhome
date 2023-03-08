#!/bin/zsh

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
		echo "Can't find zephyr/merged.hex"
		exit 1
	fi
fi

function flash_dongle() {
	nrfutil pkg generate --debug-mode --hw-version 52 --sd-req 0 --application ${BUILDDIR}/zephyr/merged.hex dfu.zip

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
	#exec /opt/homebrew/bin/python3 /opt/nordic/ncs/current/toolchain/bin/pyocd flash -e sector -t nrf52840 */merged.hex
	exec python3 -m pyocd flash -e sector -t nrf52840 "${HEXFILE}"
fi
