#!/bin/bash

set -euo pipefail
set -x

# can specify -R to reverse
OPTIONS="${1:-}"

patch ${OPTIONS} --directory=${ZEPHYR_BASE}/../nrfxlib -p1 < zigbee-diagnostics.patch
patch ${OPTIONS} --directory=${ZEPHYR_BASE}/../nrfxlib -p1 < zigbee-elec-measurement.patch

exit 0
