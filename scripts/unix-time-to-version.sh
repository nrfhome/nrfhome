#!/bin/bash

t=$(date +%s)
printf "%d.%d.%d\n" $(((t >> 24) & 0xff)) $(((t >> 16) & 0xff)) $((t & 0xffff))
exit 0
