#!/usr/bin/env python3

# arm-none-eabi-objdump -d build/zephyr/zephyr.elf | tracepoints.py > tracepoints.json
# arm-none-eabi-objdump -xd /opt/nordic/ncs/v1.9.1/nrfxlib/zboss/production/trace/lib/cortex-m4/hard-float/libzboss.a | tracepoints.py > tracepoints.json

import json, os, re, sys

def main():
    # lots of cases where r0/r1 can't be easily parsed, so make
    # file_id and line_number the only mandatory fields
    default_regs = [ 0, 0, None, None ]
    regs = list(default_regs)
    func = None
    errors = 0
    tracepoints = list()

    reg_regex = re.compile('((mov)|(movw)|(movs)|(mov\.w))\s+r([0-3]), #([0-9-]+)')
    bl_regex = re.compile('bl.*<zb_trace_msg_port>')
    reloc_regex = re.compile(': R_ARM_THM_CALL\s+zb_trace_msg_port$')
    func_regex = re.compile('^[0-9a-f]{8} <([^$]\S+)>:$')

    while True:
        line = sys.stdin.readline()
        if line == "":
            break
        #print(line)

        m = reg_regex.search(line)
        if m != None:
            reg = int(m.group(6))
            val = int(m.group(7))
            regs[reg] = val
            continue

        if bl_regex.search(line):
            if regs[2] == None or regs[3] == None or func == None:
                errors = errors + 1
                regs = list(default_regs)
                continue

            entry = dict()
            entry["mask"] = "0x%04x" % regs[0]
            entry["level"] = regs[1]
            entry["file_id"] = regs[2]
            entry["line_number"] = regs[3]
            entry["function"] = func
            tracepoints.append(entry)

            #print("%08x,%08x,%08x,%08x %s" % (regs[0], regs[1], regs[2], regs[3], func))
            regs = list(default_regs)
            continue

        m = func_regex.search(line)
        if m != None:
            func = m.group(1)

    print(json.dumps(tracepoints, sort_keys=True, indent=4))

    sys.stderr.write("Tracepoints: %d\n" % len(tracepoints))
    sys.stderr.write("Errors: %d\n" % errors)

if __name__ == "__main__":
    main()
