#!/usr/bin/env python3

import os, re, sys

def main():
    raw_re = re.compile('RAW \[(\S+)\]')

    if os.isatty(sys.stdout.fileno()):
        print("Refusing to write binary data to a tty")
        sys.exit(1)

    while True:
        line = sys.stdin.readline()
        if line == "":
            break

        m = raw_re.search(line)
        if m != None:
            hex_array = m.group(1).split(",")
            int_array = list(map(lambda x: int(x, 16), hex_array))
            sys.stdout.buffer.write(bytes(int_array))

if __name__ == "__main__":
    main()
