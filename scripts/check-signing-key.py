#!/usr/bin/env python3

import os, sys

def main():
    key = sys.argv[1]
    if key[0:7] == "pkcs11:":
        return 0
    try:
        os.stat(key)
    except:
        print("Can't open %s" % key)
        print("Please run 'make setup' from the top level, or generate it by hand")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
