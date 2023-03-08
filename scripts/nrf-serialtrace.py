#!/usr/bin/env python3

import json, logging, os, sys, serial, time

SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 10

class UartWrapper:
    def __init__(self, dev_path, baud):
        self.dev = serial.Serial(dev_path, baud)
        self.record = []

    def close(self):
        self.dev.close()

    def fetch8(self):
        b = self.dev.read(1)
        b = int(b[0])
        self.record.append(b)
        return b

    def fetch16(self):
        lo = self.fetch8()
        hi = self.fetch8()
        return (hi << 8) | lo

    def clear_record(self):
        self.record = []

    def get_record(self):
        return self.record

def read_loop(uart, tpidx):
    last_counter = None

    while True:
        uart.clear_record()
        if uart.fetch8() != 0xde:
            continue
        if uart.fetch8() != 0xad:
            continue

        count = uart.fetch8()
        if count < 10:
            logging.error("# warning: malformed packet: byte 2 is 0x%02x" % count)
            continue

        b = uart.fetch8()
        if b != 0x02:
            logging.error("# warning: malformed packet: byte 3 is 0x%02x" % b)
            continue

        zb_timer = uart.fetch16()
        zb_counter = uart.fetch16()
        file_id = uart.fetch16()
        line_number = uart.fetch16()

        args = []
        remaining = count - 10
        while remaining > 0:
            args.append("%02x" % uart.fetch8())
            remaining = remaining - 1

        raw = ",".join(map(lambda x: "%02x" % x, uart.get_record()))
        logging.info("RAW [%s]" % raw)

        if last_counter != None:
            if last_counter > zb_counter:
                delta = 0x10000 + zb_counter - last_counter
            else:
                delta = zb_counter - last_counter
            if delta != 1:
                logging.info("# warning: lost %d events" % delta)
        last_counter = zb_counter

        idx = (file_id, line_number)
        if idx in tpidx:
            tp = tpidx[idx]
        else:
            tp = {"function": "file_id_%d" % file_id,
                  "mask": 0, "level": 0}

        msg = "ts=%04x m=%s lev=%d %s:%d" % (zb_timer, tp["mask"],
            tp["level"], tp["function"], line_number)

        if len(args) > 0:
            msg = msg + " data=[" + ",".join(args) + "]"

        logging.info(msg)

def reconnect_after_drop(uart_device, serial_baud):
    start_time = time.time()

    while True:
        try:
            uart = UartWrapper(uart_device, serial_baud)
            return uart
        except serial.serialutil.SerialException:
            if time.time() - start_time > SERIAL_TIMEOUT_S:
                logging.error("# can't reconnect, exiting")
                sys.exit(1)
        time.sleep(0.05)

def main():
    logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO, stream=sys.stdout)

    with open(sys.argv[2], "r") as f:
        tracepoints = json.load(f)

    tpidx = dict()
    for entry in tracepoints:
        idx = (entry["file_id"], entry["line_number"])
        tpidx[idx] = entry

    # first attempt:
    uart_device = sys.argv[1]
    uart = UartWrapper(uart_device, SERIAL_BAUD)

    while True:
        try:
            read_loop(uart, tpidx)
        except serial.serialutil.SerialException:
            uart.close()
            logging.error("# warning: serial connection dropped")
            uart = reconnect_after_drop(uart_device, SERIAL_BAUD)

if __name__ == "__main__":
    main()
