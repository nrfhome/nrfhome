#!/usr/bin/env python3

DEFAULT_TIMEBEACON_DEV = "/dev/timebeacon"

import calendar, time, os, re, sys

class GoveeReport:
    def __init__(self, data):
        if isinstance(data, str):
            self.data = bytes.fromhex(data)
        elif isinstance(data, bytes):
            self.data = data
        else:
            raise ValueError

        # GVH5177
        if len(self.data) != 39 or self.data[29:33] != b"\x09\xff\x01\x00":
            raise ValueError

        self.mac = "%02x:%02x:%02x:%02x:%02x:%02x" % (
            self.data[7], self.data[6], self.data[5],
            self.data[4], self.data[3], self.data[2])
        self.battery = self.data[38]

        encoded_data = (self.data[35] << 16 |
            self.data[36] << 8 | self.data[35])
        self.temp_c = round(encoded_data / 10000)
        self.temp_f = round(self.temp_c * 1.8 + 32, 1)
        self.humidity = encoded_data % 1000 / 10

    def __str__(self):
        return "[mac=%s temp_c=%d temp_f=%.1f hum=%.1f%% bat=%d%%]" % (
            self.mac, self.temp_c, self.temp_f, self.humidity, self.battery)

class Timebeacon:
    def __init__(self, devname):
        global serial
        import serial

        self.uart = serial.Serial(devname, timeout=5)
        self.uart.write(b"\x15")
        self.parsers = list()

    def add_parser(self, filt, parser):
        cmd = "F" + "".join(map(lambda b: "%02x" % b, filt)) + "\n"
        self.uart.write(bytes(cmd, "utf-8"))
        self.parsers.append(parser)

    def run(self):
        next_timestamp = 0
        data_re = re.compile("M(([0-9a-f]{2})+)\r\n", re.IGNORECASE)

        while True:
            now = time.time()
            if now > next_timestamp:
                self.send_timestamp(now)
                next_timestamp = now + 10
                print("[TIMESTAMP]")

            # read a single '\n' terminated line
            # This may generate an exception if there is a serial port issue
            line = self.uart.read_until()

            try:
                # if the data is junk, this might throw an exception
                line = str(line, "utf-8")
                matches = data_re.match(line)
            except:
                matches = None

            if matches:
                for p in self.parsers:
                    try:
                        data = p(matches.group(1))
                        print(data)
                        break
                    except ValueError:
                        pass

    def send_timestamp(self, now):
        ts = self.localtime_to_time_t(now)

        # send millisecond timestamp, hex, big endian
        self.uart.write(bytes("T%016x\n" % round(ts * 1000), "utf-8"))

    def localtime_to_time_t(self, now):
        # Create a UNIX time_t value representing the current
        # time which, when decoded to UTC, renders the *local*
        # time instead of the UTC time
        #
        # This allows all timezone/DST calculations to be done
        # on the transmitter rather than the receiver
        t = int(now)
        fraction = t - now

        return calendar.timegm(time.localtime(t)) + fraction

def main():
    if len(sys.argv) > 1:
        devname = sys.argv[1]
    else:
        devname = DEFAULT_TIMEBEACON_DEV

    tb = Timebeacon(devname)

    # match 16-bit UUID 0xec88 in Govee advertising packets
    tb.add_parser([0x03, 0x03, 0x88, 0xec], GoveeReport)

    tb.run()

if __name__ == "__main__":
    sys.exit(main())
