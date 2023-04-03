#!/usr/bin/env python3

DEFAULT_TIMEBEACON_DEV = "/dev/timebeacon"

import asyncio, calendar, logging, re, sys, time

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
            self.data[36] << 8 | self.data[37])
        self.temp_c = round(encoded_data / 10000, 1)
        self.temp_f = round(self.temp_c * 1.8 + 32, 1)
        self.humidity = encoded_data % 1000 / 10

    def __str__(self):
        return "[mac=%s temp_c=%.1f temp_f=%.1f hum=%.1f%% bat=%d%%]" % (
            self.mac, self.temp_c, self.temp_f, self.humidity, self.battery)

class Timebeacon:
    def __init__(self, devname):
        global serial
        import serial
        self.devname = devname
        self.parsers = list()
        self.adv_re = re.compile("M(([0-9a-f]{2})+)", re.IGNORECASE)

    def add_parser(self, filt, parser, cb):
        self.parsers.append((filt, parser, cb))

    async def start(self):
        self.restart()

    def restart(self):
        self.uart = serial.Serial(self.devname)
        self.line = b""
        self.loop = asyncio.get_event_loop()
        self.loop.add_reader(self.uart, self.on_uart_rx)

        self.uart.write(b"\x15")
        for (filt, parser, cb) in self.parsers:
            cmd = "F" + "".join(map(lambda b: "%02x" % b, filt)) + "\n"
            self.uart.write(bytes(cmd, "utf-8"))
        self.send_timestamp()

    def on_uart_ioerror(self):
        logging.warning("%s is disconnected" % self.devname)
        try:
            self.loop.remove_reader(self.uart)
        except:
            pass

        try:
            self.uart.close()
        except:
            pass

        if self.timestamp_task != None:
            self.timestamp_task.cancel()
            self.timestamp_task = None

        self.uart = None
        self.loop.call_later(5, self.reconnect)

    def reconnect(self):
        logging.debug("retrying timebeacon %s" % self.devname)

        try:
            self.restart()
            logging.info("%s is reconnected" % self.devname)
        except:
            self.on_uart_ioerror()

    def on_uart_rx(self):
        try:
            c = self.uart.read(1)
        except serial.serialutil.SerialException:
            self.on_uart_ioerror()
            return

        if c == b"\r" or c == b"\n":
            if len(self.line) > 0:
                self.handle_line(self.line)
            self.line = b""
        else:
            self.line += c

    def handle_line(self, line):
        try:
            # if the data is junk, this might throw an exception
            line = str(line, "utf-8")
            matches = self.adv_re.match(line)
        except:
            matches = None

        if not matches:
            return

        for (filt, parser, cb) in self.parsers:
            try:
                data = parser(matches.group(1))
                cb(data)
                break
            except ValueError:
                pass

    def send_timestamp(self):
        logging.debug("timebeacon: sending timestamp")
        now = time.time()
        ts = self.localtime_to_time_t(now)

        # send millisecond timestamp, hex, big endian
        self.uart.write(bytes("T%016x\n" % round(ts * 1000), "utf-8"))

        self.timestamp_task = self.loop.call_later(10, self.send_timestamp)

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

def demo_cb(x):
    logging.info(x)

def main():
    logging.basicConfig(format='%(asctime)s %(message)s', level=logging.DEBUG)

    if len(sys.argv) > 1:
        devname = sys.argv[1]
    else:
        devname = DEFAULT_TIMEBEACON_DEV

    tb = Timebeacon(devname)

    # match 16-bit UUID 0xec88 in Govee advertising packets
    tb.add_parser([0x03, 0x03, 0x88, 0xec], GoveeReport, demo_cb)
    loop = asyncio.new_event_loop()
    loop.create_task(tb.start())
    loop.run_forever()

if __name__ == "__main__":
    sys.exit(main())
