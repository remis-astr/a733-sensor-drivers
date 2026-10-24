#!/usr/bin/env python3
"""Record the board's debug UART to a file, read-only.

Never writes to the port (no echo loop that would stop U-Boot's autoboot)
and survives the adapter disappearing/reappearing.

usage: serlog.py [-d /dev/ttyUSB0] [-b 115200] logfile
"""
import argparse, sys, time

import serial

p = argparse.ArgumentParser()
p.add_argument("-d", "--device", default="/dev/ttyUSB0")
p.add_argument("-b", "--baud", type=int, default=115200)
p.add_argument("logfile")
a = p.parse_args()

with open(a.logfile, "ab", buffering=0) as log:
    while True:
        try:
            with serial.Serial(a.device, a.baud, timeout=0.2) as port:
                print(f"serlog: {a.device} open", file=sys.stderr)
                while True:
                    data = port.read(4096)
                    if data:
                        log.write(data)
        except (serial.SerialException, OSError):
            time.sleep(0.5)
