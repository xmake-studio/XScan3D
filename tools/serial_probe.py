#!/usr/bin/env python3
"""Is the device talking? Answers that without any of the UI in the way.

    python serial_probe.py                 # list ports
    python serial_probe.py --port COM7     # listen, then try a command
    python serial_probe.py --port COM7 --no-dtr

On the RP2040's native USB the CDC endpoint only produces output once the host
asserts DTR. pyserial's default DTR state is platform dependent, so a port can
open perfectly, report no error, and stay completely silent. That failure looks
exactly like a dead firmware from the application's side, which is why this
tool exists: it prints the raw byte counts and the DTR state that produced them.
"""

import argparse
import sys
import time

import numpy as np
import serial
from serial.tools import list_ports

import scan_proto as sp


def show_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports found")
        return
    print("available ports:")
    for p in ports:
        print(f"  {p.device:10s} {p.description}")
        if p.vid is not None:
            print(f"{'':12s} VID:PID {p.vid:04X}:{p.pid:04X}  hwid {p.hwid}")


def classify(buf):
    """Count record types without consuming, so we can report what arrived."""
    counts = {"sample": 0, "telem": 0, "config": 0, "event": 0, "other": 0}
    i = 0
    while True:
        j = buf.find(sp.MAGIC, i)
        if j < 0 or j + 4 > len(buf):
            break
        tag = buf[j + 3]
        if tag == sp.SAMPLE_TAG:
            counts["sample"] += 1; i = j + sp.SAMPLE_LEN
        elif tag == sp.TELEM_TAG:
            counts["telem"] += 1; i = j + sp.TELEM_LEN
        elif tag == sp.CONFIG_TAG:
            counts["config"] += 1; i = j + sp.CONFIG_LEN
        elif tag == sp.EVENT_TAG:
            counts["event"] += 1; i = j + 5 + (buf[j + 4] if j + 4 < len(buf) else 0)
        else:
            counts["other"] += 1; i = j + 1
    return counts


def probe(port, baud, dtr, seconds, send):
    print(f"opening {port} at {baud} with DTR={dtr}")
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.1
    # Set before open so the line is in the right state from the first moment.
    ser.dtr = dtr
    ser.rts = dtr
    try:
        ser.open()
    except Exception as exc:
        sys.exit(f"could not open {port}: {exc}")

    # And again after open: some drivers only apply it on a live handle.
    try:
        ser.dtr = dtr
        ser.rts = dtr
    except Exception as exc:
        print(f"  (could not set DTR after open: {exc})")

    with ser:
        raw = bytearray()
        t0 = time.time()
        print(f"listening {seconds:.0f} s ...")
        while time.time() - t0 < seconds:
            raw += ser.read(4096)

        print(f"\n  {len(raw)} bytes in {seconds:.0f} s "
              f"({len(raw)/seconds:.0f} B/s)")
        if not raw:
            print("\n  NOTHING RECEIVED.")
            print("  - if you ran with DTR on, try --no-dtr (and vice versa)")
            print("  - check the board is running this firmware, not the")
            print("    bootloader (a fresh UF2 flash reboots into the app)")
            print("  - make sure no serial monitor holds the port")
            return

        print(f"  records: {classify(raw)}")
        print(f"  first bytes: {raw[:32].hex(' ')}")

        cap = sp.Capture()
        parser = sp.StreamParser(cap)
        parser.feed(bytes(raw))
        print(f"  decoded: {len(cap)} samples, {len(cap.telem)} telem, "
              f"config={cap.config}")
        for e in cap.events[:10]:
            print(f"  [mcu] {e}")

        if cap.telem:
            t = cap.telem[-1]
            q = t[sp.TEL_QUAT]
            print(f"  latest: state={sp.STATE_NAMES.get(int(t[sp.TEL_STATE]),'?')} "
                  f"platform={t[sp.TEL_PLATFORM]:+.2f} deg "
                  f"dropped={int(t[sp.TEL_DROPPED])}")
            print(f"  accel: {np.array(t[sp.TEL_ACCEL]).round(3)} g   "
                  f"quat: {np.array(q).round(3)}")
            if abs(np.linalg.norm(q) - 1.0) > 0.1:
                print("  AHRS quaternion is not unit length - filter not running")
        if not cap.samples:
            print("\n  No lidar samples -- telemetry works but the lidar UART")
            print("  is quiet. Check the lidar's power and its TX -> GPIO13.")

        if send:
            print(f"\nsending '{send}' ...")
            ser.reset_input_buffer()
            ser.write((send + "\n").encode())
            time.sleep(1.5)
            reply = ser.read(65536)
            print(f"  {len(reply)} bytes back")
            cap2 = sp.Capture()
            sp.StreamParser(cap2).feed(bytes(reply))
            for e in cap2.events:
                print(f"  [mcu] {e}")
            if not cap2.events and not cap2.telem:
                print("  no response -- the device is not reading commands")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--send", default="?",
                   help="command to try after listening (default '?')")
    p.add_argument("--no-dtr", action="store_true",
                   help="open with DTR deasserted, to compare")
    args = p.parse_args()

    if not args.port:
        show_ports()
        print("\nrerun with --port COMx")
        return
    probe(args.port, args.baud, not args.no_dtr, args.seconds, args.send)


if __name__ == "__main__":
    main()
