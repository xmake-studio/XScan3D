#!/usr/bin/env python3
import serial
import time
import sys
import os
from pathlib import Path

Import("env")

def reset_to_bootloader(target, source, env):
    """Reset RP2040 into bootloader mode via DTR/RTS."""

    upload_port = env.get("UPLOAD_PORT")
    if not upload_port:
        print("ERROR: UPLOAD_PORT not found. Check your board connection.")
        sys.exit(1)

    print(f"Attempting to reset board on port: {upload_port}")

    try:
        # Method 1: Try to open serial port and use DTR/RTS reset
        ser = serial.Serial()
        ser.port = upload_port
        ser.baudrate = 1200
        ser.timeout = 0.5

        # Configure for RTS/DTR reset (typical for Arduino-like reset)
        ser.dtr = True
        ser.rts = False

        try:
            ser.open()
            print("Serial port opened, triggering bootloader mode...")

            # Toggle DTR to reset
            time.sleep(0.1)
            ser.dtr = False
            time.sleep(0.1)
            ser.dtr = True

            ser.close()
        except:
            ser.close()

        # Wait for bootloader to be ready
        time.sleep(2)
        print("Bootloader mode activated, proceeding with upload...")

    except Exception as e:
        print(f"Error resetting board: {e}")
        print("\nTroubleshooting:")
        print("1. Verify the board is connected via USB")
        print("2. Check Device Manager for the COM port")
        print("3. Make sure no other serial monitor is open")
        print("4. Try manually pressing BOOTSEL button on the board while uploading")
        sys.exit(1)

# Register callback before upload
env.AddPreAction("upload", reset_to_bootloader)
