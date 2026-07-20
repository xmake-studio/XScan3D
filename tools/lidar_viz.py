#!/usr/bin/env python3
"""Capture, analyse and plot the 28-byte lidar frames forwarded by the ESP32-S3.

The firmware emits validated frames verbatim, so this script owns all decoding.
Record once, then re-analyse offline as often as you like:

    python lidar_viz.py --port COM7 --record scan.bin --seconds 20
    python lidar_viz.py --replay scan.bin --diag       # solve the angle mapping
    python lidar_viz.py --replay scan.bin              # polar plot
    python lidar_viz.py --port COM7                    # live polar plot

Needs: pyserial, numpy, matplotlib.

NOTE: the firmware no longer emits bare 28-byte frames -- it wraps each one in a
PktSample together with the orientation (see src/protocol.h), so --port and
--record here decode nothing. Use tools/scan3d.py for live capture. This script
still works on --replay of older recordings, and the angle calibration constants
below remain the single source of truth for both tools.
"""

import argparse
import struct
import sys
import time

import numpy as np

FRAME_LEN = 28
HEADER = bytes((0x55, 0xAA, 0x02, 0x08))
POINTS = 8

DIST_INVALID = 0x8000
DIST_MASK = 0x7FFF

# Raw angle -> degrees, solved from a 7555-frame capture.
# The field is 1/64 degree with a 0xA000 offset: angle = (raw - 40960) / 64,
# giving a full 0..360 turn over 40960..64000 (= 0xA000 + 360*64).
# The same 1/64 scale applies to the speed field, which reads ~23053 = 360 RPM,
# matching the 6 Hz rotation implied by the observed frame rate.
RAW_ANGLE_MIN = 40960
RAW_ANGLE_MAX = 64000

# Speed field -> RPM.
SPEED_SCALE = 64.0


class Frame:
    __slots__ = ("speed", "raw_angle", "stamp", "dist", "valid")

    def __init__(self, buf):
        self.speed, self.raw_angle = struct.unpack_from("<HH", buf, 4)
        (self.stamp,) = struct.unpack_from("<I", buf, 24)
        words = np.frombuffer(buf, dtype="<u2", count=POINTS, offset=8)
        self.valid = (words & DIST_INVALID) == 0
        self.dist = (words & DIST_MASK).astype(np.float64)


def iter_frames(byte_source):
    """Re-sync on the header and yield Frames. Tolerates boot text and dropouts."""
    buf = bytearray()
    for chunk in byte_source:
        if not chunk:
            continue
        buf += chunk
        while True:
            i = buf.find(HEADER)
            if i < 0:
                # Keep a header-length tail in case it straddles chunks.
                del buf[: max(0, len(buf) - (len(HEADER) - 1))]
                break
            if len(buf) - i < FRAME_LEN:
                del buf[:i]
                break
            yield Frame(bytes(buf[i : i + FRAME_LEN]))
            del buf[: i + FRAME_LEN]


def serial_source(port, baud, seconds=None, record=None):
    import serial  # imported late so --replay works without pyserial

    fh = open(record, "wb") if record else None
    deadline = time.time() + seconds if seconds else None
    with serial.Serial(port, baud, timeout=0.1) as ser:
        try:
            while deadline is None or time.time() < deadline:
                chunk = ser.read(4096)
                if chunk and fh:
                    fh.write(chunk)
                yield chunk
        finally:
            if fh:
                fh.close()
                print(f"recorded -> {record}", file=sys.stderr)


def file_source(path):
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(4096)
            if not chunk:
                return
            yield chunk


def to_degrees(raw):
    span = RAW_ANGLE_MAX - RAW_ANGLE_MIN
    return np.mod((np.asarray(raw, dtype=np.float64) - RAW_ANGLE_MIN) * 360.0 / span,
                  360.0)


def spread_points(frames):
    """Assign an angle to every point.

    A frame carries only its start angle, so the angular width of frame N is
    measured as the gap to frame N+1 and the 8 points are spread across it.
    Returns (angle_deg, distance_mm, source_frame_index); distance is NaN where
    the lidar reported no return.
    """
    ang = to_degrees([f.raw_angle for f in frames])
    step = np.mod(np.diff(ang), 360.0)
    out_a, out_d, out_i = [], [], []
    for k in range(len(frames) - 1):
        s = step[k]
        if not (0.0 < s < 90.0):
            continue  # dropped frames: a bogus span would smear points
        f = frames[k]
        out_a.append(ang[k] + s * np.arange(POINTS) / POINTS)
        out_d.append(np.where(f.valid, f.dist, np.nan))
        out_i.append(np.full(POINTS, k))
    if not out_a:
        return np.array([]), np.array([]), np.array([], dtype=int)
    return (np.mod(np.concatenate(out_a), 360.0),
            np.concatenate(out_d),
            np.concatenate(out_i))


def diagnose(frames):
    if len(frames) < 50:
        sys.exit(f"only {len(frames)} frames; capture a few seconds first")

    raw = np.array([f.raw_angle for f in frames], dtype=np.float64)
    speed = np.array([f.speed for f in frames], dtype=np.float64)
    dist = np.concatenate([f.dist for f in frames])
    valid = np.concatenate([f.valid for f in frames])

    print(f"frames          : {len(frames)}")
    print(f"raw angle       : min={raw.min():.0f} max={raw.max():.0f} "
          f"unique={len(np.unique(raw))}")
    print(f"speed field     : min={speed.min():.0f} max={speed.max():.0f} "
          f"mean={speed.mean():.1f}")
    print(f"points          : {valid.sum()} valid / {valid.size} "
          f"({100.0 * valid.sum() / valid.size:.1f}%)")
    if valid.any():
        v = dist[valid]
        print(f"valid distance  : min={v.min():.0f} max={v.max():.0f}")

    d = np.diff(raw)
    print(f"consecutive delta: median={np.median(d):.1f} "
          f"p5={np.percentile(d, 5):.1f} p95={np.percentile(d, 95):.1f}")
    print()
    print("Read the top-left plot: a clean rising sawtooth means the field IS")
    print("the start angle, and the ramp's floor/ceiling are the RAW_ANGLE_MIN")
    print("and RAW_ANGLE_MAX to hard-code above. Flat or noise means it is not.")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    n = min(len(raw), 1500)
    ax[0][0].plot(raw[:n], ".-", lw=0.5, ms=2)
    ax[0][0].set_title("raw angle vs frame index (expect sawtooth)")
    ax[0][0].set_xlabel("frame")

    ax[0][1].hist(raw, bins=200)
    ax[0][1].set_title("raw angle histogram (expect flat over the sweep)")

    ax[1][0].plot(speed[:n], lw=0.8)
    ax[1][0].set_title("speed field vs frame index")

    ax[1][1].hist(d[(d > -2000) & (d < 2000)], bins=200)
    ax[1][1].set_title("delta between consecutive frames")

    fig.tight_layout()
    plt.show()


def polar_plot(frames, live_source=None):
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    scat = ax.scatter([], [], s=3)
    ax.set_title("lidar scan")

    def draw(fr):
        a, d, _ = spread_points(fr)
        good = ~np.isnan(d)
        if not good.any():
            return
        a, d = a[good], d[good]
        scat.set_offsets(np.column_stack([np.radians(a), d]))
        ax.set_rmax(np.percentile(d, 99) * 1.1)

    if live_source is None:
        draw(frames)
        plt.show()
        return

    plt.ion()
    plt.show()
    window = list(frames)
    for f in live_source:
        window.append(f)
        if len(window) > 900:  # roughly one revolution's worth
            del window[:-900]
        if len(window) % 60 == 0:
            draw(window)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial port, e.g. COM7 or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--replay", help="read frames from a recorded .bin instead")
    p.add_argument("--record", help="write the raw byte stream to this file")
    p.add_argument("--seconds", type=float, help="stop capturing after N seconds")
    p.add_argument("--diag", action="store_true",
                   help="field diagnostics instead of a polar plot")
    args = p.parse_args()

    if not args.port and not args.replay:
        p.error("need --port or --replay")

    if args.replay:
        frames = list(iter_frames(file_source(args.replay)))
        print(f"{len(frames)} frames from {args.replay}", file=sys.stderr)
        diagnose(frames) if args.diag else polar_plot(frames)
        return

    src = iter_frames(serial_source(args.port, args.baud, args.seconds, args.record))
    if args.diag or args.seconds:
        frames = list(src)
        print(f"{len(frames)} frames captured", file=sys.stderr)
        if args.diag:
            diagnose(frames)
        return
    polar_plot([], live_source=src)


if __name__ == "__main__":
    main()
