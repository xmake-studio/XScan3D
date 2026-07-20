#!/usr/bin/env python3
"""Host-side mirror of src/protocol.h: parse the scanner's USB stream.

Three record types share the 55 AA 03 xx magic, where xx is the record length
(0x09 marks the variable-length event record). Anything that is not a valid
record is skipped a byte at a time, so boot chatter and dropouts cost at most
one record.

This module only decodes and reconstructs; tools/scan3d.py owns the rendering.
"""

import struct
import sys
import time

import numpy as np

import lidar_viz as lv  # RAW_ANGLE_MIN/MAX calibration lives there

MAGIC = bytes((0x55, 0xAA, 0x03))

SAMPLE_TAG, SAMPLE_LEN = 0x30, 48
TELEM_TAG, TELEM_LEN = 0x38, 56
CONFIG_TAG, CONFIG_LEN = 0x1A, 26
EVENT_TAG = 0x09
# scanDegrees scanTime gearRatio | mode reserved steps settleMs averageMs
# captureMs -- the stepped-mode fields are zero in a continuous scan.
CONFIG_FMT = struct.Struct("<4x3f2B4H")

# Config field positions, same reasoning as the telemetry constants below.
CFG_DEGREES, CFG_TIME, CFG_GEAR = 0, 1, 2
CFG_MODE, CFG_STEPS = 3, 5
CFG_SETTLE_MS, CFG_AVERAGE_MS, CFG_CAPTURE_MS = 6, 7, 8

MODE_CONTINUOUS, MODE_STEPPED = 0, 1

# Telemetry field positions, so callers index by name instead of by a magic
# number that silently shifts whenever the record grows.
TEL_T_US = 0
TEL_ACCEL = slice(1, 4)
TEL_GYRO = slice(4, 7)
TEL_QUAT = slice(7, 11)
TEL_PLATFORM = 11
TEL_STATE = 12
TEL_DROPPED = 14

# The scan plane does not pass through the rotation axis: the lidar's beam
# origin sits this far along the sensor's +Z (up) from it. That lever arm swings
# as the platform tilts, so ignoring it smears every surface by up to
# BEAM_OFFSET_MM * sin(scan angle) -- about 30 mm at +/-30 deg, which reads as
# doubled walls. It is corrected by offsetting each point in the sensor frame
# before the rotation is applied.
BEAM_OFFSET_MM = 59.0

# The stepper tilts the platform about the sensor's Y axis.
TILT_AXIS = (0.0, 1.0, 0.0)

# Rotation of the lidar about its own spin axis relative to the IMU's frame,
# in degrees clockwise (the same sense the azimuth below runs in).
#
# The lidar and the IMU are two separate parts bolted to the same platform, and
# nothing made their zero directions agree. The tilt axis above is the sensor's
# Y, so if the lidar is mounted a quarter turn out, the axis the platform
# actually tilts about lies along the lidar's X instead -- the cloud then hinges
# about the wrong direction and the sweep smears sideways rather than stacking
# into slices. The rig as built is 90 deg clockwise out, hence the default.
#
# It only ever needs to be a multiple of 90 unless the mount is genuinely
# skewed, and the symptom of getting it wrong is unmistakable: a flat wall
# comes out as a curved fan. Because it is a rotation about the lidar's own
# spin axis, it is exactly an offset on the azimuth, and the beam standoff
# (which lies along that axis) is unaffected.
LIDAR_ROTATION_DEG = 90.0

# Direction the lidar's reported angle runs, as seen from its +Z. The point
# construction below places azimuth 0 at +Y and advances clockwise; set this
# when the hardware actually advances the other way, which reflects the cloud.
LIDAR_REVERSE = True

# 180 deg about world X, applied to the finished cloud: (x, y, z) ->
# (x, -y, -z). Scans otherwise come out upside down.
FLIP_UPRIGHT = True

# --- On the four knobs above, and what they can and cannot be derived from ---
#
# BEAM_OFFSET_MM, LIDAR_ROTATION_DEG, LIDAR_REVERSE and FLIP_UPRIGHT together
# describe how the lidar is bolted on relative to the IMU. Three of them are
# entangled in ways that are worth stating, because a lot of time can be lost
# trying to solve for them from a capture:
#
#   * The azimuth zero and the beam offset SIGN are degenerate. Rotating the
#     azimuth 180 deg and negating the offset differ only by a global inversion
#     of the whole cloud, so they score identically on any self-consistency
#     measure. Measured on a real capture, (+59, 105 deg) and (-59, 285 deg)
#     came out bit-identical. This is why a lidar that physically sits +59 mm
#     ABOVE the axis can appear to need -59: the sign is not independently
#     meaningful until the azimuth zero is pinned down.
#
#   * LIDAR_REVERSE is unobservable in --from-stepper mode and only barely
#     observable without it. At the default 90 deg rotation, reversing the
#     azimuth is exactly a mirror about the XZ plane, and that mirror commutes
#     with rotation about Y -- so against the stepper's pure-Y tilt the two
#     reconstructions are identical point for point (verified: same cloud, same
#     6816 occupied voxels).
#
#     Against the AHRS quaternion they are not quite identical, but only
#     because the real rotation is not purely about Y: its axis carries about
#     0.07 of X and Z, which is genuine mechanical slop. That residue makes the
#     mirror very weakly measurable -- 6764 occupied voxels unreversed against
#     6927 reversed, a 2.4% edge resting entirely on how the rig wobbles. That
#     is not a basis for deciding a handedness convention. Set it by eye.
#
#   * FLIP_UPRIGHT is a rigid transform of the finished cloud, so it changes
#     no metric either. Purely cosmetic, and safe to change on its own.
#
# What IS verifiable, and was: the AHRS tracks the sweep correctly. On a
# +/-45 deg capture the quaternion's rotation axis came out [-0.02 +1.00 +0.05]
# -- the sensor's Y, as designed -- at 0.994 deg per commanded degree. So when
# a cloud looks wrong, these mount conventions are the place to look, not the
# filter.
#
# The practical consequence: dial these in once by eye against a scene you know
# (a room corner is ideal -- it fixes all three axes at once), and the UI will
# remember them. The symptom-to-knob map is:
#
#   upside down, otherwise correct ............ FLIP_UPRIGHT
#   handed wrong / text reads backwards ....... LIDAR_REVERSE
#   hinges about the wrong axis, walls fan .... LIDAR_ROTATION_DEG
#   walls doubled or thickened ................ BEAM_OFFSET_MM magnitude

POINTS = 8
DIST_INVALID = 0x8000
DIST_MASK = 0x7FFF

# struct PktSample: magic[4] t_us qw qx qy qz platformDeg speed rawAngle dist[8]
SAMPLE_FMT = struct.Struct("<4xI5f2H8H")
# struct PktTelem: magic[4] t_us accel[3] gyro[3] quat[4] platformDeg
#                  state reserved dropped
TELEM_FMT = struct.Struct("<4xI11f2BH")

# The wire format defines a record's tag to BE its length, and each struct must
# match that length. Growing PktTelem once already broke both halves of that
# silently -- the tag still said 40 while the struct read 56, so the parser
# walked off the end of its buffer. Assert it at import instead.
for _name, _tag, _len, _fmt in (
        ("sample", SAMPLE_TAG, SAMPLE_LEN, SAMPLE_FMT),
        ("telem", TELEM_TAG, TELEM_LEN, TELEM_FMT),
        ("config", CONFIG_TAG, CONFIG_LEN, CONFIG_FMT)):
    assert _tag == _len, f"{_name}: tag 0x{_tag:02X} != length {_len}"
    assert _fmt.size == _len, f"{_name}: struct is {_fmt.size} B, length says {_len}"
del _name, _tag, _len, _fmt

# Scanner state machine (scanner.h). Only the two capturing states contribute
# to the cloud; everything else is the platform in transit.
STATE_IDLE, STATE_PARKING, STATE_SETTLING, STATE_SWEEPING, STATE_DONE = range(5)
STATE_STEP_MOVE, STATE_STEP_SETTLE, STATE_STEP_AVERAGE, STATE_STEP_CAPTURE = \
    range(5, 9)
STATE_NAMES = {
    STATE_IDLE: "idle",
    STATE_PARKING: "parking",
    STATE_SETTLING: "settling",
    STATE_SWEEPING: "sweeping",
    STATE_DONE: "done",
    STATE_STEP_MOVE: "stepping",
    STATE_STEP_SETTLE: "settling (step)",
    STATE_STEP_AVERAGE: "averaging IMU",
    STATE_STEP_CAPTURE: "capturing",
}

# States whose lidar frames carry a pose worth trusting. In stepped mode the
# firmware only stamps the averaged pose during STEP_CAPTURE; frames arriving
# while it is moving or averaging carry a live quaternion for a platform that
# is either mid-move or not being captured against, so they are not cloud data.
CAPTURE_STATES = (STATE_SWEEPING, STATE_STEP_CAPTURE)


# Column layouts for _ColumnCache: name -> (slice or index, dtype or None).
# A plain index yields a 1-D column, a slice a 2-D one.
_SAMPLE_COLS = {
    "t_us": (0, None),
    "quat": (slice(1, 5), None),          # w x y z
    "platform": (5, None),
    "speed": (6, None),
    "raw_angle": (7, None),
    "dist": (slice(8, 16), None),
}
_TELEM_COLS = {
    "t_us": (TEL_T_US, None),
    "accel": (TEL_ACCEL, None),
    "gyro": (TEL_GYRO, None),
    "quat": (TEL_QUAT, None),
    "platform": (TEL_PLATFORM, None),
    "state": (TEL_STATE, int),
    "dropped": (TEL_DROPPED, None),
}


class _ColumnCache:
    """Incremental list-of-tuples -> columnar-numpy conversion.

    Backed by an over-allocated array that doubles on growth, so appending a
    live stream costs amortised O(1) per sample instead of O(n) per redraw.
    A shrinking list (the scene was cleared) just starts over.
    """

    def __init__(self, layout):
        self.layout = layout
        self.buf = None     # (capacity, width) float64
        self.n = 0          # rows of buf that are populated
        self.cols = None    # cached views, valid while n matches the source

    def columns(self, rows):
        n = len(rows)
        if n < self.n:                      # cleared or replaced wholesale
            self.buf, self.n, self.cols = None, 0, None
        if n > self.n:
            new = np.array(rows[self.n:], dtype=np.float64)
            if self.buf is None or self.buf.shape[0] < n:
                grown = np.empty((max(1024, n * 2), new.shape[1]), np.float64)
                if self.n:
                    grown[:self.n] = self.buf[:self.n]
                self.buf = grown
                self.cols = None            # old views point at the dead buffer
            self.buf[self.n:n] = new
            self.n = n
            self.cols = None
        if self.cols is None:
            a = self.buf[:n]
            self.cols = {
                name: (a[:, sel] if dt is None else a[:, sel].astype(dt))
                for name, (sel, dt) in self.layout.items()
            }
        return self.cols


class Capture:
    """Columnar store of everything decoded from a stream."""

    def __init__(self):
        self.samples = []  # tuples straight out of SAMPLE_FMT
        self.telem = []
        self.events = []
        # Latest (scanDegrees, scanTime, gearRatio) the device reported.
        self.config = None
        self._sample_cols = _ColumnCache(_SAMPLE_COLS)
        self._telem_cols = _ColumnCache(_TELEM_COLS)

    def __len__(self):
        return len(self.samples)

    # --- columns ------------------------------------------------------------
    # Built lazily because live capture appends constantly and only the
    # rendering path needs them, and cached incrementally because they are
    # rebuilt far more often than they change. Every redraw -- a spinbox
    # nudged, a 4 Hz live tick -- used to re-run np.array() over the entire
    # sample list, which is a Python-level walk of every tuple and by far the
    # most expensive thing in the render path on a large capture. Now only
    # samples that arrived since the last call are converted.
    #
    # The returned dicts are shared, not copies. Treat them as read-only:
    # index them (which copies) rather than writing through them.

    def arrays(self):
        if not self.samples:
            raise SystemExit("no samples decoded - is the firmware running?")
        return self._sample_cols.columns(self.samples)

    def telem_arrays(self):
        if not self.telem:
            return None
        return self._telem_cols.columns(self.telem)


class StreamParser:
    """Incremental decoder that keeps its buffer between feeds.

    This has to be long-lived. Reads off a serial port land on arbitrary
    boundaries, so a record is routinely split across two of them. Building a
    fresh parser per chunk discards the partial record at every seam and loses
    data in proportion to how often you read.
    """

    def __init__(self, capture=None, on_sample=None, echo_events=True,
                 sweep_only=False):
        self.cap = capture if capture is not None else Capture()
        self.on_sample = on_sample
        self.echo_events = echo_events
        # Drop frames that arrive outside a capture state instead of storing
        # them. The firmware streams lidar frames unconditionally -- see
        # Scanner::update, which calls serviceLidar() in every state -- so a
        # connected link appends samples forever, scan or no scan. Offline
        # tools want them all (scan3d --all reconstructs the parked frames);
        # a live viewer does not, because build_cloud's own _sweep_mask throws
        # exactly these away again at the far end of every redraw. Without
        # this the sample count never stops rising, which keeps the viewer
        # rebuilding a cloud that cannot change, over an input that only ever
        # grows.
        self.sweep_only = sweep_only
        self.state = None      # last state telemetry reported, for the above
        self._grace = False    # keep one frame past the end of a run
        self.buf = bytearray()
        self.bytes_in = 0
        self.junk = 0          # bytes skipped that matched no record
        self.skipped = 0       # samples dropped by sweep_only

    def feed(self, chunk):
        """Absorb bytes and decode whatever completed. Returns the Capture."""
        cap = self.cap
        if chunk:
            self.buf += chunk
            self.bytes_in += len(chunk)
        buf = self.buf
        i = 0
        n = len(buf)

        while True:
            j = buf.find(MAGIC, i)
            if j < 0:
                # Keep a short tail in case a magic straddles the next chunk;
                # anything before that is noise.
                keep = max(i, n - (len(MAGIC) - 1))
                self.junk += keep - i
                i = keep
                break
            if j + 4 > n:
                i = j
                break
            tag = buf[j + 3]

            if tag == SAMPLE_TAG:
                if j + SAMPLE_LEN > n:
                    i = j
                    break
                # Nothing is lost by dropping here rather than at render time:
                # _sweep_mask holds the 10 Hz state forward onto the samples,
                # so a frame is kept exactly when the most recent telemetry
                # said "capturing" -- which is the test applied here.
                #
                # ...with one frame of slack past the end of each run. A
                # frame's angular width is the gap to its *successor*
                # (build_cloud's np.diff), so dropping the first frame after a
                # sweep would leave the last frame of that sweep measuring its
                # span against the start of the next one instead. It gets
                # rejected as an implausible step rather than mis-drawn, but
                # keeping the successor is free and keeps the cloud exact.
                if self.sweep_only and self.state not in CAPTURE_STATES \
                        and not self._grace:
                    self.skipped += 1
                else:
                    self._grace = False
                    cap.samples.append(SAMPLE_FMT.unpack_from(buf, j))
                    if self.on_sample:
                        self.on_sample(cap.samples[-1])
                i = j + SAMPLE_LEN
            elif tag == TELEM_TAG:
                if j + TELEM_LEN > n:
                    i = j
                    break
                rec = TELEM_FMT.unpack_from(buf, j)
                cap.telem.append(rec)
                was, self.state = self.state, int(rec[TEL_STATE])
                if was in CAPTURE_STATES and self.state not in CAPTURE_STATES:
                    self._grace = True
                i = j + TELEM_LEN
            elif tag == CONFIG_TAG:
                if j + CONFIG_LEN > n:
                    i = j
                    break
                cap.config = CONFIG_FMT.unpack_from(buf, j)
                i = j + CONFIG_LEN
            elif tag == EVENT_TAG:
                if j + 5 > n:
                    i = j
                    break
                ln = buf[j + 4]
                if j + 5 + ln > n:
                    i = j
                    break
                text = buf[j + 5 : j + 5 + ln].decode("ascii", "replace")
                cap.events.append(text)
                if self.echo_events:
                    print(f"[mcu] {text}", file=sys.stderr)
                i = j + 5 + ln
            else:
                # Not one of ours; the magic was a coincidence in the payload.
                self.junk += 1
                i = j + 1

        del buf[:i]
        return cap


def parse(byte_source, capture=None, on_sample=None):
    """Generator wrapper over StreamParser, for the command-line tools.

    Yields the Capture after each chunk so callers can drive a live view.
    """
    p = StreamParser(capture, on_sample)
    for chunk in byte_source:
        yield p.feed(chunk)


# --- sources ----------------------------------------------------------------


def file_source(path, chunk=65536):
    with open(path, "rb") as fh:
        while True:
            b = fh.read(chunk)
            if not b:
                return
            yield b


def open_port(port, baud=115200, timeout=0.05, dtr=True):
    """Open a port with DTR asserted.

    The RP2040's native USB CDC only produces output once the host raises DTR,
    and pyserial's default for it is platform dependent. Without this a port
    opens cleanly, reports no error, and stays silent forever -- which is
    indistinguishable from dead firmware unless you know to look.
    """
    import serial

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = timeout
    ser.dtr = dtr          # before open: correct state from the first moment
    ser.rts = dtr
    ser.open()
    try:
        ser.dtr = dtr      # after open: some drivers only apply it live
        ser.rts = dtr
    except Exception:
        pass
    return ser


def serial_source(port, baud=115200, seconds=None, record=None, start=False):
    fh = open(record, "wb") if record else None
    deadline = time.time() + seconds if seconds else None
    with open_port(port, baud) as ser:
        if start:
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b"s")
        try:
            while deadline is None or time.time() < deadline:
                b = ser.read(8192)
                if b and fh:
                    fh.write(b)
                yield b
        finally:
            if fh:
                fh.close()
                print(f"recorded -> {record}", file=sys.stderr)


# --- geometry ---------------------------------------------------------------


def quat_to_matrix(q):
    """(N,4) quaternions as w,x,y,z -> (N,3,3) rotation matrices."""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q, axis=1, keepdims=True).clip(1e-12)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1),
    ], axis=1)


def resolve_frame(cap, col=None):
    """Decide whether the AHRS quaternion maps sensor->world or world->sensor.

    Madgwick implementations differ on this, and guessing wrong mirrors the
    whole cloud in a way that is easy to mistake for a wiring error. The rig is
    never in free fall, so the accelerometer always reads the up axis: rotating
    it into the world frame must give (0,0,+1) no matter how far the platform
    has tilted. Whichever of R and R^T holds that across the sweep is correct.
    """
    t = cap.telem_arrays()
    if t is None or len(t["accel"]) < 5:
        return False  # nothing to go on; assume the quaternion is sensor->world

    if col is None:
        col = cap.arrays()
    a = t["accel"]
    a = a / np.linalg.norm(a, axis=1, keepdims=True).clip(1e-9)
    idx = np.searchsorted(col["t_us"], t["t_us"]).clip(0, len(cap) - 1)
    R = quat_to_matrix(col["quat"][idx])

    fwd = np.einsum("nij,nj->ni", R, a)[:, 2].mean()
    inv = np.einsum("nji,nj->ni", R, a)[:, 2].mean()
    transpose = inv > fwd
    print(f"[frame] gravity along world +Z: R={fwd:+.3f} R^T={inv:+.3f} -> "
          f"using {'R^T (world->sensor)' if transpose else 'R (sensor->world)'}",
          file=sys.stderr)
    return transpose


def axis_angle_matrix(axis, deg):
    """(N,) angles about a fixed unit `axis` -> (N,3,3) rotation matrices."""
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    th = np.radians(np.asarray(deg, dtype=np.float64))
    c, s = np.cos(th), np.sin(th)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return (np.eye(3)[None] + s[:, None, None] * K[None]
            + (1 - c)[:, None, None] * (K @ K)[None])


def estimate_tilt_offset(cap, col=None):
    """Degrees the stepper's zero sits away from the IMU's, or None.

    The stepper only knows the angle it was commanded to, measured from
    wherever the platform happened to be when it was homed. The IMU measures
    the real tilt against gravity. The difference is the mechanical zero
    error, and it shows up in --from-stepper mode as the whole scene sitting
    at a slight angle.

    Only the tilt about Y is taken. For a rotation about Y, R[0,2] is sin of
    the angle and R[2,2] is cos, so the pitch falls straight out. The median
    rather than the mean, because the sweep ends carry the most mechanical
    slop and a couple of degrees there should not drag the estimate.

    Using the IMU here does not undo the point of --from-stepper. That mode
    exists because the filter's YAW drifts over a long sweep; pitch is
    accelerometer-corrected and does not drift, and this is a single constant
    taken across the whole sweep rather than a per-sample pose.
    """
    if col is None:
        try:
            col = cap.arrays()
        except SystemExit:
            return None
    m = _sweep_mask(cap, col)
    if m.sum() < 10:
        return None
    R = quat_to_matrix(col["quat"][m])
    imu_tilt = np.degrees(np.arctan2(R[:, 0, 2], R[:, 2, 2]))
    return float(np.median(imu_tilt - col["platform"][m]))


def build_cloud(cap, sweep_only=True, max_range=None, min_range=60.0,
                transpose=None, from_stepper=False, tilt_axis=TILT_AXIS,
                beam_offset=BEAM_OFFSET_MM, lidar_rotation=LIDAR_ROTATION_DEG,
                lidar_reverse=LIDAR_REVERSE, flip_upright=FLIP_UPRIGHT,
                tilt_offset=0.0):
    """Reconstruct the 3D point cloud.

    Returns (xyz, dist, platform_deg). Each sample carries its own pose, so
    there is no interpolation here: the 8 points of a frame are spread across
    the azimuth gap to the next frame and rotated by that frame's quaternion.
    """
    col = cap.arrays()
    n = len(cap)
    if n < 2:
        raise SystemExit(f"only {n} samples; capture a sweep first")

    # The AHRS is the better source in principle -- it sees real mechanical
    # slop and any flex in the 2:1 link. But Madgwick 6-DOF cannot observe yaw,
    # so it drifts, and over a 30 s sweep that slowly rotates the whole cloud
    # about Z and smears the room azimuthally. The stepper's own angle has no
    # drift at all, only whatever error the gearing contributes, so it is worth
    # reconstructing both ways and keeping the one that looks sharper.
    if not from_stepper and transpose is None:
        transpose = resolve_frame(cap, col)

    # Azimuth: a frame reports only its start angle, so its angular width is
    # the gap to the next frame. Frames whose successor was dropped would smear
    # their points over a bogus span, so they are discarded.
    ang = lv.to_degrees(col["raw_angle"])
    step = np.mod(np.diff(ang), 360.0)
    keep = (step > 0.0) & (step < 90.0)

    state_ok = np.ones(n - 1, dtype=bool)
    if sweep_only:
        state_ok = _sweep_mask(cap, col)[:-1]
    keep &= state_ok
    if not keep.any():
        raise SystemExit("no usable frames in the sweep - was 's' sent?")

    k = np.flatnonzero(keep)
    # (M, 8) azimuth for every point, then flattened.
    az = ang[k, None] + step[k, None] * (np.arange(POINTS) / POINTS)
    d = col["dist"][k]
    valid = (d.astype(np.uint16) & DIST_INVALID) == 0
    d = (d.astype(np.uint16) & DIST_MASK).astype(np.float64)

    good = valid & (d > min_range)
    if max_range is not None:
        good &= d <= max_range

    # Bring the lidar's azimuth into the IMU's frame. A rotation about the spin
    # axis is just an offset here, so it costs nothing and has to happen before
    # the pose is applied -- afterwards the tilt has already been taken about
    # the wrong direction and no amount of spinning fixes it. See
    # LIDAR_ROTATION_DEG.
    th = np.radians((-az if lidar_reverse else az) + lidar_rotation)
    # Sensor frame: the lidar sweeps its own XY plane about +Z, and 0 deg is
    # +Y turning clockwise -- the convention the 2D polar view already uses.
    # Z is the beam's standoff from the rotation axis, not zero: the whole scan
    # plane rides that far above the pivot and swings with it.
    p = np.stack([d * np.sin(th), d * np.cos(th),
                  np.full_like(d, beam_offset)], axis=-1)

    if from_stepper:
        # "auto" resolves against the IMU; a number is taken as given, and 0
        # keeps the raw commanded angle. Only meaningful here -- the AHRS path
        # measures the true tilt directly and has nothing to offset.
        off = tilt_offset
        if isinstance(off, str):
            off = estimate_tilt_offset(cap, col) or 0.0
        R = axis_angle_matrix(tilt_axis, col["platform"][k] + off)
        sub = "nij,nkj->nki"
    else:
        R = quat_to_matrix(col["quat"][k])
        sub = "nji,nkj->nki" if transpose else "nij,nkj->nki"
    world = np.einsum(sub, R, p)

    # 180 deg about world X, after the pose. See FLIP_UPRIGHT.
    if flip_upright:
        world[..., 1] *= -1.0
        world[..., 2] *= -1.0

    g = good.ravel()
    return (world.reshape(-1, 3)[g],
            d.ravel()[g],
            np.repeat(col["platform"][k], POINTS)[g])


def _sweep_mask(cap, col):
    """Per-sample boolean: was the scanner mid-sweep when this frame arrived?

    State is only reported in the 10 Hz telemetry, so it is held forward onto
    the much faster sample stream.
    """
    t = cap.telem_arrays()
    if t is None:
        return np.ones(len(cap), dtype=bool)
    order = np.searchsorted(t["t_us"], col["t_us"], side="right") - 1
    order = order.clip(0, len(t["state"]) - 1)
    return np.isin(t["state"][order], CAPTURE_STATES)


def voxel_downsample(xyz, extra, size):
    """Keep one point per `size`-mm cube. Cheap way to drop the redundancy that
    piles up close to the sensor, where the angular sampling is densest."""
    if size <= 0:
        return xyz, extra
    keys = np.floor(xyz / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    idx.sort()
    return xyz[idx], [e[idx] for e in extra]
