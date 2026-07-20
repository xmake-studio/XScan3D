#!/usr/bin/env python3
"""Host-side mirror of src/protocol.h: parse the scanner's USB stream.

Three record types share the 55 AA 03 xx magic, where xx is the record length
(0x09 marks the variable-length event record). Anything that is not a valid
record is skipped a byte at a time, so boot chatter and dropouts cost at most
one record.

This module only decodes and reconstructs; tools/scanner_ui.py owns the
rendering.
"""

import struct
import sys
import time

import numpy as np

MAGIC = bytes((0x55, 0xAA, 0x03))

SAMPLE_TAG, SAMPLE_LEN = 0x20, 32
TELEM_TAG, TELEM_LEN = 0x10, 16
CONFIG_TAG, CONFIG_LEN = 0x14, 20
EVENT_TAG = 0x09
# scanDegrees scanTime | mode reserved steps settleMs captureMs -- the
# stepped-mode fields are zero in a continuous scan.
CONFIG_FMT = struct.Struct("<4x2f2B3H")

# Config field positions, same reasoning as the telemetry constants below.
CFG_DEGREES, CFG_TIME = 0, 1
CFG_MODE, CFG_STEPS = 2, 4
CFG_SETTLE_MS, CFG_CAPTURE_MS = 5, 6

MODE_CONTINUOUS, MODE_STEPPED = 0, 1

# Telemetry field positions, so callers index by name instead of by a magic
# number that silently shifts whenever the record grows.
TEL_T_US = 0
TEL_PLATFORM = 1
TEL_STATE = 2
TEL_DROPPED = 4

# --- Mount geometry ---------------------------------------------------------
#
# The lidar is bolted straight to the stepper shaft, and the shaft turns about
# the world vertical. The lidar's own scan plane is mounted vertical -- it
# contains the rotation axis -- which is what makes the scan three-dimensional:
# each shaft angle contributes one vertical slice, and half a turn carries that
# slice through every azimuth, so 180 degrees of travel is a whole sphere.
#
# At shaft angle 0 that slice is taken to be the world XZ plane, with the
# lidar's spin axis along world Y. Everything below is stated in those terms.

# How far the lidar's beam origin sits off the rotation axis, along its own spin
# axis. Zero on this rig -- the sensor is centred on the shaft -- but kept as a
# knob because a few millimetres of it shows up as doubled or thickened walls,
# and it is the only mount error that is not a pure rotation.
BEAM_OFFSET_MM = 0.0

# --- The lateral standoff, and why it splits flat surfaces in two -----------
#
# The lidar body sits on the axis, but the rangefinder inside it does not: the
# laser diode and the receiver are separate optics roughly EMITTER_SPACING_MM
# apart, and the range is referenced to a point between them. That reference
# point is therefore offset from the spin axis *sideways* -- perpendicular to
# the beam, within the scan plane -- and that offset turns with the head.
#
# Write the in-plane beam direction as u = (sin th, 0, cos th) and the in-plane
# perpendicular as v = (cos th, 0, -sin th). A measured point lands at
#
#     P = d*u + b*v      so      P_z = d*cos(th) - b*sin(th)
#
# where b is the lateral standoff. The -b*sin(th) term is the whole problem: it
# flips sign between the two halves of the lidar's revolution. A table top hit
# on the way down one side (th ~ 150 deg) and again on the way down the other
# (th ~ 210 deg) has almost the same cos(th) but opposite sin(th), so the same
# physical surface reconstructs at two heights about b apart -- one side of the
# table higher than the other. It is not noise and it does not average out.
#
# Getting b wrong does two distinct things, and it is worth keeping them apart
# because the two cures below do not address them equally. Measured against a
# simulated room with a true 15 mm standoff:
#
#     spacing   A-vs-B step   within-half warp
#           0      12.87 mm           12.23 mm
#          15       6.42 mm            6.48 mm
#          30       0.04 mm            0.96 mm     <- true value
#          45       6.52 mm            6.31 mm
#
#   * the STEP is the visible artefact: the surface built from one half sits
#     bodily above the surface built from the other, so a table arrives as two
#     sheets and a mesher bridges them into a staircase.
#   * the WARP is the surface bending within a single half, because the error
#     term varies smoothly with th across that half. It is a bowl rather than a
#     step, so it survives meshing looking plausible while still being wrong.
#
# So the two cures are NOT equivalent:
#
#   * Setting EMITTER_SPACING_MM correctly is the real fix. It is the only one
#     that removes both, and the table above is also how to tune it: the step
#     is V-shaped in the error and bottoms out at the true value, so dial the
#     number until a flat surface stops stepping. Being 15 mm out either way
#     costs the same, which means the step tells you the magnitude but not the
#     sign -- try both directions.
#   * Keeping one half (SCAN_HALF_A / _B) removes the step *by construction*,
#     since the two halves are never mixed, but leaves the warp untouched. It
#     is insurance against the two optical paths not being symmetric in a way
#     this one-parameter model cannot express -- not a substitute for getting
#     the spacing right. Costs half the points and a 360 deg sweep.
#
# Best results use both: correct spacing, then one half.
#
# The reconstruction uses half of this value as the lateral standoff, which
# assumes the axis is centred between the two optics. If it is not, this stops
# being a spec and becomes purely a tuning knob.
EMITTER_SPACING_MM = 30.0

# Which half of each lidar revolution to keep. The split is at sin(th) == 0 --
# straight up and straight down -- because that is exactly where the lateral
# term above changes sign, so each half is internally consistent.
#
# One half-plane runs pole to pole, so yawing it a full 360 deg covers the whole
# sphere; that is why one-side scanning needs twice the shaft travel that a
# both-halves scan does, for half as many points.
SCAN_HALF_BOTH, SCAN_HALF_A, SCAN_HALF_B = 0, 1, 2
SCAN_HALF_NAMES = {
    SCAN_HALF_BOTH: "both halves",
    SCAN_HALF_A: "one side only (A)",
    SCAN_HALF_B: "one side only (B)",
}

# The shaft turns about world Z: this is a plain yaw, and nothing else.
SPIN_AXIS = (0.0, 0.0, 1.0)

# Rotation of the lidar about its own spin axis, in degrees clockwise (the same
# sense the azimuth below runs in).
#
# This is the one mount constant that genuinely has to be dialled in, because it
# decides which direction within the vertical scan plane is *up*. Get it wrong
# and the slice is tipped: the floor climbs into the walls and a room comes out
# as a cone. Because it is a rotation about the lidar's own spin axis it is
# exactly an offset on the azimuth, so it costs nothing to apply -- but it must
# be applied before the shaft rotation, since afterwards the slice has already
# been swung into the wrong place.
#
# It only ever needs to be a multiple of 90 unless the mount is genuinely
# skewed. Try 0 / 90 / 180 / -90 against a scene you know and keep the one where
# the floor is flat.
LIDAR_ROTATION_DEG = 0.0

# Direction the lidar's reported angle runs, as seen from its +Z. The point
# construction below places azimuth 0 at +Y and advances clockwise; set this
# when the hardware actually advances the other way.
#
# Unlike a rotation this is a reflection, so nothing else can undo it, and it is
# not measurable from a capture: a mirrored scan of a room is exactly as
# self-consistent as a correct one. Set it by eye against a scene whose
# handedness you know -- text on a wall reading backwards is the giveaway.
LIDAR_REVERSE = True

# 180 deg about world X, applied to the finished cloud: (x, y, z) ->
# (x, -y, -z). Purely cosmetic -- it is a rigid transform, so it changes no
# metric -- and only needed if the sensor is mounted inverted.
FLIP_UPRIGHT = False

# --- Lidar angle calibration ------------------------------------------------
# Raw angle -> degrees, solved from a 7555-frame capture. The field is 1/64
# degree with a 0xA000 offset: angle = (raw - 40960) / 64, giving a full 0..360
# turn over 40960..64000 (= 0xA000 + 360*64). The same 1/64 scale applies to the
# speed field, which reads ~23053 = 360 RPM.
RAW_ANGLE_MIN = 40960
RAW_ANGLE_MAX = 64000
SPEED_SCALE = 64.0

POINTS = 8
DIST_INVALID = 0x8000
DIST_MASK = 0x7FFF

# struct PktSample: magic[4] t_us platformDeg speed rawAngle dist[8]
SAMPLE_FMT = struct.Struct("<4xIf2H8H")
# struct PktTelem: magic[4] t_us platformDeg state reserved dropped
TELEM_FMT = struct.Struct("<4xIf2BH")

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
STATE_STEP_MOVE, STATE_STEP_SETTLE, STATE_STEP_CAPTURE, STATE_UNWRAP = \
    range(5, 9)
STATE_NAMES = {
    STATE_IDLE: "idle",
    STATE_PARKING: "parking",
    STATE_SETTLING: "settling",
    STATE_SWEEPING: "sweeping",
    STATE_DONE: "done",
    STATE_STEP_MOVE: "stepping",
    STATE_STEP_SETTLE: "settling (step)",
    STATE_STEP_CAPTURE: "capturing",
    STATE_UNWRAP: "unwrapping",
}

# States whose lidar frames carry an angle worth trusting. In stepped mode the
# frames arriving while the shaft is moving between stops are not cloud data.
CAPTURE_STATES = (STATE_SWEEPING, STATE_STEP_CAPTURE)

# States in which the platform is working its way across the sweep, for a
# progress readout. Not the same list: the transit states belong here and not
# above, because they say where the run has got to without contributing points.
SWEEP_STATES = CAPTURE_STATES + (STATE_STEP_MOVE, STATE_STEP_SETTLE)


def to_degrees(raw):
    span = RAW_ANGLE_MAX - RAW_ANGLE_MIN
    return np.mod((np.asarray(raw, dtype=np.float64) - RAW_ANGLE_MIN) * 360.0 / span,
                  360.0)


# Column layouts for _ColumnCache: name -> (slice or index, dtype or None).
# A plain index yields a 1-D column, a slice a 2-D one.
_SAMPLE_COLS = {
    "t_us": (0, None),
    "platform": (1, None),
    "speed": (2, None),
    "raw_angle": (3, None),
    "dist": (slice(4, 12), None),
}
_TELEM_COLS = {
    "t_us": (TEL_T_US, None),
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
        # Latest config tuple the device reported.
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
        # connected link appends samples forever, scan or no scan. Without this
        # the sample count never stops rising, which keeps the viewer
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


# --- geometry ---------------------------------------------------------------


def axis_angle_matrix(axis, deg):
    """(N,) angles about a fixed unit `axis` -> (N,3,3) rotation matrices."""
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    th = np.radians(np.asarray(deg, dtype=np.float64))
    c, s = np.cos(th), np.sin(th)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return (np.eye(3)[None] + s[:, None, None] * K[None]
            + (1 - c)[:, None, None] * (K @ K)[None])


def build_cloud(cap, sweep_only=True, max_range=None, min_range=60.0,
                beam_offset=BEAM_OFFSET_MM,
                lidar_rotation=LIDAR_ROTATION_DEG,
                lidar_reverse=LIDAR_REVERSE, flip_upright=FLIP_UPRIGHT,
                emitter_spacing=EMITTER_SPACING_MM, half=SCAN_HALF_BOTH):
    """Reconstruct the 3D point cloud.

    Returns (xyz, dist, platform_deg). Each sample carries the shaft angle it
    was taken at, so there is no interpolation and no pose estimation here: the
    8 points of a frame are spread across the azimuth gap to the next frame,
    lifted into the vertical scan plane, and yawed by that frame's shaft angle.

    `emitter_spacing` corrects the rangefinder's lateral standoff and `half`
    discards one side of each revolution; see the notes at the top of this
    module for what they are for and why they are two answers to one problem.
    """
    col = cap.arrays()
    n = len(cap)
    if n < 2:
        raise SystemExit(f"only {n} samples; capture a sweep first")

    # Azimuth: a frame reports only its start angle, so its angular width is
    # the gap to the next frame. Frames whose successor was dropped would smear
    # their points over a bogus span, so they are discarded.
    ang = to_degrees(col["raw_angle"])
    step = np.mod(np.diff(ang), 360.0)
    keep = (step > 0.0) & (step < 90.0)

    if sweep_only:
        keep &= _sweep_mask(cap, col)[:-1]
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

    # Roll the lidar about its own spin axis into the mount's frame. That is
    # exactly an offset on the azimuth, so it costs nothing -- and it has to
    # happen here, before the shaft rotation, or the slice has already been
    # swung into place tipped. See LIDAR_ROTATION_DEG.
    th = np.radians((-az if lidar_reverse else az) + lidar_rotation)
    sin_th, cos_th = np.sin(th), np.cos(th)

    # Keep one half of each revolution, split at the poles where the lateral
    # term below changes sign. Applied to `good` rather than to `k`, because the
    # 8 points of a frame can straddle the boundary and the frame as a whole is
    # on neither side.
    if half == SCAN_HALF_A:
        good &= sin_th >= 0.0
    elif half == SCAN_HALF_B:
        good &= sin_th < 0.0

    # The scan plane, at shaft angle 0. The lidar sweeps its own plane about its
    # spin axis with 0 deg at "+cos" turning clockwise; that plane is mounted
    # vertical, so the sin component runs out along world X and the cos
    # component runs up world Z. The spin axis is then world Y, and the mount's
    # own standoff from the rotation axis lies along it.
    #
    # The lateral term is the rangefinder's own offset inside the lidar: the
    # range is referenced to a point b to the side of the spin axis, along the
    # in-plane perpendicular v = (cos th, 0, -sin th). Half the emitter spacing,
    # on the assumption the axis is centred between the two optics. Getting this
    # wrong is what splits a flat surface into two heights -- see the module
    # header.
    b = 0.5 * emitter_spacing
    p = np.stack([d * sin_th + b * cos_th,
                  np.full_like(d, beam_offset),
                  d * cos_th - b * sin_th], axis=-1)

    # Yaw about world Z by the shaft angle. No transpose ambiguity, no drift,
    # no offset to solve for: this is the commanded angle of a sensor rigidly
    # bolted to the shaft that produced it.
    R = axis_angle_matrix(SPIN_AXIS, col["platform"][k])
    world = np.einsum("nij,nkj->nki", R, p)

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
