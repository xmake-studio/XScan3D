# XScan3D

The desktop app for the XScan3D lidar scanner: find the device, sweep, correct
every distortion the rig has, merge scans, build a surface, export.

It replaces the PyQt prototype that used to live in `tools/` — same
mathematics, ported to Rust and verified against it point for point, wrapped
in an interface with one obvious button. The prototype's interface is gone;
what stayed in `tools/` are the headless Python scripts the ports are still
checked against (see the table below).

```
app/
  src/                 the interface: Svelte 5 + a WebGL2 viewer
    lib/render/        renderer, camera, controls, shaders
    lib/components/    title bar, library, scan dock, toolbars, sheets
    lib/api.ts         the backend behind one interface (Tauri IPC, or a mock)
  src-tauri/           the core: Rust
    src/protocol.rs    the USB wire format (mirrors ../src/protocol.h)
    src/geometry.rs    cloud reconstruction, mount geometry, microstep, range
    src/calibration.rs self-calibration from a room scan
    src/registration.rs point-to-plane ICP with a floor-plan search (merging)
    src/meshing.rs     normals, screened Poisson, trimming
    cpp/               C ABI over Kazhdan's PoissonRecon (vendored, MIT)
```

## Running and building

```bash
npm install
npm run tauri dev      # the app, with hot reload for the interface
npm run tauri build    # a portable exe plus an installer
```

The build leaves `src-tauri/target/release/xscan3d.exe` (portable; the C
runtime is linked statically, so no Visual C++ redistributable is needed) and
an NSIS installer under `src-tauri/target/release/bundle/nsis/`. Windows 10 and
11 already carry the WebView2 runtime the window is drawn with; the installer
fetches it on the rare machine that does not.

### Developing the interface without the device

`npm run dev` alone serves the interface in any browser against a mock backend
(`src/lib/mock.ts`) that replays real captures and simulates a sweep, a merge,
a mesh build and a calibration. Export the sample data first:

```bash
cd src-tauri
cargo run --release --example dev_assets -- ../dev-assets ../../scans/room.bin ../../scans/autosaves/<other>.bin
```

## How the core maps to the old tools

| Python (`tools/`)                    | Rust (`src-tauri/src/`)              |
| ------------------------------------ | ------------------------------------ |
| `scan_proto.py` stream parser        | `protocol.rs`, `capture.rs`          |
| `scan_proto.build_cloud`             | `geometry.rs`                        |
| `scan_proto.fit_microstep`           | `geometry.rs` + `patches.rs`         |
| `scan_proto.fit_range_error`         | `calibration.rs` + `patches.rs`      |
| `calibrate_mount.py`                 | `calibration.rs`                     |
| `registration.py`                    | `registration.rs`, `kdtree.rs`       |
| `meshing.py`                         | `meshing.rs`, `cpp/poisson_bridge.*` |
| `device_detect.py`, `serial_probe.py` | `device.rs`                         |
| `cloud_io.py`                        | `export.rs`, `library.rs`            |
| `scanner_ui.py` and its panels       | `src/` (removed; see git history)    |

The ports are numerically faithful: on `scans/room.bin` the reconstructed
cloud matches the Python one to 0.0001 mm (float32 rounding), the microstep
coefficients to 5e-15 degrees, and a merge of two real scans returns the same
transform to 0.1 mm — while being 16x (microstep), 30x (merge) and 30x
(calibration) faster.

## The scan library

Scans are kept automatically, the way a photo library keeps photos:

```
Documents/XScan3D/
  library.json        names, merge groups, poses, the microstep fit per scan
  scans/<id>.bin      the raw stream, exactly as the device sent it
  scans/<id>.bin.part a scan still recording; recovered on the next start
  thumbs/<id>.png     thumbnails the viewer renders
```

Because the raw stream is the stored form, every scan is rebuilt from it with
the current calibration — recalibrating fixes every scan you already took,
and nothing is ever baked in.

## Wire formats between the core and the viewer

Point clouds and meshes are handed over as raw bytes, not JSON:

- cloud: 64-byte header (count, flags, sweep angle, bbox, robust height and
  range spans, ceiling height) then positions (f32 x3), ranges (u16, mm) and
  sweep order (u16). Points are shuffled, so any prefix of them is a uniform
  sample: the viewer uploads progressively and thins while the camera moves.
- live points: the same arrays with a 20-byte header, polled during a sweep.
- mesh: 16-byte header, positions (f32 x3), normals (i8 x4) and u32 indices.

## A web version

The interface is already a web app: the renderer, the state and every screen
run in a plain browser, and everything the core does goes through the small
`Backend` interface in `src/lib/api.ts` (`src/lib/mock.ts` is a second
implementation of it). A hosted version needs a third implementation —
the Rust core compiled to WebAssembly for the geometry, merging and meshing,
with the scanner reached over WebSerial, or a small server doing both. No part
of the interface has to be rewritten for it.
