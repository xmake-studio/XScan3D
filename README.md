# XScan3D

A DIY 3D lidar scanner: a spinning 2D lidar on a stepper-driven platform,
controlled by an RP2040 (Raspberry Pi Pico), plus a Windows desktop app that
scans, self-calibrates the rig, merges scans and builds meshes.

```
src/            firmware (PlatformIO, Arduino core for RP2040)
app/            desktop app (Tauri: Rust core + Svelte UI), see app/README.md
tools/          headless Python reference scripts the app's core is checked against
tools/lidar_bridge/  debug firmware that passes raw lidar bytes through USB
```

## Download

Prebuilt binaries are on the [Releases](../../releases) page:

- `XScan3D-firmware.uf2` — hold BOOTSEL while plugging in the Pico, then copy
  the file onto the `RPI-RP2` drive.
- `XScan3D_<version>_x64-setup.exe` — the app installer, or
  `XScan3D-portable.exe` to run without installing.

The app finds the scanner by its USB product string, so no port needs to be
configured. The rig's calibration is stored in the scanner's flash.

## Building

Firmware:

```bash
pio run -e pico
pio run -e pico -t upload --upload-port <port>
```

App: see [app/README.md](app/README.md).

## Releasing

Push a version tag; GitHub Actions builds the firmware and the app and
publishes a release with both:

```bash
git tag v1.0.0
git push origin v1.0.0
```

A tag with a suffix (`v1.1.0-beta.1`) is published as a pre-release.

## License

MIT, see [LICENSE](LICENSE). `app/src-tauri/third_party/PoissonRecon` is
Michael Kazhdan's PoissonRecon under its own MIT license.
