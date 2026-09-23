//! Dev tool: run one sweep on the real scanner through the app's own pipeline
//! (port detection, the link, the live builder, the finished cloud) without
//! the window, and drop the recording into the scan library.
//!
//!   cargo run --release --example device_scan -- [seconds] [library dir]

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use xscan3d_lib::capture::Capture;
use xscan3d_lib::clouds;
use xscan3d_lib::device;
use xscan3d_lib::geometry::{self, LiveBuilder};
use xscan3d_lib::library::{new_id, now_iso, ScanKind, ScanMeta};
use xscan3d_lib::protocol::{self, Record, StreamParser};
use xscan3d_lib::util::identity4;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let secs: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let root = args
        .get(2)
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| dirs_documents().join("XScan3D"));

    let ports = device::scan_ports();
    for p in &ports {
        println!("{:8} vid={:?} serial={:?} product={:?} scanner={}", p.device, p.vid, p.serial, p.product, p.is_scanner);
    }
    let Some(port) = device::find_scanner(&ports, &[]).first().map(|p| p.device.clone()) else {
        eprintln!("no scanner found");
        return;
    };
    println!("--- connecting to {port}");
    let mut ser = serialport::new(&port, 115_200).timeout(Duration::from_millis(15)).open().expect("open port");
    ser.write_data_terminal_ready(true).ok();
    ser.write_request_to_send(true).ok();
    let mut send = |ser: &mut Box<dyn serialport::SerialPort>, c: char, arg: Option<String>| {
        ser.write_all(&protocol::command(c, arg)).expect("write");
    };
    send(&mut ser, protocol::CMD_STATUS, None);
    std::thread::sleep(Duration::from_millis(300));
    send(&mut ser, protocol::CMD_ANGLE, Some("90.00".into()));
    send(&mut ser, protocol::CMD_TIME, Some(format!("{secs:.2}")));
    send(&mut ser, protocol::CMD_MODE, Some("0".into()));
    send(&mut ser, protocol::CMD_START, None);
    println!("--- sweeping +-90 deg over {secs} s");

    let mount = geometry::Mount::default();
    let mut parser = StreamParser::new(true);
    let mut builder = LiveBuilder::new(mount.clone());
    let mut cap = Capture::new();
    let mut raw: Vec<u8> = Vec::new();
    let mut live: Vec<([f32; 3], f32, f32)> = Vec::new();
    let mut buf = vec![0u8; 16384];
    let started = Instant::now();
    let mut seen_capture = false;
    let mut last_print = Instant::now();
    let mut state = 0u8;
    let mut dropped = 0u16;
    loop {
        match ser.read(&mut buf) {
            Ok(n) if n > 0 => {
                raw.extend_from_slice(&buf[..n]);
                let mut recs = Vec::new();
                parser.feed(&buf[..n], |r| recs.push(r));
                for r in recs {
                    match r {
                        Record::Sample(s) => {
                            cap.samples.push(s);
                            builder.push(&s, &mut live);
                        }
                        Record::Telem(t) => {
                            let mut t = t;
                            t.sample_index = cap.samples.len();
                            state = t.state;
                            dropped = t.dropped;
                            cap.telem.push(t);
                            if protocol::is_sweep_state(t.state) {
                                seen_capture = true;
                            }
                        }
                        Record::Config(c) => {
                            println!("[cfg] +-{:.0} deg, {:.1} s, mode {}", c.degrees, c.time, c.mode);
                            cap.config = Some(c);
                        }
                        Record::Event(e) => println!("[mcu] {e}"),
                    }
                }
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                eprintln!("read error: {e}");
                break;
            }
        }
        if last_print.elapsed() > Duration::from_millis(1000) {
            last_print = Instant::now();
            println!("  state={state} points={} dropped={dropped}", live.len());
        }
        if seen_capture && (state == protocol::STATE_DONE || state == protocol::STATE_IDLE) {
            break;
        }
        if started.elapsed() > Duration::from_secs_f64(secs * 3.0 + 40.0) {
            eprintln!("timed out waiting for the sweep");
            break;
        }
    }
    println!("--- sweep over: {} live points, {} frames", live.len(), cap.samples.len());

    // Finish the way the app does: fit the microstep, build the final cloud.
    let id = new_id();
    let dir = root.join("scans");
    std::fs::create_dir_all(&dir).expect("library dir");
    let path = dir.join(format!("{id}.bin"));
    std::fs::write(&path, &raw).expect("write recording");
    let t = Instant::now();
    let meta = ScanMeta {
        id: id.clone(),
        name: None,
        created: now_iso(),
        kind: ScanKind::Device,
        file: format!("scans/{id}.bin"),
        points: 0,
        duration: cap.sweep_seconds(),
        sweep_deg: cap.config.map(|c| c.degrees as f64),
        complete: true,
        group: None,
        pose: identity4(),
        microstep: None,
        source_name: None,
        bytes: raw.len() as u64,
    };
    let built = clouds::build_capture(&meta, &cap, &mount).expect("build");
    println!(
        "--- final cloud: {} points in {:.2}s (microstep {}), sweep {:.1}s, ceiling {:?}",
        built.entry.len(),
        t.elapsed().as_secs_f64(),
        if built.new_microstep.is_some() { "fitted" } else { "none" },
        cap.sweep_seconds(),
        built.entry.ceiling,
    );
    println!("--- recording -> {}", path.display());
}

fn dirs_documents() -> std::path::PathBuf {
    std::env::var("USERPROFILE")
        .map(|p| std::path::PathBuf::from(p).join("Documents"))
        .unwrap_or_else(|_| std::path::PathBuf::from("."))
}
