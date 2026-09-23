//! Finding the scanner and talking to it.
//!
//! Several RP2040 boards may be plugged in and they all enumerate as the same
//! 2E8A:000A "Pico". Opening a port to ask "are you the scanner?" is not an
//! option -- asserting DTR wakes another board's firmware, and a 1200 baud
//! open reboots it -- so the scanner is recognised purely from its USB
//! descriptors: the firmware's product string is "XScan3D", and as a fallback
//! the chip serial of a board that has already answered like the scanner.

use std::collections::HashSet;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde::Serialize;

use crate::core::{log, CoreRef};
use crate::devcalib::{self, CalibSync};
use crate::protocol::{self, Config, Record, StreamParser};
use crate::session;

pub const PRODUCT: &str = "XScan3D";
pub const RP2040_VID: u16 = 0x2E8A;
const BAUD: u32 = 115_200;

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PortInfo {
    pub device: String,
    pub description: String,
    pub vid: Option<u16>,
    pub pid: Option<u16>,
    pub serial: Option<String>,
    pub product: Option<String>,
    pub is_scanner: bool,
}

#[derive(Clone, Debug, Serialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct DeviceStatus {
    /// "searching" | "connecting" | "ready" | "silent" | "error"
    pub phase: String,
    pub port: Option<String>,
    pub state: u8,
    pub platform: f32,
    pub dropped: u16,
    pub config: Option<Config>,
    pub error: Option<String>,
    pub bytes_in: u64,
    pub ports: Vec<PortInfo>,
    /// Where the calibration in the scanner's flash stands, see
    /// devcalib::status.
    pub calib: String,
}

pub struct LinkHandle {
    pub port: String,
    pub serial: Option<String>,
    pub auto: bool,
    tx: Sender<Vec<u8>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    pub opened_at: Instant,
    pub valid: Arc<AtomicBool>,
    pub bytes_in: Arc<AtomicU64>,
    /// Set by the link thread when the port failed.
    failed: Arc<parking_lot::Mutex<Option<String>>>,
}

#[derive(Default)]
pub struct DeviceState {
    pub ports: Vec<PortInfo>,
    pub link: Option<LinkHandle>,
    /// Serials auto-connect leaves alone until they are unplugged.
    pub hold: HashSet<String>,
    pub status: DeviceStatus,
    pub calib: CalibSync,
}

// --- enumeration ----------------------------------------------------------------------

#[cfg(windows)]
fn bus_product(vid: u16, pid: u16, serial: &str) -> Option<String> {
    use windows_sys::Win32::Devices::DeviceAndDriverInstallation::{
        CM_Get_DevNode_PropertyW, CM_Locate_DevNodeW, CM_LOCATE_DEVNODE_NORMAL, CR_SUCCESS,
    };
    use windows_sys::Win32::Devices::Properties::{DEVPKEY_Device_BusReportedDeviceDesc, DEVPROP_TYPE_STRING};
    // A USB device with a serial number is instanced by it, so the parent
    // (composite) device's id can be built without walking the tree.
    let inst: Vec<u16> = format!("USB\\VID_{vid:04X}&PID_{pid:04X}\\{serial}").encode_utf16().chain([0]).collect();
    let mut node: u32 = 0;
    unsafe {
        if CM_Locate_DevNodeW(&mut node, inst.as_ptr(), CM_LOCATE_DEVNODE_NORMAL) != CR_SUCCESS {
            return None;
        }
        let mut ty: u32 = 0;
        let mut buf = [0u16; 256];
        let mut size: u32 = (buf.len() * 2) as u32;
        if CM_Get_DevNode_PropertyW(node, &DEVPKEY_Device_BusReportedDeviceDesc, &mut ty, buf.as_mut_ptr() as *mut u8, &mut size, 0) != CR_SUCCESS {
            return None;
        }
        if ty != DEVPROP_TYPE_STRING {
            return None;
        }
        let len = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
        let s = String::from_utf16_lossy(&buf[..len]);
        if s.is_empty() {
            None
        } else {
            Some(s)
        }
    }
}

#[cfg(not(windows))]
fn bus_product(_vid: u16, _pid: u16, _serial: &str) -> Option<String> {
    None
}

/// Every serial port, with the USB product string where it can be read.
pub fn scan_ports() -> Vec<PortInfo> {
    let Ok(list) = serialport::available_ports() else { return vec![] };
    list.into_iter()
        .map(|p| match p.port_type {
            serialport::SerialPortType::UsbPort(u) => {
                let product = if u.vid == RP2040_VID {
                    u.serial_number.as_deref().and_then(|s| bus_product(u.vid, u.pid, s)).or(u.product.clone())
                } else {
                    u.product.clone()
                };
                let is_scanner = u.vid == RP2040_VID && product.as_deref().map(|s| s.trim().eq_ignore_ascii_case(PRODUCT)).unwrap_or(false);
                PortInfo {
                    description: u.product.clone().unwrap_or_else(|| p.port_name.clone()),
                    device: p.port_name,
                    vid: Some(u.vid),
                    pid: Some(u.pid),
                    serial: u.serial_number,
                    product,
                    is_scanner,
                }
            }
            _ => PortInfo {
                description: p.port_name.clone(),
                device: p.port_name,
                vid: None,
                pid: None,
                serial: None,
                product: None,
                is_scanner: false,
            },
        })
        .collect()
}

/// The ports that are this scanner, best match first. A remembered serial
/// only counts when the product string says nothing either way.
pub fn find_scanner<'a>(ports: &'a [PortInfo], known: &[String]) -> Vec<&'a PortInfo> {
    let mut out: Vec<&PortInfo> = ports.iter().filter(|p| p.is_scanner).collect();
    out.extend(ports.iter().filter(|p| {
        !p.is_scanner && p.vid == Some(RP2040_VID) && p.serial.as_ref().map_or(false, |s| known.contains(s))
    }));
    out
}

// --- the link ----------------------------------------------------------------------------

/// Starts the watcher: polls the device list (cheap, opens nothing) and
/// connects to the scanner when it appears.
pub fn start(core: CoreRef) {
    std::thread::Builder::new()
        .name("device-watch".into())
        .spawn(move || {
            while !core.shutting_down.load(Ordering::Relaxed) {
                tick(&core);
                std::thread::sleep(Duration::from_millis(1000));
            }
        })
        .expect("spawn device watcher");
}

fn tick(core: &CoreRef) {
    let ports = scan_ports();
    let present: HashSet<String> = ports.iter().filter_map(|p| p.serial.clone()).collect();
    let (auto_connect, known) = {
        let s = core.settings.read();
        (s.device.auto_connect, s.device.known_serials.clone())
    };
    let mut to_connect: Option<String> = None;
    let mut lost_link = None;
    {
        let mut dev = core.device.lock();
        dev.hold.retain(|s| present.contains(s));
        dev.ports = ports.clone();
        dev.status.ports = ports.clone();
        if let Some(link) = &dev.link {
            let failed = link.failed.lock().clone();
            let unplugged = !ports.iter().any(|p| p.device == link.port);
            let age = link.opened_at.elapsed().as_secs_f64();
            let valid = link.valid.load(Ordering::Relaxed);
            if let Some(msg) = failed {
                lost_link = Some((Some(msg), true));
            } else if unplugged {
                lost_link = Some((None, false));
            } else if link.auto && !valid && age > 5.0 {
                // Auto-connected but never answered like the scanner.
                lost_link = Some((Some("not-scanner".into()), true));
            } else if link.bytes_in.load(Ordering::Relaxed) == 0 && age > 3.0 && dev.status.phase != "silent" {
                dev.status.phase = "silent".into();
            }
        } else if auto_connect {
            for p in find_scanner(&ports, &known) {
                if p.serial.as_ref().map_or(false, |s| dev.hold.contains(s)) {
                    continue;
                }
                to_connect = Some(p.device.clone());
                break;
            }
            if to_connect.is_none() && dev.status.phase != "searching" && dev.status.phase != "error" {
                dev.status.phase = "searching".into();
            }
        }
    }
    if let Some((err, hold)) = lost_link {
        disconnect(core, hold, err);
    } else if let Some(port) = to_connect {
        log::info(&format!("scanner found on {port}, connecting"));
        connect(core, &port, true);
    }
    devcalib::tick(core);
    emit_status(core);
}

pub fn emit_status(core: &CoreRef) {
    let calib = devcalib::status(core);
    let st = {
        let mut dev = core.device.lock();
        dev.status.calib = calib;
        if let Some(l) = &dev.link {
            dev.status.bytes_in = l.bytes_in.load(Ordering::Relaxed);
        }
        dev.status.clone()
    };
    core.emit("device", st);
}

pub fn connect(core: &CoreRef, port: &str, auto: bool) {
    disconnect(core, false, None);
    let dtr = core.settings.read().device.dtr;
    let serial = {
        let dev = core.device.lock();
        dev.ports.iter().find(|p| p.device == port).and_then(|p| p.serial.clone())
    };
    let (tx, rx) = channel::<Vec<u8>>();
    let stop = Arc::new(AtomicBool::new(false));
    let valid = Arc::new(AtomicBool::new(false));
    let bytes_in = Arc::new(AtomicU64::new(0));
    let failed = Arc::new(parking_lot::Mutex::new(None));
    let thread = {
        let core = core.clone();
        let port = port.to_string();
        let (stop, valid, bytes_in, failed) = (stop.clone(), valid.clone(), bytes_in.clone(), failed.clone());
        let serial = serial.clone();
        std::thread::Builder::new()
            .name("device-link".into())
            .spawn(move || run_link(core, port, serial, dtr, rx, stop, valid, bytes_in, failed))
            .expect("spawn link")
    };
    let mut dev = core.device.lock();
    if let Some(s) = &serial {
        dev.hold.remove(s);
    }
    dev.calib = CalibSync::default();
    dev.link = Some(LinkHandle {
        port: port.to_string(),
        serial,
        auto,
        tx,
        stop,
        thread: Some(thread),
        opened_at: Instant::now(),
        valid,
        bytes_in,
        failed,
    });
    dev.status.phase = "connecting".into();
    dev.status.port = Some(port.to_string());
    dev.status.error = None;
    dev.status.config = None;
}

/// Drops the link. `hold` keeps auto-connect off this board until replugged.
pub fn disconnect(core: &CoreRef, hold: bool, error: Option<String>) {
    let link = {
        let mut dev = core.device.lock();
        let link = dev.link.take();
        if let Some(l) = &link {
            if hold {
                if let Some(s) = &l.serial {
                    dev.hold.insert(s.clone());
                }
            }
        }
        if link.is_some() {
            dev.status.phase = if error.as_deref().map_or(false, |e| e != "not-scanner") { "error".into() } else { "searching".into() };
            dev.status.error = error.clone();
            dev.status.port = None;
            dev.status.config = None;
            dev.status.state = 0;
        }
        link
    };
    if let Some(mut l) = link {
        l.stop.store(true, Ordering::Relaxed);
        if let Some(t) = l.thread.take() {
            let _ = t.join();
        }
        log::info(&format!("link to {} closed{}", l.port, error.map(|e| format!(": {e}")).unwrap_or_default()));
        // Whatever the sweep in progress holds is now all it will get.
        session::on_link_lost(core);
    }
}

pub fn is_connected(core: &CoreRef) -> bool {
    let dev = core.device.lock();
    dev.link.as_ref().map_or(false, |l| l.valid.load(Ordering::Relaxed))
}

pub fn send(core: &CoreRef, bytes: Vec<u8>) -> bool {
    let dev = core.device.lock();
    match &dev.link {
        Some(l) => l.tx.send(bytes).is_ok(),
        None => false,
    }
}

pub fn send_cmd(core: &CoreRef, letter: char, arg: Option<String>) -> bool {
    send(core, protocol::command(letter, arg))
}

#[allow(clippy::too_many_arguments)]
fn run_link(
    core: CoreRef,
    port_name: String,
    serial: Option<String>,
    dtr: bool,
    rx: Receiver<Vec<u8>>,
    stop: Arc<AtomicBool>,
    valid: Arc<AtomicBool>,
    bytes_in: Arc<AtomicU64>,
    failed: Arc<parking_lot::Mutex<Option<String>>>,
) {
    let opened = serialport::new(&port_name, BAUD).timeout(Duration::from_millis(15)).open();
    let mut port = match opened {
        Ok(p) => p,
        Err(e) => {
            *failed.lock() = Some(format!("open: {e}"));
            return;
        }
    };
    // The RP2040's native USB CDC only produces output once DTR is up.
    let _ = port.write_data_terminal_ready(dtr);
    let _ = port.write_request_to_send(dtr);
    let _ = port.write_all(b"?\n");
    // What calibration the scanner holds; old firmware answers "unknown
    // command", which is how devcalib tells it apart.
    let _ = port.write_all(&protocol::command(protocol::CMD_CALIB_READ, None));
    log::info(&format!("link open on {port_name}"));
    let mut parser = StreamParser::new(true);
    let mut buf = vec![0u8; 16384];
    let mut records: Vec<Record> = Vec::with_capacity(512);
    let mut last_emit = Instant::now();
    let mut dirty = false;
    while !stop.load(Ordering::Relaxed) {
        while let Ok(cmd) = rx.try_recv() {
            if let Err(e) = port.write_all(&cmd) {
                *failed.lock() = Some(format!("write: {e}"));
                return;
            }
        }
        match port.read(&mut buf) {
            Ok(n) if n > 0 => {
                bytes_in.fetch_add(n as u64, Ordering::Relaxed);
                let chunk = &buf[..n];
                records.clear();
                parser.feed(chunk, |r| records.push(r));
                let mut first_config = false;
                {
                    let mut dev = core.device.lock();
                    for r in &records {
                        match r {
                            Record::Telem(t) => {
                                dev.status.state = t.state;
                                dev.status.platform = t.platform;
                                dev.status.dropped = t.dropped;
                                dirty = true;
                            }
                            Record::Config(c) => {
                                dev.status.config = Some(*c);
                                if !valid.swap(true, Ordering::Relaxed) {
                                    first_config = true;
                                }
                                dev.status.phase = "ready".into();
                                dev.status.error = None;
                                dirty = true;
                            }
                            Record::Event(e) => {
                                log::info(&format!("[mcu] {e}"));
                            }
                            Record::Sample(_) | Record::Calib(_) => {}
                        }
                    }
                    if dev.status.phase == "silent" {
                        dev.status.phase = if valid.load(Ordering::Relaxed) { "ready".into() } else { "connecting".into() };
                    }
                }
                if first_config {
                    remember_serial(&core, serial.as_deref());
                }
                session::on_link_data(&core, chunk, &records);
                for r in &records {
                    match r {
                        Record::Event(e) => {
                            devcalib::on_event(&core, e);
                            core.emit("device-log", e.clone());
                        }
                        Record::Calib(c) => devcalib::on_chunk(&core, c),
                        _ => {}
                    }
                }
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                *failed.lock() = Some(format!("read: {e}"));
                return;
            }
        }
        if dirty && last_emit.elapsed() >= Duration::from_millis(90) {
            dirty = false;
            last_emit = Instant::now();
            emit_status(&core);
            session::on_tick(&core);
        }
    }
}

/// Only the scanner firmware sends a config record: remember this board so
/// auto-connect finds it even without the product string.
fn remember_serial(core: &CoreRef, serial: Option<&str>) {
    let Some(s) = serial else { return };
    let known = core.settings.read().device.known_serials.clone();
    if !known.iter().any(|k| k == s) {
        let mut k = known;
        k.push(s.to_string());
        core.update_settings(&serde_json::json!({ "device": { "knownSerials": k } }));
    }
}
