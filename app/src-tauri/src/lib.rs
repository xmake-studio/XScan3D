pub mod calibration;
pub mod capture;
pub mod clouds;
pub mod commands;
pub mod core;
pub mod devcalib;
pub mod device;
pub mod export;
pub mod geometry;
pub mod jobs;
pub mod kdtree;
pub mod library;
pub mod meshing;
pub mod patches;
pub mod poisson;
pub mod protocol;
pub mod registration;
pub mod session;
pub mod settings;
pub mod util;

use std::sync::atomic::Ordering;

use tauri::Manager;

use crate::core::{log, Core, CoreRef};

/// Stops the motor if a sweep is running and closes the link, so quitting
/// never leaves the rig turning. A half-written recording stays on disk and
/// is recovered as an interrupted scan on the next start.
fn shutdown(core: &CoreRef) {
    if core.shutting_down.swap(true, Ordering::SeqCst) {
        return;
    }
    if core.scan.lock().active.is_some() {
        device::send_cmd(core, protocol::CMD_ABORT, None);
        std::thread::sleep(std::time::Duration::from_millis(80));
    }
    device::disconnect(core, false, None);
}

pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_single_instance::init(|app, _args, _cwd| {
            if let Some(w) = app.get_webview_window("main") {
                let _ = w.unminimize();
                let _ = w.show();
                let _ = w.set_focus();
            }
        }))
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_opener::init())
        .setup(|app| {
            if let Ok(dir) = app.path().app_log_dir() {
                log::init(dir);
            }
            log::info(&format!("XScan3D {} starting", app.package_info().version));
            let core = Core::new(app.handle().clone());
            app.manage(core.clone());
            device::start(core.clone());
            let c = core.clone();
            std::thread::spawn(move || {
                c.recover_orphans();
                c.emit_library();
            });
            Ok(())
        })
        .on_window_event(|window, event| {
            if let tauri::WindowEvent::Destroyed = event {
                if let Some(core) = window.app_handle().try_state::<CoreRef>() {
                    shutdown(core.inner());
                }
            }
        })
        .invoke_handler(tauri::generate_handler![
            commands::bootstrap,
            commands::settings_update,
            commands::range_error_at,
            commands::device_connect,
            commands::device_disconnect,
            commands::device_command,
            commands::scan_start,
            commands::scan_stop,
            commands::live_points,
            commands::scan_cloud,
            commands::scan_rename,
            commands::scans_delete,
            commands::scans_ungroup,
            commands::thumbnail_save,
            commands::thumbnail,
            commands::import_files,
            commands::export_cloud,
            commands::export_raw,
            commands::export_mesh,
            commands::merge_start,
            commands::merge_decide,
            commands::merge_cancel,
            commands::mesh_start,
            commands::mesh_cancel,
            commands::mesh_state,
            commands::mesh_data,
            commands::calibrate_start,
            commands::calibrate_cancel,
            commands::calibrate_apply,
            commands::library_root,
            commands::new_scan_id,
        ])
        .build(tauri::generate_context!())
        .expect("error while building XScan3D")
        .run(|app, event| {
            if let tauri::RunEvent::Exit = event {
                if let Some(core) = app.try_state::<CoreRef>() {
                    shutdown(core.inner());
                }
            }
        });
}
