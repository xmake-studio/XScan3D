//! The UI's entry points. Anything heavier than a few milliseconds runs on
//! the blocking pool or its own thread, so the webview never waits on it.

use serde::{Deserialize, Serialize};
use tauri::ipc::{Request, Response};
use tauri::State;

use crate::clouds;
use crate::core::CoreRef;
use crate::device::{self, DeviceStatus};
use crate::geometry;
use crate::jobs;
use crate::library;
use crate::meshing::MeshInfo;
use crate::protocol;
use crate::session::{self, ScanProgress};
use crate::settings::Settings;

type Core<'a> = State<'a, CoreRef>;

fn blocking<T: Send + 'static>(f: impl FnOnce() -> T + Send + 'static) -> impl std::future::Future<Output = Result<T, String>> {
    let h = tauri::async_runtime::spawn_blocking(f);
    async move { h.await.map_err(|e| e.to_string()) }
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Bootstrap {
    settings: Settings,
    library: crate::core::LibraryDto,
    device: DeviceStatus,
    version: String,
    log_dir: String,
    range_at_3m: f64,
    range_default: Vec<f64>,
}

#[tauri::command]
pub fn bootstrap(core: Core<'_>) -> Bootstrap {
    let settings = core.settings.read().clone();
    let range_at_3m = geometry::range_error_at(settings.mount.active_range_error(), 3000.0);
    let log_dir = tauri::Manager::path(&core.app).app_log_dir().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
    Bootstrap {
        settings,
        library: core.library_dto(),
        device: core.device.lock().status.clone(),
        version: core.app.package_info().version.to_string(),
        log_dir,
        range_at_3m,
        range_default: geometry::default_range_error(),
    }
}

#[tauri::command]
pub fn settings_update(core: Core<'_>, patch: serde_json::Value) -> Settings {
    core.update_settings(&patch)
}

#[tauri::command]
pub fn range_error_at(model: Option<Vec<f64>>) -> f64 {
    geometry::range_error_at(model.as_deref(), 3000.0)
}

// --- device ------------------------------------------------------------------------------

#[tauri::command]
pub fn device_connect(core: Core<'_>, port: String) {
    device::connect(&core, &port, false);
    device::emit_status(&core);
}

#[tauri::command]
pub fn device_disconnect(core: Core<'_>) {
    device::disconnect(&core, true, None);
    device::emit_status(&core);
}

/// Maintenance actions: "home", "unwrap", "beep", "status".
#[tauri::command]
pub fn device_command(core: Core<'_>, cmd: String) -> bool {
    match cmd.as_str() {
        "home" => device::send_cmd(&core, protocol::CMD_HOME, None),
        "unwrap" => device::send_cmd(&core, protocol::CMD_UNWRAP, Some("90".into())),
        "beep" => device::send_cmd(&core, protocol::CMD_BEEP, None),
        "status" => device::send_cmd(&core, protocol::CMD_STATUS, None),
        _ => false,
    }
}

// --- scanning ------------------------------------------------------------------------------

#[tauri::command]
pub fn scan_start(core: Core<'_>) -> Result<ScanProgress, String> {
    session::start(&core)
}

#[tauri::command]
pub fn scan_stop(core: Core<'_>) {
    session::stop(&core);
}

#[tauri::command]
pub fn live_points(core: Core<'_>, seq: u32, since: u32) -> Response {
    Response::new(session::live_points(&core, seq, since))
}

// --- library ---------------------------------------------------------------------------------

#[tauri::command]
pub async fn scan_cloud(core: Core<'_>, id: String) -> Result<Response, String> {
    let core = core.inner().clone();
    blocking(move || core.cloud(&id).map(|e| Response::new(clouds::serialize(&e)))).await?
}

#[tauri::command]
pub fn scan_rename(core: Core<'_>, id: String, name: Option<String>) {
    let name = name.map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
    {
        let mut lib = core.library.lock();
        if let Some(m) = lib.get_mut(&id) {
            m.name = name;
        } else if let Some(g) = lib.index.groups.iter_mut().find(|g| g.id == id) {
            g.name = name;
        }
    }
    core.save_library();
    core.emit_library();
}

#[tauri::command]
pub fn scans_delete(core: Core<'_>, ids: Vec<String>) {
    let scans = jobs::resolve(&core, &ids);
    {
        let mut cache = core.clouds.lock();
        for id in &scans {
            cache.remove(id);
        }
    }
    core.library.lock().delete(&scans);
    core.save_library();
    core.emit_library();
}

#[tauri::command]
pub fn scans_ungroup(core: Core<'_>, ids: Vec<String>) {
    let scans = jobs::resolve(&core, &ids);
    core.library.lock().ungroup(&scans, false);
    core.save_library();
    core.emit_library();
}

#[tauri::command]
pub fn thumbnail_save(core: Core<'_>, request: Request<'_>) -> Result<(), String> {
    let id = request.headers().get("x-scan-id").and_then(|v| v.to_str().ok()).ok_or("missing id")?.to_string();
    let tauri::ipc::InvokeBody::Raw(bytes) = request.body() else { return Err("expected raw body".into()) };
    if id.contains(['/', '\\', '.']) {
        return Err("bad id".into());
    }
    let path = core.library.lock().thumb_path(&id);
    std::fs::write(path, bytes).map_err(|e| e.to_string())
}

#[tauri::command]
pub fn thumbnail(core: Core<'_>, id: String) -> Response {
    let path = core.library.lock().thumb_path(&id);
    Response::new(std::fs::read(path).unwrap_or_default())
}

#[derive(Serialize)]
pub struct ImportResult {
    added: Vec<String>,
    failed: Vec<String>,
}

#[tauri::command]
pub async fn import_files(core: Core<'_>, paths: Vec<String>) -> Result<ImportResult, String> {
    let core = core.inner().clone();
    blocking(move || {
        let (added, failed) = jobs::import_files(&core, &paths);
        ImportResult { added, failed }
    })
    .await
}

#[tauri::command]
pub async fn export_cloud(core: Core<'_>, ids: Vec<String>, path: String, color_mode: String) -> Result<usize, String> {
    let core = core.inner().clone();
    blocking(move || jobs::export_cloud(&core, &ids, std::path::Path::new(&path), &color_mode)).await?
}

#[tauri::command]
pub async fn export_raw(core: Core<'_>, id: String, path: String) -> Result<(), String> {
    let core = core.inner().clone();
    blocking(move || jobs::export_raw(&core, &id, std::path::Path::new(&path))).await?
}

#[tauri::command]
pub async fn export_mesh(core: Core<'_>, path: String) -> Result<(), String> {
    let core = core.inner().clone();
    blocking(move || jobs::export_mesh(&core, std::path::Path::new(&path))).await?
}

// --- processing ------------------------------------------------------------------------------

#[tauri::command]
pub fn merge_start(core: Core<'_>, ids: Vec<String>) -> Result<(), String> {
    jobs::merge_start(&core, &ids)
}

#[tauri::command]
pub fn merge_decide(core: Core<'_>, accept: bool) {
    jobs::merge_decide(&core, accept);
}

#[tauri::command]
pub fn merge_cancel(core: Core<'_>) {
    jobs::merge_cancel(&core);
}

#[tauri::command]
pub fn mesh_start(core: Core<'_>, ids: Vec<String>) -> Result<String, String> {
    jobs::mesh_start(&core, &ids)
}

#[tauri::command]
pub fn mesh_cancel(core: Core<'_>) {
    core.mesh_cancel.store(true, std::sync::atomic::Ordering::SeqCst);
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct MeshState {
    key: Option<String>,
    info: Option<MeshInfo>,
    busy: bool,
}

/// The mesh on hand and whether one is being built. `ids` asks for the key
/// that selection would have, so the UI can tell if the mesh is current.
#[tauri::command]
pub fn mesh_state(core: Core<'_>, ids: Vec<String>) -> (MeshState, String) {
    let scans = jobs::resolve(&core, &ids);
    let want = jobs::mesh_key(&core, &scans);
    let guard = core.mesh.lock();
    (
        MeshState {
            key: guard.as_ref().map(|m| m.key.clone()),
            info: guard.as_ref().map(|m| m.info.clone()),
            busy: core.mesh_busy.load(std::sync::atomic::Ordering::SeqCst),
        },
        want,
    )
}

#[tauri::command]
pub async fn mesh_data(core: Core<'_>) -> Result<Response, String> {
    let core = core.inner().clone();
    blocking(move || {
        let guard = core.mesh.lock();
        guard.as_ref().map(|m| Response::new(jobs::mesh_bytes(&m.mesh))).ok_or_else(|| "no-mesh".to_string())
    })
    .await?
}

#[tauri::command]
pub fn calibrate_start(core: Core<'_>, id: String) -> Result<(), String> {
    jobs::calibrate_start(&core, &id)
}

#[tauri::command]
pub fn calibrate_cancel(core: Core<'_>) {
    core.calib_cancel.store(true, std::sync::atomic::Ordering::SeqCst);
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CalibApply {
    rotation: f64,
    spacing: f64,
    tilt: f64,
    range_error: Option<Vec<f64>>,
}

#[tauri::command]
pub fn calibrate_apply(core: Core<'_>, fit: CalibApply) -> Settings {
    let has_range = fit.range_error.is_some();
    core.update_settings(&serde_json::json!({
        "mount": {
            "lidarRotation": fit.rotation,
            "emitterSpacing": fit.spacing,
            "scanTilt": fit.tilt,
            "rangeError": fit.range_error,
            "rangeCorrection": has_range,
        }
    }))
}

#[tauri::command]
pub fn library_root(core: Core<'_>) -> String {
    core.library.lock().root.to_string_lossy().into_owned()
}

#[tauri::command]
pub fn new_scan_id() -> String {
    library::new_id()
}
