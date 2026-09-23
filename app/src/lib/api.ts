// The backend, behind one small interface. Inside the app it is the Rust
// core over Tauri IPC; in a plain browser (UI development) it is a mock that
// replays real captures exported to /dev-assets, so every screen can be
// exercised without the device.

import type { Bootstrap, CalibEvent, DeviceStatus, LibraryDto, MergeEvent, MeshEvent, MeshInfo, ScanProgress, Settings } from "./types";

export const IS_TAURI = typeof window !== "undefined" && "__TAURI_INTERNALS__" in window;

type Unlisten = () => void;

export interface Events {
  device: DeviceStatus;
  "device-log": string;
  /** Settings changed by the backend itself (calibration read from the scanner). */
  settings: Settings;
  scan: ScanProgress;
  "scan-finished": { id: string; complete: boolean; points: number };
  "scan-error": { code: string; detail: string | null };
  "scan-cancelled": string;
  library: LibraryDto;
  geometry: number;
  merge: MergeEvent;
  mesh: MeshEvent;
  calib: CalibEvent;
}

export interface Backend {
  bootstrap(): Promise<Bootstrap>;
  settingsUpdate(patch: unknown): Promise<Settings>;
  rangeErrorAt(model: number[] | null): Promise<number>;
  deviceConnect(port: string): Promise<void>;
  deviceDisconnect(): Promise<void>;
  deviceCommand(cmd: "home" | "unwrap" | "beep" | "status"): Promise<boolean>;
  scanStart(): Promise<ScanProgress>;
  scanStop(): Promise<void>;
  livePoints(seq: number, since: number): Promise<ArrayBuffer>;
  scanCloud(id: string): Promise<ArrayBuffer>;
  scanRename(id: string, name: string | null): Promise<void>;
  scansDelete(ids: string[]): Promise<void>;
  scansUngroup(ids: string[]): Promise<void>;
  thumbnailSave(id: string, png: Uint8Array): Promise<void>;
  thumbnail(id: string): Promise<ArrayBuffer>;
  importFiles(paths: string[]): Promise<{ added: string[]; failed: string[] }>;
  exportCloud(ids: string[], path: string, colorMode: string): Promise<number>;
  exportRaw(id: string, path: string): Promise<void>;
  exportMesh(path: string): Promise<void>;
  mergeStart(ids: string[]): Promise<void>;
  mergeDecide(accept: boolean): Promise<void>;
  mergeCancel(): Promise<void>;
  meshStart(ids: string[]): Promise<string>;
  meshCancel(): Promise<void>;
  meshState(ids: string[]): Promise<[{ key: string | null; info: MeshInfo | null; busy: boolean }, string]>;
  meshData(): Promise<ArrayBuffer>;
  calibrateStart(id: string): Promise<void>;
  calibrateCancel(): Promise<void>;
  calibrateApply(fit: { rotation: number; spacing: number; tilt: number; rangeError: number[] | null }): Promise<Settings>;
  on<K extends keyof Events>(event: K, fn: (payload: Events[K]) => void): Promise<Unlisten>;
  // Dialogs and shell.
  pickFiles(): Promise<string[]>;
  pickFolder(current?: string): Promise<string | null>;
  saveAs(defaultName: string, filters: { name: string; extensions: string[] }[]): Promise<string | null>;
  reveal(path: string): Promise<void>;
  openFolder(path: string): Promise<void>;
  onDrop(fn: (paths: string[]) => void, onHover?: (over: boolean) => void): Promise<Unlisten>;
  // Window.
  windowShow(): Promise<void>;
  windowMinimize(): Promise<void>;
  windowToggleMaximize(): Promise<void>;
  windowClose(): Promise<void>;
  windowIsMaximized(): Promise<boolean>;
  onWindowResize(fn: () => void): Promise<Unlisten>;
  onCloseRequested(fn: () => Promise<boolean>): Promise<Unlisten>;
  setBackground(hex: string): Promise<void>;
}

async function tauriBackend(): Promise<Backend> {
  const core = await import("@tauri-apps/api/core");
  const ev = await import("@tauri-apps/api/event");
  const win = await import("@tauri-apps/api/window");
  const wv = await import("@tauri-apps/api/webview");
  const dialog = await import("@tauri-apps/plugin-dialog");
  const opener = await import("@tauri-apps/plugin-opener");
  const invoke = core.invoke;
  const w = win.getCurrentWindow();
  return {
    bootstrap: () => invoke("bootstrap"),
    settingsUpdate: (patch) => invoke("settings_update", { patch }),
    rangeErrorAt: (model) => invoke("range_error_at", { model }),
    deviceConnect: (port) => invoke("device_connect", { port }),
    deviceDisconnect: () => invoke("device_disconnect"),
    deviceCommand: (cmd) => invoke("device_command", { cmd }),
    scanStart: () => invoke("scan_start"),
    scanStop: () => invoke("scan_stop"),
    livePoints: (seq, since) => invoke("live_points", { seq, since }),
    scanCloud: (id) => invoke("scan_cloud", { id }),
    scanRename: (id, name) => invoke("scan_rename", { id, name }),
    scansDelete: (ids) => invoke("scans_delete", { ids }),
    scansUngroup: (ids) => invoke("scans_ungroup", { ids }),
    thumbnailSave: (id, png) => invoke("thumbnail_save", png, { headers: { "x-scan-id": id } }),
    thumbnail: (id) => invoke("thumbnail", { id }),
    importFiles: (paths) => invoke("import_files", { paths }),
    exportCloud: (ids, path, colorMode) => invoke("export_cloud", { ids, path, colorMode }),
    exportRaw: (id, path) => invoke("export_raw", { id, path }),
    exportMesh: (path) => invoke("export_mesh", { path }),
    mergeStart: (ids) => invoke("merge_start", { ids }),
    mergeDecide: (accept) => invoke("merge_decide", { accept }),
    mergeCancel: () => invoke("merge_cancel"),
    meshStart: (ids) => invoke("mesh_start", { ids }),
    meshCancel: () => invoke("mesh_cancel"),
    meshState: (ids) => invoke("mesh_state", { ids }),
    meshData: () => invoke("mesh_data"),
    calibrateStart: (id) => invoke("calibrate_start", { id }),
    calibrateCancel: () => invoke("calibrate_cancel"),
    calibrateApply: (fit) => invoke("calibrate_apply", { fit }),
    on: (event, fn) => ev.listen(event, (e) => fn(e.payload as never)),
    pickFiles: async () => {
      const r = await dialog.open({
        multiple: true,
        filters: [{ name: "XScan3D", extensions: ["bin", "ply"] }],
      });
      return r ? (Array.isArray(r) ? r : [r]) : [];
    },
    pickFolder: async (current) => {
      const r = await dialog.open({ directory: true, defaultPath: current });
      return typeof r === "string" ? r : null;
    },
    saveAs: (defaultName, filters) => dialog.save({ defaultPath: defaultName, filters }),
    reveal: (path) => opener.revealItemInDir(path),
    openFolder: (path) => opener.openPath(path),
    onDrop: (fn, onHover) =>
      wv.getCurrentWebview().onDragDropEvent((e) => {
        const p = e.payload;
        if (p.type === "drop") {
          onHover?.(false);
          fn(p.paths);
        } else if (p.type === "enter" || p.type === "over") onHover?.(true);
        else onHover?.(false);
      }),
    windowShow: () => w.show(),
    windowMinimize: () => w.minimize(),
    windowToggleMaximize: () => w.toggleMaximize(),
    windowClose: () => w.close(),
    windowIsMaximized: () => w.isMaximized(),
    onWindowResize: (fn) => w.onResized(() => fn()),
    onCloseRequested: (fn) =>
      w.onCloseRequested(async (e) => {
        const ok = await fn();
        if (!ok) e.preventDefault();
      }),
    setBackground: async (hex) => {
      const c = hexToRgb(hex);
      try {
        await w.setBackgroundColor(c);
        await wv.getCurrentWebview().setBackgroundColor(c);
      } catch {
        /* older runtimes */
      }
    },
  };
}

function hexToRgb(hex: string): [number, number, number, number] {
  const v = parseInt(hex.replace("#", ""), 16);
  return [(v >> 16) & 255, (v >> 8) & 255, v & 255, 255];
}

let backend: Backend | null = null;

export async function api(): Promise<Backend> {
  if (backend) return backend;
  backend = IS_TAURI ? await tauriBackend() : (await import("./mock")).mockBackend();
  return backend;
}

/** Synchronous handle once `api()` has resolved (after startup). */
export function B(): Backend {
  if (!backend) throw new Error("backend not ready");
  return backend;
}
