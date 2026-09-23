// Application state and the actions the UI calls. One reactive object; the
// backend pushes events into it and components read from it.

import { api, B } from "./api";
import { dateTime } from "./format";
import { i18n, plural, systemLang, t } from "./i18n.svelte";
import type {
  CalibResult, CalibStage, DeviceStatus, LibraryDto, LibraryItem, MergeEvent, MeshInfo, MeshStage, Reason, ScanMeta, ScanProgress,
  Settings, Verdict,
} from "./types";

export interface Toast {
  id: number;
  text: string;
  kind: "info" | "success" | "error";
  action?: { label: string; run: () => void };
}

export interface PendingAlign {
  unit: string[];
  poses: Record<string, number[]>;
  verdict: Verdict;
  reasons: Reason[];
  overlap: number;
  rmse: number;
}

export interface Confirm {
  title: string;
  body: string;
  ok: string;
  danger?: boolean;
  resolve: (ok: boolean) => void;
}

const EMPTY_DEVICE: DeviceStatus = { phase: "searching", port: null, state: 0, platform: 0, dropped: 0, config: null, error: null, bytesIn: 0, ports: [], calib: "" };

class AppState {
  ready = $state(false);
  version = $state("");
  logDir = $state("");
  settings = $state<Settings>(null as unknown as Settings);
  library = $state<LibraryDto>({ root: "", index: { version: 1, scans: [], groups: [] } });
  device = $state<DeviceStatus>(EMPTY_DEVICE);
  deviceLog = $state<string[]>([]);
  scan = $state<ScanProgress | null>(null);
  countdown = $state(0);
  selection = $state<string[]>([]);
  systemDark = $state(true);
  rangeAt3m = $state(0);
  geometryVersion = $state(1);
  thumbs = $state<Record<string, string>>({});
  loading = $state<string[]>([]);

  sidebarOpen = $state(true);
  settingsOpen = $state(false);
  settingsSection = $state("general");
  exportOpen = $state(false);
  confirm = $state<Confirm | null>(null);
  dropHover = $state(false);

  merge = $state<{ text: string; index: number; total: number; pending: PendingAlign | null } | null>(null);
  mesh = $state<{ busy: boolean; stage: MeshStage | null; key: string | null; info: MeshInfo | null; show: boolean; wantKey: string | null }>({
    busy: false, stage: null, key: null, info: null, show: false, wantKey: null,
  });
  calib = $state<{ running: boolean; stage: CalibStage | null; fraction: number; result: CalibResult | null; scan: string | null }>({
    running: false, stage: null, fraction: 0, result: null, scan: null,
  });
  toasts = $state<Toast[]>([]);
  /** Cut from the top: "auto" removes a detected ceiling (dollhouse view). */
  clipMode = $state<"auto" | "on" | "off">("auto");
  /** Manual cut height (world mm) in "on" mode. */
  clipZ = $state<number | null>(null);
  /** Bumped when the visible set's statistics change, so the UI re-reads them. */
  sceneTick = $state(0);

  theme = $derived.by((): "light" | "dark" => {
    const pref = this.settings?.theme ?? "system";
    return pref === "system" ? (this.systemDark ? "dark" : "light") : pref;
  });

  /** Library rows: merged groups and loose scans, newest first. */
  items = $derived.by((): LibraryItem[] => {
    const idx = this.library.index;
    const out: LibraryItem[] = [];
    for (const g of idx.groups) {
      const scans = idx.scans.filter((s) => s.group === g.id);
      if (scans.length < 1) continue;
      const created = scans.map((s) => s.created).sort().at(-1) ?? g.created;
      out.push({ id: g.id, kind: "group", created, scans, name: g.name });
    }
    for (const s of idx.scans) if (!s.group) out.push({ id: s.id, kind: "scan", created: s.created, scans: [s], name: s.name });
    out.sort((a, b) => (a.created < b.created ? 1 : a.created > b.created ? -1 : 0));
    return out;
  });

  /** Scans on screen: the members of every selected row. */
  visibleScans = $derived.by((): ScanMeta[] => {
    const out: ScanMeta[] = [];
    for (const id of this.selection) {
      const it = this.items.find((i) => i.id === id);
      if (it) for (const s of it.scans) if (!out.includes(s)) out.push(s);
    }
    return out;
  });

  scanning = $derived(this.scan !== null);
  connected = $derived(this.device.phase === "ready");
}

export const app = new AppState();

// --- helpers -------------------------------------------------------------

let toastId = 0;
export function toast(text: string, kind: Toast["kind"] = "info", action?: Toast["action"], ms = 3800) {
  const id = ++toastId;
  app.toasts = [...app.toasts, { id, text, kind, action }].slice(-3);
  setTimeout(() => (app.toasts = app.toasts.filter((x) => x.id !== id)), ms);
}

export function ask(c: Omit<Confirm, "resolve">): Promise<boolean> {
  return new Promise((resolve) => {
    app.confirm = { ...c, resolve: (ok) => { app.confirm = null; resolve(ok); } };
  });
}

export function itemTitle(it: LibraryItem): string {
  if (it.name) return it.name;
  if (it.kind === "group") return `${t("library.merged")} · ${dateTime(it.created)}`;
  return `${t("library.scan")} · ${dateTime(it.created)}`;
}

export function scanTitle(s: ScanMeta): string {
  return s.name || `${t("library.scan")} · ${dateTime(s.created)}`;
}

export function describeSelection(): string {
  const items = app.items.filter((i) => app.selection.includes(i.id));
  if (items.length === 1) return itemTitle(items[0]);
  const n = app.visibleScans.length;
  return `${n} ${plural("scans", n)}`;
}

function applyLanguage() {
  const pref = app.settings?.language ?? "system";
  i18n.lang = pref === "system" ? systemLang() : pref;
  document.documentElement.lang = i18n.lang;
}

function errText(prefix: string, e: unknown): string {
  const code = typeof e === "string" ? e : (e as Error)?.message ?? String(e);
  const key = `${prefix}.${code}`;
  const s = t(key);
  return s === key ? code : s;
}

// --- startup --------------------------------------------------------------

let started = false;

export async function init() {
  // Guard against a second run (a dev hot reload) stacking up listeners.
  if (started) return;
  started = true;
  const b = await api();
  const boot = await b.bootstrap();
  app.settings = boot.settings;
  app.library = boot.library;
  app.device = boot.device.phase ? boot.device : EMPTY_DEVICE;
  app.version = boot.version;
  app.logDir = boot.logDir;
  app.rangeAt3m = boot.rangeAt3m;
  applyLanguage();

  const mq = window.matchMedia("(prefers-color-scheme: dark)");
  app.systemDark = mq.matches;
  mq.addEventListener("change", (e) => {
    app.systemDark = e.matches;
  });

  await b.on("device", (d) => {
    app.device = d;
  });
  await b.on("settings", (s) => {
    app.settings = s;
  });
  await b.on("device-log", (line) => {
    app.deviceLog = [...app.deviceLog.slice(-199), line];
  });
  await b.on("scan", (p) => {
    app.scan = p;
  });
  await b.on("scan-finished", (f) => {
    app.scan = null;
    app.selection = [f.id];
    toast(f.complete ? t("scan.saved") : t("scan.savedPartial"), "success", {
      label: t("view.export"),
      run: () => (app.exportOpen = true),
    });
  });
  await b.on("scan-error", (e) => {
    app.scan = null;
    toast(errText("scan.err", e.code), "error");
  });
  await b.on("scan-cancelled", () => {
    app.scan = null;
  });
  await b.on("library", (l) => {
    app.library = l;
    // Drop selections that no longer exist (deleted, regrouped).
    const ids = new Set([...l.index.scans.filter((s) => !s.group).map((s) => s.id), ...l.index.groups.map((g) => g.id)]);
    const kept = app.selection.filter((id) => ids.has(id));
    // A scan that just joined a group is now shown through the group.
    for (const id of app.selection) {
      const s = l.index.scans.find((x) => x.id === id);
      if (s?.group && !kept.includes(s.group)) kept.push(s.group);
    }
    app.selection = kept;
  });
  await b.on("geometry", (v) => {
    app.geometryVersion = v;
    b.rangeErrorAt(app.settings.mount.rangeCorrection ? app.settings.mount.rangeError : null).then((mm) => (app.rangeAt3m = mm));
  });
  await b.on("merge", onMerge);
  await b.on("mesh", (e) => {
    if (e.kind === "progress") app.mesh.stage = e.stage;
    else if (e.kind === "done") {
      app.mesh = { ...app.mesh, busy: false, stage: null, key: e.key, info: e.info, show: true };
      toast(t("mesh.done", { s: e.info.seconds.toFixed(1) }), "success");
    } else {
      app.mesh = { ...app.mesh, busy: false, stage: null };
      toast(`${t("mesh.failed")}: ${e.detail}`, "error");
    }
  });
  await b.on("calib", (e) => {
    if (e.kind === "progress") app.calib = { ...app.calib, running: true, stage: e.stage, fraction: e.fraction };
    else if (e.kind === "done") app.calib = { running: false, stage: null, fraction: 1, result: e.result, scan: e.scan };
    else {
      app.calib = { running: false, stage: null, fraction: 0, result: null, scan: null };
      toast(errText("settings.calibErr", e.code), e.code === "cancelled" ? "info" : "error");
    }
  });
  await b.onDrop(
    (paths) => importPaths(paths),
    (over) => {
      app.dropHover = over;
    },
  );
  await b.onCloseRequested(async () => {
    if (!app.scanning) return true;
    const ok = await ask({ title: t("closeScan.title"), body: t("closeScan.body"), ok: t("closeScan.ok"), danger: true });
    if (ok) await b.scanStop();
    return ok;
  });

  // Start on the newest scan, so the viewport is never empty for no reason.
  if (app.items.length) app.selection = [app.items[0].id];
  app.ready = true;
}

// --- settings --------------------------------------------------------------

function mergeInto(target: Record<string, unknown>, patch: Record<string, unknown>) {
  for (const [k, v] of Object.entries(patch)) {
    if (v && typeof v === "object" && !Array.isArray(v) && target[k] && typeof target[k] === "object") {
      mergeInto(target[k] as Record<string, unknown>, v as Record<string, unknown>);
    } else target[k] = v;
  }
}

let settingsQueue: Promise<unknown> = Promise.resolve();

/** Applies a partial settings change at once, and persists it in order. */
export function updateSettings(patch: Record<string, unknown>) {
  mergeInto(app.settings as unknown as Record<string, unknown>, structuredClone(patch));
  if ("language" in patch) applyLanguage();
  settingsQueue = settingsQueue.then(() => B().settingsUpdate(patch)).catch((e) => toast(String(e), "error"));
}

// --- scanning --------------------------------------------------------------

let countdownTimer = 0;

export async function startScan() {
  if (app.scanning || app.countdown > 0) return;
  const delay = app.settings.scan.startDelay;
  if (delay > 0) {
    app.countdown = delay;
    countdownTimer = window.setInterval(() => {
      app.countdown -= 1;
      if (app.countdown <= 0) {
        window.clearInterval(countdownTimer);
        app.countdown = 0;
        void begin();
      }
    }, 1000);
    return;
  }
  await begin();
}

export function cancelCountdown() {
  window.clearInterval(countdownTimer);
  app.countdown = 0;
}

async function begin() {
  try {
    app.mesh.show = false;
    app.scan = await B().scanStart();
  } catch (e) {
    app.scan = null;
    toast(errText("scan.err", e), "error");
  }
}

export async function stopScan() {
  const ok = await ask({ title: t("scan.stopTitle"), body: t("scan.stopBody"), ok: t("scan.stop"), danger: true });
  if (ok) await B().scanStop();
}

// --- selection & library ---------------------------------------------------

export function select(id: string, mode: "single" | "toggle" | "range" = "single") {
  if (mode === "single") {
    app.selection = [id];
  } else if (mode === "toggle") {
    app.selection = app.selection.includes(id) ? app.selection.filter((x) => x !== id) : [...app.selection, id];
  } else {
    const ids = app.items.map((i) => i.id);
    const last = app.selection.at(-1);
    const a = last ? ids.indexOf(last) : -1;
    const b = ids.indexOf(id);
    if (a < 0 || b < 0) app.selection = [id];
    else {
      const [lo, hi] = a < b ? [a, b] : [b, a];
      const range = ids.slice(lo, hi + 1);
      app.selection = [...app.selection.filter((x) => !range.includes(x)), ...range];
    }
  }
}

export async function renameItem(id: string, name: string) {
  await B().scanRename(id, name.trim() || null);
}

export async function deleteItems(ids: string[]) {
  const items = app.items.filter((i) => ids.includes(i.id));
  if (!items.length) return;
  const n = items.reduce((a, i) => a + i.scans.length, 0);
  const what = items.length === 1 ? `«${itemTitle(items[0])}»` : `${n} ${plural("scans", n)}`;
  const ok = await ask({ title: t("library.deleteTitle"), body: t("library.deleteBody", { what }), ok: t("library.deleteOk"), danger: true });
  if (!ok) return;
  app.selection = app.selection.filter((x) => !ids.includes(x));
  await B().scansDelete(ids);
}

export async function ungroupItems(ids: string[]) {
  await B().scansUngroup(ids);
}

export async function importPaths(paths: string[]) {
  if (!paths.length) return;
  const r = await B().importFiles(paths);
  if (r.added.length) {
    toast(t("library.importDone", { n: r.added.length }), "success");
    app.selection = [...r.added];
  }
  if (r.failed.length) {
    const names = r.failed.map((p) => p.split(/[\\/]/).pop()).join(", ");
    toast(t("library.importFailed", { names }), "error");
  }
}

export async function openFiles() {
  const paths = await B().pickFiles();
  await importPaths(paths);
}

// --- merge -------------------------------------------------------------------

export async function mergeSelection() {
  try {
    app.merge = { text: t("merge.running"), index: 0, total: 0, pending: null };
    await B().mergeStart([...app.selection]);
  } catch (e) {
    app.merge = null;
    toast(errText("merge.err", e), "error");
  }
}

function onMerge(e: MergeEvent) {
  if (e.kind === "progress") {
    app.merge = { text: t(`merge.${e.stage}`), index: e.index, total: e.total, pending: null };
  } else if (e.kind === "pending") {
    app.merge = {
      text: t("merge.reviewTitle"),
      index: app.merge?.index ?? 0,
      total: app.merge?.total ?? 0,
      pending: { unit: e.unit, poses: Object.fromEntries(e.poses), verdict: e.verdict, reasons: e.reasons, overlap: e.overlap, rmse: e.rmse },
    };
  } else if (e.kind === "accepted" || e.kind === "rejected") {
    if (app.merge) app.merge = { ...app.merge, pending: null };
  } else if (e.kind === "done") {
    app.merge = null;
    if (e.group) {
      app.selection = [e.group];
      toast(t("merge.done"), "success");
    } else if (e.merged === 0) toast(t("merge.nothing"));
  } else {
    app.merge = null;
    toast(errText("merge.err", e.code), "error");
  }
}

export async function decideMerge(accept: boolean) {
  await B().mergeDecide(accept);
}

export async function cancelMerge() {
  await B().mergeCancel();
  app.merge = null;
}

// --- surface --------------------------------------------------------------------

export async function buildMesh() {
  if (app.mesh.busy) return;
  try {
    app.mesh = { ...app.mesh, busy: true, stage: "thinning" };
    await B().meshStart([...app.selection]);
  } catch (e) {
    app.mesh = { ...app.mesh, busy: false, stage: null };
    toast(`${t("mesh.failed")}: ${e}`, "error");
  }
}

export async function refreshMeshKey() {
  if (!app.selection.length) {
    app.mesh.wantKey = null;
    return;
  }
  const [st, want] = await B().meshState([...app.selection]);
  app.mesh = { ...app.mesh, key: st.key, info: st.info, busy: st.busy, wantKey: want };
}

// --- calibration -----------------------------------------------------------------

export function calibrationScan(): ScanMeta | null {
  const raw = app.visibleScans.filter((s) => s.kind !== "ply");
  if (raw.length) return raw[0];
  return app.library.index.scans.filter((s) => s.kind !== "ply").sort((a, b) => (a.created < b.created ? 1 : -1))[0] ?? null;
}

export async function startCalibration() {
  const s = calibrationScan();
  if (!s) return;
  try {
    app.calib = { running: true, stage: "microstep", fraction: 0, result: null, scan: s.id };
    await B().calibrateStart(s.id);
  } catch (e) {
    app.calib = { running: false, stage: null, fraction: 0, result: null, scan: null };
    toast(errText("settings.calibErr", e), "error");
  }
}

export async function applyCalibration() {
  const r = app.calib.result;
  if (!r) return;
  app.settings = await B().calibrateApply({ rotation: r.rotation, spacing: r.spacing, tilt: r.tilt, rangeError: r.rangeError });
  app.calib = { running: false, stage: null, fraction: 0, result: null, scan: null };
  toast(t("settings.applied"), "success");
}
