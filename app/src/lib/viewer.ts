// Glue between the app state and the renderer: which clouds are loaded and
// shown, the live cloud during a sweep, the alignment preview and the mesh.

import { B } from "./api";
import { Controls } from "./render/Controls";
import { LIVE_MAGIC, parseCloud, parseMesh } from "./render/layers";
import type { Vec3 } from "./render/math";
import { Renderer } from "./render/Renderer";
import { app, toast } from "./state.svelte";
import { t } from "./i18n.svelte";
import type { ScanMeta, ScanProgress } from "./types";

export const view = {
  r: null as Renderer | null,
  controls: null as Controls | null,
  /** Geometry version each loaded cloud was built with. */
  loaded: new Map<string, number>(),
  shownKey: "",
};

export function attach(canvas: HTMLCanvasElement, onSpeed: (v: number) => void, onMove: () => void): Renderer {
  const r = new Renderer(canvas);
  view.r = r;
  view.controls = new Controls(r, { onSpeed, onUserMove: onMove });
  return r;
}

export function detach() {
  view.controls?.dispose();
  view.r?.dispose();
  view.r = null;
  view.controls = null;
  view.loaded.clear();
}

let loadEpoch = 0;

/** Loads what `scans` needs, then shows exactly those. */
export async function showScans(scans: ScanMeta[]) {
  const r = view.r;
  if (!r) return;
  const epoch = ++loadEpoch;
  const key = scans.map((s) => s.id).sort().join("|");
  const setChanged = key !== view.shownKey;
  const gv = app.geometryVersion;
  const missing = scans.filter((s) => view.loaded.get(s.id) !== gv || !r.clouds.has(s.id));
  const firstTime = missing.filter((s) => !r.clouds.has(s.id));
  if (missing.length) app.loading = missing.map((s) => s.id);
  let framed = false;
  for (const s of missing) {
    try {
      const buf = await B().scanCloud(s.id);
      if (epoch !== loadEpoch) return;
      const data = parseCloud(buf);
      // Fresh scans sweep into view; a rebuild after a calibration change
      // swaps in place.
      const reveal = firstTime.includes(s) && !justScanned.has(s.id);
      justScanned.delete(s.id);
      r.setCloud(s.id, data, s.pose, reveal);
      view.loaded.set(s.id, gv);
      if (setChanged && !framed) {
        applyVisible(scans);
        r.frame(scans.length > 1 ? 900 : 800);
        framed = true;
      }
    } catch (e) {
      if (epoch !== loadEpoch) return;
      toast(`${t("library.loadFailed")}: ${e}`, "error");
    }
  }
  if (epoch !== loadEpoch) return;
  app.loading = [];
  applyVisible(scans);
  app.sceneTick++;
  if (setChanged) {
    view.shownKey = key;
    if (!framed && scans.length) r.frame(800);
    // The final cloud of a sweep has replaced the live one.
    if (!app.scan) r.endLive();
  }
  r.trim();
  maybeThumbnail();
}

/** The height to cut the view at, from the clip mode and the scene. */
export function effectiveClip(): number {
  const r = view.r;
  if (!r) return Infinity;
  const st = r.stats;
  const auto = st.ceiling !== null ? st.ceiling - 180 : Infinity;
  if (app.clipMode === "auto") return auto;
  if (app.clipMode === "on") return app.clipZ ?? (Number.isFinite(auto) ? auto : st.zhi);
  return Infinity;
}

/** Poses, colours and highlights for the visible set (no loading). */
export function applyVisible(scans: ScanMeta[]) {
  const r = view.r;
  if (!r) return;
  const pending = app.merge?.pending ?? null;
  const poses: Record<string, number[] | null> = {};
  for (const s of scans) poses[s.id] = pending?.poses[s.id] ?? s.pose;
  r.showOnly(
    scans.map((s) => s.id).filter((id) => r.clouds.has(id)),
    poses,
  );
  for (const s of scans) {
    const l = r.clouds.get(s.id);
    if (l) l.highlight = pending?.unit.includes(s.id) ? 0.55 : 0;
  }
  const markers: Vec3[] = scans.map((s) => {
    const p = poses[s.id] ?? s.pose;
    return [p[3], p[7], p[11]];
  });
  r.setMarkers(app.scan ? [] : markers);
  r.requestRender();
}

// --- live sweep ------------------------------------------------------------

const justScanned = new Set<string>();
let liveSeq = 0;
let liveSince = 0;
let polling = false;
let planeAngle = 0;

export function onScan(p: ScanProgress | null) {
  const r = view.r;
  if (!r) return;
  if (!p) {
    r.scanPlane.visible = false;
    r.requestRender();
    return;
  }
  if (p.seq !== liveSeq) {
    liveSeq = p.seq;
    liveSince = 0;
    justScanned.add(p.id);
    r.beginLive();
    r.showOnly([], {});
    r.setMarkers([]);
    r.mesh && (r.showMesh = false);
    view.shownKey = "";
    // A room-sized view around the scanner while points arrive.
    r.camera.animateTo({ target: [0, 0, 0], distance: 7500, azimuth: -55, elevation: 32 }, 900);
  }
  planeAngle = p.platform;
  r.scanPlane.visible = p.phase === "preparing" || p.phase === "scanning";
  if (!polling) void poll();
  r.requestRender();
}

/** Called every frame: eases the scan plane towards the reported angle. */
export function animatePlane() {
  const r = view.r;
  if (!r || !r.scanPlane.visible) return;
  const a = r.scanPlane.angleDeg;
  r.scanPlane.angleDeg = a + (planeAngle - a) * 0.2;
}

async function poll() {
  polling = true;
  try {
    while (app.scan && app.scan.seq === liveSeq && view.r) {
      const buf = await B().livePoints(liveSeq, liveSince);
      const dv = new DataView(buf);
      if (buf.byteLength >= 20 && dv.getUint32(0, true) === LIVE_MAGIC && dv.getUint32(4, true) === liveSeq) {
        const total = dv.getUint32(8, true);
        const since = dv.getUint32(12, true);
        const n = dv.getUint32(16, true);
        if (n > 0 && since === liveSince && view.r?.live) {
          const pos = new Float32Array(buf.slice(20, 20 + n * 12));
          const range = new Uint16Array(buf.slice(20 + n * 12, 20 + n * 14));
          const order = new Uint16Array(buf.slice(20 + n * 14, 20 + n * 16));
          view.r.live.append(pos, range, order);
          liveSince = total;
          view.r.requestRender();
        }
      }
      await new Promise((res) => setTimeout(res, 70));
    }
  } catch {
    /* the sweep ended under us */
  } finally {
    polling = false;
  }
}

// --- surface -------------------------------------------------------------------

export async function syncMesh() {
  const r = view.r;
  if (!r) return;
  const m = app.mesh;
  const current = m.show && m.key !== null && m.key === m.wantKey && !app.scan;
  if (current && r.mesh?.key !== m.key) {
    try {
      const data = parseMesh(await B().meshData());
      r.setMesh(m.key!, data);
    } catch (e) {
      toast(`${t("mesh.failed")}: ${e}`, "error");
      return;
    }
  }
  r.showMesh = current && r.mesh !== null;
  r.requestRender();
}

// --- thumbnails ----------------------------------------------------------------

let thumbTimer = 0;

function maybeThumbnail() {
  window.clearTimeout(thumbTimer);
  if (app.selection.length !== 1 || app.scan) return;
  const id = app.selection[0];
  if (app.thumbs[id]) return;
  thumbTimer = window.setTimeout(
    () =>
      idle(async () => {
        const r = view.r;
        if (!r || app.selection.length !== 1 || app.selection[0] !== id || app.scan) return;
        if ([...r.clouds.values()].some((l) => l.visible && l.uploading)) return maybeThumbnail();
        const png = await r.thumbnail();
        if (!png) return;
        await B().thumbnailSave(id, png);
        app.thumbs[id] = URL.createObjectURL(new Blob([png as BlobPart], { type: "image/png" }));
      }),
    1400,
  );
}

/** Runs `fn` once the browser is not busy (with a timeout as a backstop). */
function idle(fn: () => void) {
  const ric = (window as unknown as { requestIdleCallback?: (cb: () => void, o?: { timeout: number }) => void }).requestIdleCallback;
  if (ric) ric(fn, { timeout: 2500 });
  else setTimeout(fn, 0);
}

export async function loadThumb(id: string) {
  if (app.thumbs[id]) return;
  try {
    const buf = await B().thumbnail(id);
    if (buf.byteLength > 0) app.thumbs[id] = URL.createObjectURL(new Blob([buf], { type: "image/png" }));
  } catch {
    /* none yet */
  }
}

/** Forget a thumbnail (its scan was recalibrated or regrouped). */
export function dropThumb(id: string) {
  const u = app.thumbs[id];
  if (u) URL.revokeObjectURL(u);
  delete app.thumbs[id];
}
