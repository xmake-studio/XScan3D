// A stand-in backend for developing the UI in a plain browser. It serves real
// clouds exported to /dev-assets (see src-tauri/examples/dev_assets.rs) and
// simulates the device, a sweep, merging, meshing and calibration.

import type { Backend, Events } from "./api";
import { CLOUD_MAGIC, LIVE_MAGIC, parseCloud } from "./render/layers";
import type { Bootstrap, DeviceStatus, LibraryDto, ScanMeta, ScanProgress, Settings } from "./types";

type Handler = (p: unknown) => void;

const DEFAULT_SETTINGS: Settings = {
  language: "system",
  theme: "system",
  libraryDir: null,
  scan: { duration: 180, startDelay: 0, stepped: false, dwellMs: 400, sweepDeg: 0 },
  device: { autoConnect: true, dtr: true, knownSerials: [], calibPending: false },
  mount: {
    lidarRotation: 161.65, lidarReverse: true, emitterSpacing: -36.5, scanTilt: -1.2, scanHalf: "both",
    flipUpright: false, beamOffset: 0, minRange: 60, maxRange: 0, microstepAuto: true, rangeCorrection: true,
    rangeError: [24.7, 3.5, 2, 827, 3561, -0.08861, -0.01755, -0.003573, 0.1086, -0.01447, 0.01174, -0.1113, 0.01858, -0.003672, 0.0393, 0.06772, 0.01801],
  },
  view: { colorMode: "height", pointSize: 1, grid: true, edl: true, camera: "orbit", flySpeed: 1500 },
  mesh: { enabled: true, quality: "high", trim: 3, budget: 2500000, smooth: 0 },
  merge: { mode: "auto", yawHint: 0, voxel: 40, ignoreFoliage: true },
  flyHintSeen: false,
};

export function mockBackend(): Backend {
  const handlers = new Map<string, Set<Handler>>();
  const emit = <K extends keyof Events>(ev: K, payload: Events[K]) => {
    for (const h of handlers.get(ev) ?? []) h(payload);
  };
  let settings: Settings = structuredClone(DEFAULT_SETTINGS);
  try {
    const saved = localStorage.getItem("xscan-mock-settings");
    if (saved) settings = { ...settings, ...JSON.parse(saved) };
  } catch {
    /* no storage */
  }
  const library: LibraryDto = { root: "C:\\Users\\demo\\Documents\\XScan3D", index: { version: 1, scans: [], groups: [] } };
  let manifest: { scans: { id: string; points: number; created: string; duration: number }[] } = { scans: [] };
  const device: DeviceStatus = { phase: "searching", port: null, state: 0, platform: 0, dropped: 0, config: null, error: null, bytesIn: 0, ports: [], calib: "" };
  let scan: { seq: number; id: string; start: number; source: string; sorted: ArrayBuffer | null; total: number; timer: number } | null = null;
  let seq = 0;

  const loadManifest = async () => {
    try {
      manifest = await (await fetch("/dev-assets/manifest.json")).json();
    } catch {
      manifest = { scans: [] };
    }
    library.index.scans = manifest.scans.map((s): ScanMeta => ({
      id: s.id, name: null, created: s.created, kind: "device", file: `scans/${s.id}.bin`, points: s.points,
      duration: s.duration, sweepDeg: 90, complete: true, group: null,
      pose: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], microstep: null, sourceName: null, bytes: s.points * 5,
    }));
  };

  setTimeout(() => {
    device.phase = "ready";
    device.port = "COM4";
    device.config = { degrees: 90, time: 180, mode: 0, steps: 60, settleMs: 250, captureMs: 400 };
    device.ports = [{ device: "COM4", description: "USB Serial Device", vid: 0x2e8a, pid: 0xa, serial: "E6600000DEADBEEF", product: "XScan3D", isScanner: true }];
    device.calib = "synced";
    emit("device", { ...device });
  }, 1800);

  const progressOf = (): ScanProgress | null => {
    if (!scan) return null;
    const dur = Math.min(settings.scan.duration, 25);
    const el = (performance.now() - scan.start) / 1000;
    let phase: ScanProgress["phase"] = "starting";
    let progress = 0;
    if (el > 0.6) phase = "preparing";
    if (el > 2.6) {
      phase = "scanning";
      progress = Math.min(1, (el - 2.6) / dur);
    }
    if (el > 2.6 + dur) phase = "finishing";
    const points = Math.floor(scan.total * progress);
    return {
      id: scan.id, seq: scan.seq, phase, progress, platform: -90 + 180 * progress, sweepDeg: 90, points,
      elapsed: Math.max(0, el - 2.6), remaining: phase === "scanning" ? settings.scan.duration * (1 - progress) : settings.scan.duration, duration: settings.scan.duration,
    };
  };

  const sortedCloud = async (id: string): Promise<{ buf: ArrayBuffer; n: number }> => {
    const buf = await (await fetch(`/dev-assets/${id}.xsc`)).arrayBuffer();
    const c = parseCloud(buf);
    const n = c.header.count;
    const idx = new Uint32Array(n);
    for (let i = 0; i < n; i++) idx[i] = i;
    idx.sort((a, b) => c.order[a] - c.order[b]);
    const out = new ArrayBuffer(n * 16);
    const pos = new Float32Array(out, 0, n * 3), rng = new Uint16Array(out, n * 12, n), ord = new Uint16Array(out, n * 14, n);
    for (let i = 0; i < n; i++) {
      const j = idx[i];
      pos[i * 3] = c.pos[j * 3];
      pos[i * 3 + 1] = c.pos[j * 3 + 1];
      pos[i * 3 + 2] = c.pos[j * 3 + 2];
      rng[i] = c.range[j];
      ord[i] = c.order[j];
    }
    return { buf: out, n };
  };

  const b: Backend = {
    async bootstrap(): Promise<Bootstrap> {
      await loadManifest();
      return { settings, library: structuredClone(library), device: { ...device }, version: "1.0.0-dev", logDir: "C:\\logs", rangeAt3m: 7.8, rangeDefault: DEFAULT_SETTINGS.mount.rangeError! };
    },
    async settingsUpdate(patch) {
      const merge = (a: Record<string, unknown>, p: Record<string, unknown>) => {
        for (const [k, v] of Object.entries(p)) {
          if (v && typeof v === "object" && !Array.isArray(v) && a[k] && typeof a[k] === "object") merge(a[k] as Record<string, unknown>, v as Record<string, unknown>);
          else a[k] = v;
        }
      };
      const before = JSON.stringify(settings.mount);
      merge(settings as unknown as Record<string, unknown>, patch as Record<string, unknown>);
      try {
        localStorage.setItem("xscan-mock-settings", JSON.stringify(settings));
      } catch {
        /* no storage */
      }
      if (JSON.stringify(settings.mount) !== before) emit("geometry", Date.now());
      return structuredClone(settings);
    },
    async rangeErrorAt(model) {
      return model ? 7.8 : 0;
    },
    async deviceConnect() {},
    async deviceDisconnect() {},
    async deviceCommand() {
      return true;
    },
    async scanStart() {
      if (!manifest.scans.length) throw "not-connected";
      const src = manifest.scans[0].id;
      const { buf, n } = await sortedCloud(src);
      seq += 1;
      scan = { seq, id: `demo-${Date.now()}`, start: performance.now(), source: src, sorted: buf, total: n, timer: 0 };
      device.state = 1;
      scan.timer = window.setInterval(() => {
        const p = progressOf();
        if (!p || !scan) return;
        device.state = p.phase === "scanning" ? 3 : p.phase === "preparing" ? 1 : 4;
        device.platform = p.platform;
        emit("device", { ...device });
        emit("scan", p);
        if (p.phase === "finishing" && performance.now() - scan.start > (2.6 + Math.min(settings.scan.duration, 25) + 1.2) * 1000) {
          window.clearInterval(scan.timer);
          const meta: ScanMeta = {
            ...library.index.scans.find((s) => s.id === src)!,
            id: scan.id,
            created: new Date().toISOString(),
            name: null,
          };
          library.index.scans.push(meta);
          const id = scan.id;
          scan = null;
          device.state = 0;
          emit("device", { ...device });
          emit("library", structuredClone(library));
          emit("scan-finished", { id, complete: true, points: meta.points });
        }
      }, 100);
      return progressOf()!;
    },
    async scanStop() {
      if (scan) {
        window.clearInterval(scan.timer);
        const id = scan.id;
        scan = null;
        emit("scan-cancelled", id);
      }
    },
    async livePoints(s, since) {
      const p = progressOf();
      const head = new ArrayBuffer(20);
      const dv = new DataView(head);
      dv.setUint32(0, LIVE_MAGIC, true);
      dv.setUint32(4, s, true);
      if (!scan || !p || scan.seq !== s || !scan.sorted) return head;
      const total = Math.max(since, p.points);
      const n = total - since;
      dv.setUint32(8, total, true);
      dv.setUint32(12, since, true);
      dv.setUint32(16, n, true);
      const out = new ArrayBuffer(20 + n * 16);
      new Uint8Array(out).set(new Uint8Array(head), 0);
      const N = scan.total;
      new Uint8Array(out, 20, n * 12).set(new Uint8Array(scan.sorted, since * 12, n * 12));
      new Uint8Array(out, 20 + n * 12, n * 2).set(new Uint8Array(scan.sorted, N * 12 + since * 2, n * 2));
      new Uint8Array(out, 20 + n * 14, n * 2).set(new Uint8Array(scan.sorted, N * 14 + since * 2, n * 2));
      return out;
    },
    async scanCloud(id) {
      const meta = library.index.scans.find((s) => s.id === id);
      const src = manifest.scans.find((s) => s.id === id) ? id : manifest.scans[0]?.id;
      if (!meta || !src) throw "unknown scan";
      const buf = await (await fetch(`/dev-assets/${src}.xsc`)).arrayBuffer();
      if (new DataView(buf).getUint32(0, true) !== CLOUD_MAGIC) throw "bad asset";
      return buf;
    },
    async scanRename(id, name) {
      const s = library.index.scans.find((x) => x.id === id);
      if (s) s.name = name;
      const g = library.index.groups.find((x) => x.id === id);
      if (g) g.name = name;
      emit("library", structuredClone(library));
    },
    async scansDelete(ids) {
      const drop = new Set(ids);
      for (const g of library.index.groups) if (drop.has(g.id)) for (const s of library.index.scans) if (s.group === g.id) drop.add(s.id);
      library.index.scans = library.index.scans.filter((s) => !drop.has(s.id));
      emit("library", structuredClone(library));
    },
    async scansUngroup(ids) {
      for (const s of library.index.scans) if (ids.includes(s.id) || (s.group && ids.includes(s.group))) s.group = null;
      library.index.groups = library.index.groups.filter((g) => library.index.scans.filter((s) => s.group === g.id).length >= 2);
      emit("library", structuredClone(library));
    },
    async thumbnailSave(id, png) {
      try {
        sessionStorage.setItem(`thumb-${id}`, btoa(String.fromCharCode(...png)));
      } catch {
        /* quota */
      }
    },
    async thumbnail(id) {
      try {
        const s = sessionStorage.getItem(`thumb-${id}`);
        if (!s) return new ArrayBuffer(0);
        const bin = atob(s);
        const out = new Uint8Array(bin.length);
        for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
        return out.buffer;
      } catch {
        return new ArrayBuffer(0);
      }
    },
    async importFiles(paths) {
      return { added: [], failed: paths };
    },
    async exportCloud() {
      await new Promise((r) => setTimeout(r, 600));
      return 1_000_000;
    },
    async exportRaw() {},
    async exportMesh() {},
    async mergeStart(ids) {
      const scans = library.index.scans.filter((s) => ids.includes(s.id) || (s.group && ids.includes(s.group)));
      if (scans.length < 2) throw "need-two";
      const stages = ["structure", "floorPlan", "orientation", "refine", "polish"] as const;
      let i = 0;
      const t = window.setInterval(() => {
        if (i < stages.length) {
          emit("merge", { kind: "progress", unit: [scans[1].id], index: 1, total: scans.length - 1, stage: stages[i], step: 0, steps: 1 });
          i++;
          return;
        }
        window.clearInterval(t);
        const gid = `g-${Date.now()}`;
        library.index.groups.push({ id: gid, name: null, created: new Date().toISOString() });
        for (const s of scans) s.group = gid;
        emit("merge", { kind: "pending", unit: [scans[1].id], poses: [[scans[1].id, [1, 0, 0, 180, 0, 1, 0, -90, 0, 0, 1, 0, 0, 0, 0, 1]]], verdict: "check", reasons: ["someOverlap"], overlap: 0.53, rmse: 29, stability: 0.19 });
      }, 450);
    },
    async mergeDecide(accept) {
      if (!accept) for (const g of library.index.groups.splice(-1)) for (const s of library.index.scans) if (s.group === g.id) s.group = null;
      emit("library", structuredClone(library));
      emit("merge", { kind: "done", group: library.index.groups.at(-1)?.id ?? null, merged: accept ? 1 : 0 });
    },
    async mergeCancel() {
      emit("merge", { kind: "done", group: null, merged: 0 });
    },
    async meshStart() {
      const t0 = performance.now();
      const stages = ["thinning", "normals", "solving", "cleaning"] as const;
      stages.forEach((s, i) => setTimeout(() => emit("mesh", { kind: "progress", stage: s }), i * 500));
      setTimeout(
        () =>
          emit("mesh", {
            kind: "done",
            key: "demo",
            info: { input: 1369506, used: 541892, voxel: 4.1, spacing: 3.5, depth: 10, cellMm: 5.5, verts: 676544, tris: 1277877, seconds: (performance.now() - t0) / 1000 },
          }),
        2200,
      );
      return "demo";
    },
    async meshCancel() {},
    async meshState() {
      return [{ key: null, info: null, busy: false }, "demo"];
    },
    async meshData() {
      return (await fetch("/dev-assets/mesh.xsm")).arrayBuffer();
    },
    async calibrateStart() {
      const stages = ["microstep", "range", "planes", "rollSpacing", "tiltSearch", "allThree", "rangeRefit"] as const;
      stages.forEach((s, i) => setTimeout(() => emit("calib", { kind: "progress", stage: s, fraction: (i + 1) / 8 }), i * 400));
      setTimeout(() => emit("calib", {
        kind: "done", scan: "x",
        result: { rotation: 161.96, spacing: -38.69, tilt: -1.22, rangeError: settings.mount.rangeError, rangeErrorBefore: settings.mount.rangeError, rangeAt3mBefore: 7.81, rangeAt3mAfter: 7.76, before: { planes: 2.01, up: 8.56, horizon: 1.66, down: 0.49 }, after: { planes: 1.79, up: 1.87, horizon: 1.67, down: 0.75 }, stride: 2 },
      }), 3200);
    },
    async calibrateCancel() {},
    async calibrateApply(fit) {
      return b.settingsUpdate({ mount: { lidarRotation: fit.rotation, emitterSpacing: fit.spacing, scanTilt: fit.tilt } });
    },
    async on(event, fn) {
      if (!handlers.has(event)) handlers.set(event, new Set());
      handlers.get(event)!.add(fn as Handler);
      return () => handlers.get(event)?.delete(fn as Handler);
    },
    async pickFiles() {
      return [];
    },
    async pickFolder() {
      return null;
    },
    async saveAs(name) {
      return `C:\\Users\\demo\\Documents\\${name}`;
    },
    async reveal() {},
    async openFolder() {},
    async onDrop() {
      return () => {};
    },
    async windowShow() {},
    async windowMinimize() {},
    async windowToggleMaximize() {},
    async windowClose() {},
    async windowIsMaximized() {
      return false;
    },
    async onWindowResize(fn) {
      window.addEventListener("resize", fn);
      return () => window.removeEventListener("resize", fn);
    },
    async onCloseRequested() {
      return () => {};
    },
    async setBackground() {},
  };
  return b;
}
