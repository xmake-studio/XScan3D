// The 3D viewport. Renders on demand -- only while something moves, streams
// or animates -- so an idle window costs nothing and the glass panels over
// it blur a still image.

import { Camera, type CamMode } from "./Camera";
import { CloudLayer, MeshLayer, type CloudData, type MeshData } from "./layers";
import { clamp, invert, multiply, transformPoint, type Mat4, type Vec3 } from "./math";
import { rampTexels, SCAN_COLORS } from "./palette";
import * as S from "./shaders";

export type ColorMode = "height" | "distance" | "mono" | "scan";
export type Theme = "light" | "dark";

interface Program {
  prog: WebGLProgram;
  u: Record<string, WebGLUniformLocation | null>;
}

interface Target {
  fbo: WebGLFramebuffer;
  color: WebGLTexture;
  depth: WebGLTexture;
  zbuf: WebGLRenderbuffer;
  w: number;
  h: number;
}

export interface ScanPlane {
  origin: Vec3;
  angleDeg: number;
  visible: boolean;
}

const THEMES = {
  dark: {
    top: [0.105, 0.108, 0.125],
    bottom: [0.045, 0.046, 0.055],
    glow: [0.035, 0.04, 0.06],
    grid: [0.62, 0.66, 0.74],
    gridOpacity: 0.32,
    mono: [0.88, 0.89, 0.92],
    clay: [0.8, 0.81, 0.84],
    outline: 0.0,
    accent: [0.039, 0.518, 1.0],
  },
  light: {
    top: [0.975, 0.977, 0.985],
    bottom: [0.88, 0.886, 0.905],
    glow: [0.02, 0.02, 0.02],
    grid: [0.32, 0.34, 0.4],
    gridOpacity: 0.3,
    mono: [0.24, 0.25, 0.28],
    clay: [0.9, 0.9, 0.91],
    outline: 0.12,
    accent: [0.0, 0.478, 1.0],
  },
} as const;

export class Renderer {
  readonly canvas: HTMLCanvasElement;
  readonly gl: WebGL2RenderingContext;
  readonly camera = new Camera();

  clouds = new Map<string, CloudLayer>();
  live: CloudLayer | null = null;
  mesh: MeshLayer | null = null;
  showMesh = false;

  colorMode: ColorMode = "height";
  pointSize = 1.0;
  edl = true;
  grid = true;
  theme: Theme = "dark";
  clipTop = Infinity;
  scanPlane: ScanPlane = { origin: [0, 0, 0], angleDeg: 0, visible: false };
  markers: Vec3[] = [];

  /** Called after each rendered frame (for overlays that follow the view). */
  onFrame: (() => void) | null = null;

  private progs!: Record<"point" | "mesh" | "grid" | "fan" | "marker" | "composite", Program>;
  private target: Target | null = null;
  private ramp!: WebGLTexture;
  private quad!: WebGLVertexArrayObject;
  private disc!: WebGLVertexArrayObject;
  private discCount = 0;
  private markerVao!: WebGLVertexArrayObject;
  private markerBuf!: WebGLBuffer;
  private dirty = true;
  private raf = 0;
  private lastT = 0;
  private interacting = false;
  private lodBudget = Infinity;
  private heightRange: [number, number] = [-1500, 1500];
  private rangeRange: [number, number] = [0, 5000];
  private floorZ = -1500;
  /** Robust height range and lowest detected ceiling of what is visible. */
  stats: { zlo: number; zhi: number; ceiling: number | null } = { zlo: -1500, zhi: 1500, ceiling: null };
  private floatOk = false;
  private dpr = 1;
  private resizeObs: ResizeObserver;
  private flyInput: (() => { v: Vec3; fast: boolean }) | null = null;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const gl = canvas.getContext("webgl2", {
      antialias: false,
      alpha: false,
      depth: false,
      stencil: false,
      premultipliedAlpha: false,
      preserveDrawingBuffer: false,
      powerPreference: "high-performance",
    });
    if (!gl) throw new Error("WebGL2 is not available");
    this.gl = gl;
    this.floatOk = !!gl.getExtension("EXT_color_buffer_float");
    this.init();
    this.resizeObs = new ResizeObserver(() => this.requestRender());
    this.resizeObs.observe(canvas);
  }

  // --- setup --------------------------------------------------------------

  private compile(vs: string, fs: string, names: string[]): Program {
    const gl = this.gl;
    const mk = (type: number, src: string) => {
      const s = gl.createShader(type)!;
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s) || "shader");
      return s;
    };
    const prog = gl.createProgram()!;
    gl.attachShader(prog, mk(gl.VERTEX_SHADER, vs));
    gl.attachShader(prog, mk(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog) || "link");
    const u: Program["u"] = {};
    for (const n of names) u[n] = gl.getUniformLocation(prog, n);
    return { prog, u };
  }

  private init() {
    const gl = this.gl;
    this.progs = {
      point: this.compile(S.POINT_VS, S.POINT_FS, [
        "uModel", "uView", "uProj", "uPointSize", "uProjScale", "uMinPx", "uMaxPx", "uColorMode",
        "uHeightRange", "uRangeRange", "uFlatColor", "uMonoColor", "uReveal", "uLiveCount",
        "uHighlight", "uHighlightColor", "uClipTop", "uRamp",
      ]),
      mesh: this.compile(S.MESH_VS, S.MESH_FS, ["uModel", "uView", "uProj", "uEye", "uColorMode", "uHeightRange", "uClay", "uClipTop", "uRamp"]),
      grid: this.compile(S.GRID_VS, S.GRID_FS, ["uView", "uProj", "uCenter", "uHalf", "uLineColor", "uOpacity"]),
      fan: this.compile(S.FAN_VS, S.FAN_FS, ["uView", "uProj", "uOrigin", "uAngle", "uRadius", "uColor", "uOpacity"]),
      marker: this.compile(S.MARKER_VS, S.MARKER_FS, ["uView", "uProj", "uSize", "uColor"]),
      composite: this.compile(S.COMPOSITE_VS, S.COMPOSITE_FS, [
        "uColor", "uDepth", "uTexel", "uEdl", "uRadius", "uBgTop", "uBgBottom", "uBgGlow", "uOutline",
      ]),
    };
    // Colour ramp.
    this.ramp = gl.createTexture()!;
    gl.bindTexture(gl.TEXTURE_2D, this.ramp);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, 256, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, rampTexels());
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    // Grid quad (-1..1).
    this.quad = gl.createVertexArray()!;
    gl.bindVertexArray(this.quad);
    const qb = gl.createBuffer()!;
    gl.bindBuffer(gl.ARRAY_BUFFER, qb);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    // Unit disc for the scan plane.
    const seg = 96;
    const disc = new Float32Array((seg + 2) * 2);
    for (let i = 0; i <= seg; i++) {
      const a = (i / seg) * Math.PI * 2;
      disc[(i + 1) * 2] = Math.cos(a);
      disc[(i + 1) * 2 + 1] = Math.sin(a);
    }
    this.disc = gl.createVertexArray()!;
    gl.bindVertexArray(this.disc);
    const db = gl.createBuffer()!;
    gl.bindBuffer(gl.ARRAY_BUFFER, db);
    gl.bufferData(gl.ARRAY_BUFFER, disc, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
    this.discCount = seg + 2;
    // Scanner markers.
    this.markerVao = gl.createVertexArray()!;
    gl.bindVertexArray(this.markerVao);
    this.markerBuf = gl.createBuffer()!;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.markerBuf);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.bindVertexArray(null);
  }

  private ensureTarget(w: number, h: number): Target {
    const gl = this.gl;
    if (this.target && this.target.w === w && this.target.h === h) return this.target;
    if (this.target) {
      gl.deleteFramebuffer(this.target.fbo);
      gl.deleteTexture(this.target.color);
      gl.deleteTexture(this.target.depth);
      gl.deleteRenderbuffer(this.target.zbuf);
    }
    const tex = (internal: number, format: number, type: number) => {
      const t = gl.createTexture()!;
      gl.bindTexture(gl.TEXTURE_2D, t);
      gl.texImage2D(gl.TEXTURE_2D, 0, internal, w, h, 0, format, type, null);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      return t;
    };
    const color = tex(gl.RGBA8, gl.RGBA, gl.UNSIGNED_BYTE);
    const depth = this.floatOk ? tex(gl.R32F, gl.RED, gl.FLOAT) : tex(gl.RGBA8, gl.RGBA, gl.UNSIGNED_BYTE);
    const zbuf = gl.createRenderbuffer()!;
    gl.bindRenderbuffer(gl.RENDERBUFFER, zbuf);
    gl.renderbufferStorage(gl.RENDERBUFFER, gl.DEPTH_COMPONENT24, w, h);
    const fbo = gl.createFramebuffer()!;
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, color, 0);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT1, gl.TEXTURE_2D, depth, 0);
    gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, zbuf);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    this.target = { fbo, color, depth, zbuf, w, h };
    return this.target;
  }

  // --- scene --------------------------------------------------------------

  setCloud(id: string, data: CloudData, pose: number[] | null, reveal: boolean): CloudLayer {
    let layer = this.clouds.get(id);
    if (!layer) {
      layer = new CloudLayer(this.gl, id);
      this.clouds.set(id, layer);
    }
    layer.setData(data);
    layer.setPose(pose);
    layer.revealStart = reveal ? performance.now() : 0;
    layer.lastUsed = performance.now();
    this.updateRanges();
    this.requestRender();
    return layer;
  }

  removeCloud(id: string) {
    const l = this.clouds.get(id);
    if (l) {
      l.dispose();
      this.clouds.delete(id);
      this.requestRender();
    }
  }

  /** Makes exactly these clouds visible, with poses and flat colours. */
  showOnly(ids: string[], poses: Record<string, number[] | null>) {
    const set = new Set(ids);
    let k = 0;
    for (const [id, l] of this.clouds) {
      l.visible = set.has(id);
      if (l.visible) {
        l.setPose(poses[id] ?? null);
        l.lastUsed = performance.now();
      }
    }
    for (const id of ids) {
      const l = this.clouds.get(id);
      if (l) l.flatColor = SCAN_COLORS[k++ % SCAN_COLORS.length];
    }
    this.updateRanges();
    this.requestRender();
  }

  /** Evicts hidden clouds, oldest first, past a total point budget. */
  trim(maxPoints = 40_000_000) {
    let total = 0;
    for (const l of this.clouds.values()) total += l.count;
    const hidden = [...this.clouds.values()].filter((l) => !l.visible).sort((a, b) => a.lastUsed - b.lastUsed);
    for (const l of hidden) {
      if (total <= maxPoints) break;
      total -= l.count;
      this.removeCloud(l.id);
    }
  }

  beginLive(): CloudLayer {
    this.endLive();
    const l = new CloudLayer(this.gl, "__live__");
    l.live = true;
    this.live = l;
    this.requestRender();
    return l;
  }

  endLive() {
    if (this.live) {
      this.live.dispose();
      this.live = null;
      this.requestRender();
    }
  }

  setMesh(key: string, m: MeshData | null) {
    if (this.mesh) this.mesh.dispose();
    this.mesh = m ? new MeshLayer(this.gl, key, m) : null;
    this.requestRender();
  }

  setMarkers(pts: Vec3[]) {
    this.markers = pts;
    const gl = this.gl;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.markerBuf);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(pts.flat()), gl.DYNAMIC_DRAW);
    this.requestRender();
  }

  visibleLayers(): CloudLayer[] {
    return [...this.clouds.values()].filter((l) => l.visible && l.header);
  }

  /** World bounds of what is on screen, or null. */
  bounds(): { min: Vec3; max: Vec3 } | null {
    const min: Vec3 = [Infinity, Infinity, Infinity];
    const max: Vec3 = [-Infinity, -Infinity, -Infinity];
    let any = false;
    for (const l of this.visibleLayers()) {
      const h = l.header!;
      for (let i = 0; i < 8; i++) {
        const c: Vec3 = [i & 1 ? h.bboxMax[0] : h.bboxMin[0], i & 2 ? h.bboxMax[1] : h.bboxMin[1], i & 4 ? h.bboxMax[2] : h.bboxMin[2]];
        const w = transformPoint(l.model, c);
        for (let k = 0; k < 3; k++) {
          min[k] = Math.min(min[k], w[k]);
          max[k] = Math.max(max[k], w[k]);
        }
        any = true;
      }
    }
    return any ? { min, max } : null;
  }

  private updateRanges() {
    const vis = this.visibleLayers();
    if (!vis.length) return;
    let zlo = Infinity, zhi = -Infinity, dlo = Infinity, dhi = -Infinity;
    let ceiling: number | null = null;
    for (const l of vis) {
      const h = l.header!;
      const dz = l.model[14];
      zlo = Math.min(zlo, h.zSpan[0] + dz);
      zhi = Math.max(zhi, h.zSpan[1] + dz);
      dlo = Math.min(dlo, h.dSpan[0]);
      dhi = Math.max(dhi, h.dSpan[1]);
      if (h.ceiling !== null) ceiling = ceiling === null ? h.ceiling + dz : Math.min(ceiling, h.ceiling + dz);
    }
    this.stats = { zlo, zhi, ceiling };
    this.heightRange = [zlo, Math.max(zhi, zlo + 100)];
    this.rangeRange = [dlo, Math.max(dhi, dlo + 100)];
    this.floorZ = zlo;
    const b = this.bounds();
    if (b) this.camera.sceneRadius = Math.max(1000, 0.5 * Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]));
  }

  /** Frames everything visible (or the live cloud), animated. */
  frame(duration = 800, keepAngles = false) {
    let b = this.bounds();
    if (!b && this.live && this.live.count > 0) b = { min: [-3000, -3000, -1500], max: [3000, 3000, 1500] };
    if (!b) return;
    const c: Vec3 = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
    const ext = Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]);
    const dist = clamp((ext * 0.5) / Math.tan(this.camera.fovY / 2) * 0.9, 600, 200000);
    this.camera.animateTo({ target: c, distance: dist, ...(keepAngles ? {} : { azimuth: -55, elevation: 30 }) }, duration);
    this.requestRender();
  }

  setCamMode(mode: CamMode) {
    this.camera.setMode(mode);
    this.requestRender();
  }

  setFlyInput(fn: (() => { v: Vec3; fast: boolean }) | null) {
    this.flyInput = fn;
  }

  setInteracting(on: boolean) {
    this.interacting = on;
    if (!on) {
      this.lodBudget = Infinity;
      this.requestRender();
    }
  }

  requestRender() {
    this.dirty = true;
    if (!this.raf) this.raf = requestAnimationFrame((t) => this.tick(t));
  }

  // --- frame --------------------------------------------------------------

  private tick(t: number) {
    this.raf = 0;
    const dt = this.lastT ? Math.min(0.05, (t - this.lastT) / 1000) : 0.016;
    this.lastT = t;
    let again = false;
    // Uploads stream a chunk per frame.
    for (const l of this.clouds.values()) if (l.uploading && l.pump()) again = true;
    if (this.camera.coast(dt, this.canvas.clientHeight || 1)) again = true;
    if (this.camera.mode === "fly" && this.flyInput) {
      const { v, fast } = this.flyInput();
      if (this.camera.fly(v, fast, dt)) again = true;
    }
    const now = performance.now();
    for (const l of this.clouds.values()) {
      if (l.visible && l.revealStart && now - l.revealStart < l.revealMs + 400) again = true;
    }
    if (this.live && this.live.count > 0) again = true; // live glow fades
    if (this.scanPlane.visible) again = true;
    if (this.dirty || again) {
      this.dirty = false;
      const t0 = performance.now();
      this.render();
      const ms = performance.now() - t0;
      // Thin while the view moves if frames get slow; draw everything at rest.
      const moving = this.interacting || this.camera.animating || again;
      if (moving) {
        if (ms > 22) this.lodBudget = Math.max(400_000, Math.floor((this.lodBudget === Infinity ? this.totalVisible() : this.lodBudget) * 0.7));
        else if (ms < 10 && this.lodBudget !== Infinity) this.lodBudget = Math.floor(this.lodBudget * 1.15);
      }
      this.onFrame?.();
    }
    if (again || this.interacting) {
      this.raf = requestAnimationFrame((tt) => this.tick(tt));
    } else if (this.lodBudget !== Infinity) {
      // Came to rest: one full-detail frame.
      this.lodBudget = Infinity;
      this.requestRender();
    }
  }

  private totalVisible(): number {
    let n = 0;
    for (const l of this.visibleLayers()) n += l.uploaded;
    return n;
  }

  private resize(): [number, number] {
    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(this.canvas.clientWidth * this.dpr));
    const h = Math.max(1, Math.round(this.canvas.clientHeight * this.dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    return [w, h];
  }

  render(targetOverride?: { w: number; h: number; fbo: WebGLFramebuffer | null }) {
    const gl = this.gl;
    const [cw, ch] = this.resize();
    const w = targetOverride?.w ?? cw;
    const h = targetOverride?.h ?? ch;
    const tg = this.ensureTarget(w, h);
    const th = THEMES[this.theme];
    const view = this.camera.view();
    const proj = this.camera.proj(w / h);
    const eye = this.camera.eye();

    gl.bindFramebuffer(gl.FRAMEBUFFER, tg.fbo);
    gl.viewport(0, 0, w, h);
    gl.drawBuffers([gl.COLOR_ATTACHMENT0, gl.COLOR_ATTACHMENT1]);
    gl.clearBufferfv(gl.COLOR, 0, [0, 0, 0, 0]);
    gl.clearBufferfv(gl.COLOR, 1, [0, 0, 0, 0]);
    gl.clearBufferfi(gl.DEPTH_STENCIL, 0, 1, 0);
    gl.enable(gl.DEPTH_TEST);
    gl.depthFunc(gl.LESS);
    gl.depthMask(true);
    gl.disable(gl.BLEND);

    // Surface.
    if (this.showMesh && this.mesh) {
      const p = this.progs.mesh;
      gl.useProgram(p.prog);
      gl.uniformMatrix4fv(p.u.uModel, false, new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]));
      gl.uniformMatrix4fv(p.u.uView, false, view);
      gl.uniformMatrix4fv(p.u.uProj, false, proj);
      gl.uniform3fv(p.u.uEye, eye);
      gl.uniform1i(p.u.uColorMode, this.colorMode === "height" ? 1 : 0);
      gl.uniform2fv(p.u.uHeightRange, this.heightRange);
      gl.uniform3fv(p.u.uClay, th.clay as unknown as number[]);
      gl.uniform1f(p.u.uClipTop, this.clipTop);
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, this.ramp);
      gl.uniform1i(p.u.uRamp, 0);
      this.mesh.draw();
    }

    // Points.
    const layers = this.visibleLayers();
    const drawPoints = !(this.showMesh && this.mesh);
    if (drawPoints || this.live) {
      const p = this.progs.point;
      gl.useProgram(p.prog);
      gl.uniformMatrix4fv(p.u.uView, false, view);
      gl.uniformMatrix4fv(p.u.uProj, false, proj);
      const projScale = h / (2 * Math.tan(this.camera.fovY / 2));
      gl.uniform1f(p.u.uProjScale, projScale);
      gl.uniform1f(p.u.uPointSize, 9 * this.pointSize);
      gl.uniform1f(p.u.uMinPx, 1.4 * this.dpr * Math.sqrt(this.pointSize));
      gl.uniform1f(p.u.uMaxPx, 14 * this.dpr);
      gl.uniform2fv(p.u.uHeightRange, this.heightRange);
      gl.uniform2fv(p.u.uRangeRange, this.rangeRange);
      gl.uniform3fv(p.u.uMonoColor, th.mono as unknown as number[]);
      gl.uniform3fv(p.u.uHighlightColor, [1.0, 0.58, 0.1]);
      gl.uniform1f(p.u.uClipTop, this.clipTop);
      gl.activeTexture(gl.TEXTURE0);
      gl.bindTexture(gl.TEXTURE_2D, this.ramp);
      gl.uniform1i(p.u.uRamp, 0);
      const modeIndex = { height: 0, distance: 1, mono: 2, scan: 3 }[this.colorMode];
      const total = layers.reduce((a, l) => a + l.uploaded, 0) || 1;
      const now = performance.now();
      if (drawPoints) {
        for (const l of layers) {
          gl.uniformMatrix4fv(p.u.uModel, false, l.model);
          let mode = modeIndex;
          if (l.hasRgb() && (this.colorMode === "height" || this.colorMode === "distance")) mode = 4;
          gl.uniform1i(p.u.uColorMode, mode);
          gl.uniform3fv(p.u.uFlatColor, l.flatColor);
          const reveal = l.revealStart ? clamp((now - l.revealStart) / l.revealMs, 0, 1) * 1.08 : 2.0;
          gl.uniform1f(p.u.uReveal, reveal);
          gl.uniform1f(p.u.uLiveCount, 0);
          gl.uniform1f(p.u.uHighlight, l.highlight);
          const n = this.lodBudget === Infinity ? l.uploaded : Math.ceil((this.lodBudget * l.uploaded) / total);
          l.draw(n);
        }
      }
      if (this.live && this.live.count > 0) {
        const l = this.live;
        gl.uniformMatrix4fv(p.u.uModel, false, l.model);
        gl.uniform1i(p.u.uColorMode, modeIndex === 3 ? 0 : modeIndex);
        gl.uniform1f(p.u.uReveal, 2.0);
        gl.uniform1f(p.u.uLiveCount, l.count);
        gl.uniform1f(p.u.uHighlight, 0);
        l.draw(l.count);
      }
    }

    // Grid (colour only, behind whatever is drawn).
    gl.drawBuffers([gl.COLOR_ATTACHMENT0, gl.NONE]);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    gl.depthMask(false);
    if (this.grid) {
      const p = this.progs.grid;
      gl.useProgram(p.prog);
      gl.uniformMatrix4fv(p.u.uView, false, view);
      gl.uniformMatrix4fv(p.u.uProj, false, proj);
      const half = Math.max(6000, this.camera.sceneRadius * 2.2);
      const tgt = this.camera.target;
      gl.uniform3fv(p.u.uCenter, [Math.round(tgt[0] / 1000) * 1000, Math.round(tgt[1] / 1000) * 1000, this.floorZ]);
      gl.uniform1f(p.u.uHalf, half);
      gl.uniform3fv(p.u.uLineColor, th.grid as unknown as number[]);
      gl.uniform1f(p.u.uOpacity, th.gridOpacity);
      gl.bindVertexArray(this.quad);
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
    }
    // Scanner markers.
    if (this.markers.length) {
      const p = this.progs.marker;
      gl.useProgram(p.prog);
      gl.uniformMatrix4fv(p.u.uView, false, view);
      gl.uniformMatrix4fv(p.u.uProj, false, proj);
      gl.uniform1f(p.u.uSize, 18 * this.dpr);
      gl.uniform3fv(p.u.uColor, th.accent as unknown as number[]);
      gl.disable(gl.DEPTH_TEST);
      gl.bindVertexArray(this.markerVao);
      gl.drawArrays(gl.POINTS, 0, this.markers.length);
      gl.enable(gl.DEPTH_TEST);
    }
    // Scan plane.
    if (this.scanPlane.visible) {
      const p = this.progs.fan;
      gl.useProgram(p.prog);
      gl.uniformMatrix4fv(p.u.uView, false, view);
      gl.uniformMatrix4fv(p.u.uProj, false, proj);
      gl.uniform3fv(p.u.uOrigin, this.scanPlane.origin);
      gl.uniform1f(p.u.uAngle, (this.scanPlane.angleDeg * Math.PI) / 180);
      gl.uniform1f(p.u.uRadius, 1400);
      gl.uniform3fv(p.u.uColor, th.accent as unknown as number[]);
      const pulse = 0.85 + 0.15 * Math.sin(performance.now() / 260);
      gl.uniform1f(p.u.uOpacity, pulse);
      gl.bindVertexArray(this.disc);
      gl.drawArrays(gl.TRIANGLE_FAN, 0, this.discCount);
    }
    gl.bindVertexArray(null);
    gl.depthMask(true);
    gl.disable(gl.BLEND);

    // Composite onto the screen (or the override target).
    gl.bindFramebuffer(gl.FRAMEBUFFER, targetOverride ? targetOverride.fbo : null);
    gl.viewport(0, 0, w, h);
    gl.disable(gl.DEPTH_TEST);
    const c = this.progs.composite;
    gl.useProgram(c.prog);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, tg.color);
    gl.uniform1i(c.u.uColor, 0);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, tg.depth);
    gl.uniform1i(c.u.uDepth, 1);
    gl.uniform2f(c.u.uTexel, 1 / w, 1 / h);
    gl.uniform1f(c.u.uEdl, this.edl && this.floatOk ? 1.0 : 0.0);
    gl.uniform1f(c.u.uRadius, 1.4 * this.dpr);
    gl.uniform3fv(c.u.uBgTop, th.top as unknown as number[]);
    gl.uniform3fv(c.u.uBgBottom, th.bottom as unknown as number[]);
    gl.uniform3fv(c.u.uBgGlow, th.glow as unknown as number[]);
    gl.uniform1f(c.u.uOutline, this.edl ? th.outline : 0);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.enable(gl.DEPTH_TEST);
  }

  // --- picking, projection, thumbnails -----------------------------------

  /** World point under a CSS-pixel position, or null over background. */
  pick(x: number, y: number): Vec3 | null {
    if (!this.target || !this.floatOk) return null;
    const gl = this.gl;
    const px = Math.round(x * this.dpr);
    const py = Math.round(this.target.h - y * this.dpr);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.target.fbo);
    gl.readBuffer(gl.COLOR_ATTACHMENT1);
    let best = 0;
    const buf = new Float32Array(4 * 25);
    gl.readPixels(Math.max(0, px - 2), Math.max(0, py - 2), 5, 5, gl.RGBA, gl.FLOAT, buf);
    for (let i = 0; i < 25; i++) {
      const d = Math.abs(buf[i * 4]);
      if (d > 0 && (best === 0 || d < best)) best = d;
    }
    gl.readBuffer(gl.COLOR_ATTACHMENT0);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    if (!best) return null;
    const w = this.target.w, h = this.target.h;
    const ndcX = (px / w) * 2 - 1, ndcY = (py / h) * 2 - 1;
    const proj = this.camera.proj(w / h);
    const view = this.camera.view();
    const inv = invert(multiply(proj, view));
    if (!inv) return null;
    const eye = this.camera.eye();
    const far = transformPoint(inv, [ndcX, ndcY, 1]);
    const dir: Vec3 = [far[0] - eye[0], far[1] - eye[1], far[2] - eye[2]];
    const l = Math.hypot(dir[0], dir[1], dir[2]) || 1;
    // The stored depth is along the view axis; convert to distance along the ray.
    const f = this.camera.forward();
    const cos = (dir[0] * f[0] + dir[1] * f[1] + dir[2] * f[2]) / l;
    const t = best / Math.max(cos, 1e-3);
    return [eye[0] + (dir[0] / l) * t, eye[1] + (dir[1] / l) * t, eye[2] + (dir[2] / l) * t];
  }

  /** Screen position (CSS px) of a world point, or null behind the camera. */
  project(p: Vec3): [number, number] | null {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    const m = multiply(this.camera.proj(w / h), this.camera.view());
    const x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12];
    const y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13];
    const ww = m[3] * p[0] + m[7] * p[1] + m[11] * p[2] + m[15];
    if (ww <= 0) return null;
    return [((x / ww + 1) / 2) * w, ((1 - y / ww) / 2) * h];
  }

  /** Reads a framebuffer without stalling: the pixels go through a pixel
   * buffer and a fence, polled on later ticks. A plain readPixels waits for
   * the GPU to drain, which on a big cloud is hundreds of milliseconds of
   * frozen window. */
  private readAsync(fbo: WebGLFramebuffer, w: number, h: number): Promise<Uint8Array> {
    const gl = this.gl;
    const size = w * h * 4;
    const pbo = gl.createBuffer()!;
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, pbo);
    gl.bufferData(gl.PIXEL_PACK_BUFFER, size, gl.STREAM_READ);
    gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, 0);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0)!;
    gl.flush();
    return new Promise((resolve, reject) => {
      const poll = () => {
        const st = gl.clientWaitSync(sync, 0, 0);
        if (st === gl.WAIT_FAILED) {
          gl.deleteSync(sync);
          gl.deleteBuffer(pbo);
          reject(new Error("readback failed"));
          return;
        }
        if (st === gl.TIMEOUT_EXPIRED) {
          setTimeout(poll, 12);
          return;
        }
        const px = new Uint8Array(size);
        gl.bindBuffer(gl.PIXEL_PACK_BUFFER, pbo);
        gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, px);
        gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
        gl.deleteSync(sync);
        gl.deleteBuffer(pbo);
        resolve(px);
      };
      poll();
    });
  }

  /** Renders a small framed view of the visible clouds to PNG bytes. */
  async thumbnail(width = 288, height = 200): Promise<Uint8Array | null> {
    const b = this.bounds();
    if (!b) return null;
    const gl = this.gl;
    const cam = this.camera;
    const saved = { target: [...cam.target] as Vec3, distance: cam.distance, azimuth: cam.azimuth, elevation: cam.elevation, mode: cam.mode };
    const savedReveal = [...this.clouds.values()].map((l) => [l, l.revealStart] as const);
    for (const [l] of savedReveal) l.revealStart = 0;
    const savedLod = this.lodBudget;
    // A thumbnail is 288 px wide: a fraction of the points is plenty, and it
    // keeps the frame it costs short.
    this.lodBudget = 500_000;
    if (cam.mode === "fly") cam.setMode("orbit");
    const c: Vec3 = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
    const ext = Math.hypot(b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]);
    cam.target = c;
    cam.distance = ((ext * 0.5) / Math.tan(cam.fovY / 2)) * 0.85;
    cam.azimuth = -55;
    cam.elevation = 38;
    const tex = gl.createTexture()!;
    gl.bindTexture(gl.TEXTURE_2D, tex);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, width, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    const fbo = gl.createFramebuffer()!;
    gl.bindFramebuffer(gl.FRAMEBUFFER, fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0);
    const oldTarget = this.target;
    this.target = null;
    const gridWas = this.grid;
    this.grid = false;
    this.render({ w: width, h: height, fbo });
    // Drop the scratch target and restore the live one before the readback,
    // so the next frame can be drawn while the GPU catches up.
    if (this.target) {
      const tg = this.target as Target;
      gl.deleteFramebuffer(tg.fbo);
      gl.deleteTexture(tg.color);
      gl.deleteTexture(tg.depth);
      gl.deleteRenderbuffer(tg.zbuf);
    }
    this.target = oldTarget;
    this.grid = gridWas;
    cam.target = saved.target;
    cam.distance = saved.distance;
    cam.azimuth = saved.azimuth;
    cam.elevation = saved.elevation;
    if (saved.mode === "fly") cam.setMode("fly");
    for (const [l, r] of savedReveal) l.revealStart = r;
    this.lodBudget = savedLod;
    this.requestRender();
    let px: Uint8Array;
    try {
      px = await this.readAsync(fbo, width, height);
    } finally {
      gl.deleteFramebuffer(fbo);
      gl.deleteTexture(tex);
    }
    // Flip rows and encode.
    const cv = new OffscreenCanvas(width, height);
    const ctx = cv.getContext("2d")!;
    const img = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      img.data.set(px.subarray((height - 1 - y) * width * 4, (height - y) * width * 4), y * width * 4);
    }
    ctx.putImageData(img, 0, 0);
    const blob = await cv.convertToBlob({ type: "image/png" });
    return new Uint8Array(await blob.arrayBuffer());
  }

  dispose() {
    cancelAnimationFrame(this.raf);
    this.resizeObs.disconnect();
    for (const l of this.clouds.values()) l.dispose();
    this.clouds.clear();
    this.endLive();
    this.mesh?.dispose();
  }
}
