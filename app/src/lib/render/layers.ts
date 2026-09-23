// GPU-side geometry: point clouds (static or growing) and meshes.

import { fromRowMajor, mat4, type Mat4 } from "./math";

export const CLOUD_MAGIC = 0x31435358; // "XSC1"
export const LIVE_MAGIC = 0x314c5358; // "XSL1"
export const MESH_MAGIC = 0x314d5358; // "XSM1"

export interface CloudHeader {
  count: number;
  hasRgb: boolean;
  sweepDeg: number;
  bboxMin: [number, number, number];
  bboxMax: [number, number, number];
  zSpan: [number, number];
  dSpan: [number, number];
  /** Detected ceiling height in the scan's frame, or null. */
  ceiling: number | null;
}

export interface CloudData {
  header: CloudHeader;
  pos: Float32Array;
  range: Uint16Array;
  order: Uint16Array;
  rgb: Uint8Array | null;
}

export function parseCloud(buf: ArrayBuffer): CloudData {
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== CLOUD_MAGIC) throw new Error("bad cloud data");
  const n = dv.getUint32(4, true);
  const flags = dv.getUint32(8, true);
  const f = (o: number) => dv.getFloat32(o, true);
  const header: CloudHeader = {
    count: n,
    hasRgb: (flags & 1) !== 0,
    sweepDeg: f(12),
    bboxMin: [f(16), f(20), f(24)],
    bboxMax: [f(28), f(32), f(36)],
    zSpan: [f(40), f(44)],
    dSpan: [f(48), f(52)],
    ceiling: Number.isFinite(f(56)) ? f(56) : null,
  };
  const pos = new Float32Array(buf, 64, n * 3);
  const range = new Uint16Array(buf, 64 + n * 12, n);
  const order = new Uint16Array(buf, 64 + n * 14, n);
  const rgb = header.hasRgb ? new Uint8Array(buf, 64 + n * 16, n * 3) : null;
  return { header, pos, range, order, rgb };
}

/** Points uploaded per frame while a big cloud streams onto the GPU. */
const UPLOAD_CHUNK = 500_000;

export class CloudLayer {
  readonly id: string;
  model: Mat4 = mat4();
  poseRow: number[] | null = null;
  flatColor: [number, number, number] = [0.5, 0.5, 0.5];
  highlight = 0;
  /** performance.now() when the sweep reveal started, or 0 when shown. */
  revealStart = 0;
  revealMs = 1100;
  /** Live clouds: points glow as they arrive. */
  live = false;
  visible = true;

  count = 0;
  uploaded = 0;
  capacity = 0;
  header: CloudHeader | null = null;
  lastUsed = performance.now();

  private gl: WebGL2RenderingContext;
  private vao: WebGLVertexArrayObject;
  private bPos: WebGLBuffer;
  private bRange: WebGLBuffer;
  private bOrder: WebGLBuffer;
  private bRgb: WebGLBuffer | null = null;
  private pending: CloudData | null = null;

  constructor(gl: WebGL2RenderingContext, id: string) {
    this.gl = gl;
    this.id = id;
    this.vao = gl.createVertexArray()!;
    this.bPos = gl.createBuffer()!;
    this.bRange = gl.createBuffer()!;
    this.bOrder = gl.createBuffer()!;
    this.bind();
  }

  private bind() {
    const gl = this.gl;
    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bPos);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bRange);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 1, gl.UNSIGNED_SHORT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bOrder);
    gl.enableVertexAttribArray(2);
    gl.vertexAttribPointer(2, 1, gl.UNSIGNED_SHORT, false, 0, 0);
    if (this.bRgb) {
      gl.bindBuffer(gl.ARRAY_BUFFER, this.bRgb);
      gl.enableVertexAttribArray(3);
      gl.vertexAttribPointer(3, 3, gl.UNSIGNED_BYTE, true, 0, 0);
    } else {
      gl.disableVertexAttribArray(3);
      gl.vertexAttrib3f(3, 0.8, 0.8, 0.8);
    }
    gl.bindVertexArray(null);
  }

  setPose(row: number[] | null) {
    this.poseRow = row;
    this.model = row ? fromRowMajor(row) : mat4();
  }

  /** Replaces the contents; the upload then streams over a few frames. */
  setData(d: CloudData) {
    const gl = this.gl;
    const n = d.header.count;
    this.header = d.header;
    this.count = n;
    this.uploaded = 0;
    this.capacity = n;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bPos);
    gl.bufferData(gl.ARRAY_BUFFER, n * 12, gl.STATIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bRange);
    gl.bufferData(gl.ARRAY_BUFFER, n * 2, gl.STATIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bOrder);
    gl.bufferData(gl.ARRAY_BUFFER, n * 2, gl.STATIC_DRAW);
    if (d.rgb) {
      if (!this.bRgb) this.bRgb = gl.createBuffer()!;
      gl.bindBuffer(gl.ARRAY_BUFFER, this.bRgb);
      gl.bufferData(gl.ARRAY_BUFFER, n * 3, gl.STATIC_DRAW);
    } else if (this.bRgb) {
      gl.deleteBuffer(this.bRgb);
      this.bRgb = null;
    }
    this.bind();
    this.pending = d;
    this.pump();
  }

  /** Uploads the next chunk. Returns true while more remains. */
  pump(): boolean {
    const d = this.pending;
    if (!d) return false;
    const gl = this.gl;
    const a = this.uploaded;
    const b = Math.min(d.header.count, a + UPLOAD_CHUNK);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bPos);
    gl.bufferSubData(gl.ARRAY_BUFFER, a * 12, d.pos, a * 3, (b - a) * 3);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bRange);
    gl.bufferSubData(gl.ARRAY_BUFFER, a * 2, d.range, a, b - a);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bOrder);
    gl.bufferSubData(gl.ARRAY_BUFFER, a * 2, d.order, a, b - a);
    if (d.rgb && this.bRgb) {
      gl.bindBuffer(gl.ARRAY_BUFFER, this.bRgb);
      gl.bufferSubData(gl.ARRAY_BUFFER, a * 3, d.rgb, a * 3, (b - a) * 3);
    }
    this.uploaded = b;
    if (b >= d.header.count) {
      this.pending = null;
      return false;
    }
    return true;
  }

  get uploading(): boolean {
    return this.pending !== null;
  }

  /** Appends live points (positions, ranges, order), growing the buffers. */
  append(pos: Float32Array, range: Uint16Array, order: Uint16Array) {
    const gl = this.gl;
    const add = range.length;
    if (!add) return;
    const need = this.count + add;
    if (need > this.capacity) {
      const cap = Math.max(need, Math.ceil(this.capacity * 1.8), 65536);
      const grow = (buf: WebGLBuffer, bytesPer: number) => {
        const nb = gl.createBuffer()!;
        gl.bindBuffer(gl.COPY_WRITE_BUFFER, nb);
        gl.bufferData(gl.COPY_WRITE_BUFFER, cap * bytesPer, gl.DYNAMIC_DRAW);
        if (this.count > 0) {
          gl.bindBuffer(gl.COPY_READ_BUFFER, buf);
          gl.copyBufferSubData(gl.COPY_READ_BUFFER, gl.COPY_WRITE_BUFFER, 0, 0, this.count * bytesPer);
        }
        gl.deleteBuffer(buf);
        return nb;
      };
      this.bPos = grow(this.bPos, 12);
      this.bRange = grow(this.bRange, 2);
      this.bOrder = grow(this.bOrder, 2);
      this.capacity = cap;
      this.bind();
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bPos);
    gl.bufferSubData(gl.ARRAY_BUFFER, this.count * 12, pos);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bRange);
    gl.bufferSubData(gl.ARRAY_BUFFER, this.count * 2, range);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.bOrder);
    gl.bufferSubData(gl.ARRAY_BUFFER, this.count * 2, order);
    this.count = need;
    this.uploaded = need;
  }

  draw(count: number) {
    const gl = this.gl;
    gl.bindVertexArray(this.vao);
    gl.drawArrays(gl.POINTS, 0, Math.min(count, this.uploaded));
    gl.bindVertexArray(null);
  }

  hasRgb(): boolean {
    return this.bRgb !== null;
  }

  dispose() {
    const gl = this.gl;
    gl.deleteBuffer(this.bPos);
    gl.deleteBuffer(this.bRange);
    gl.deleteBuffer(this.bOrder);
    if (this.bRgb) gl.deleteBuffer(this.bRgb);
    gl.deleteVertexArray(this.vao);
    this.pending = null;
  }
}

export interface MeshData {
  verts: number;
  tris: number;
  pos: Float32Array;
  normals: Int8Array;
  index: Uint32Array;
}

export function parseMesh(buf: ArrayBuffer): MeshData {
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== MESH_MAGIC) throw new Error("bad mesh data");
  const nv = dv.getUint32(4, true);
  const nt = dv.getUint32(8, true);
  const pos = new Float32Array(buf, 16, nv * 3);
  const normals = new Int8Array(buf, 16 + nv * 12, nv * 4);
  const index = new Uint32Array(buf, 16 + nv * 16, nt * 3);
  return { verts: nv, tris: nt, pos, normals, index };
}

export class MeshLayer {
  private gl: WebGL2RenderingContext;
  private vao: WebGLVertexArrayObject;
  private bufs: WebGLBuffer[] = [];
  count = 0;
  readonly key: string;

  constructor(gl: WebGL2RenderingContext, key: string, m: MeshData) {
    this.gl = gl;
    this.key = key;
    this.vao = gl.createVertexArray()!;
    gl.bindVertexArray(this.vao);
    const pb = gl.createBuffer()!;
    gl.bindBuffer(gl.ARRAY_BUFFER, pb);
    gl.bufferData(gl.ARRAY_BUFFER, m.pos, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    const nb = gl.createBuffer()!;
    gl.bindBuffer(gl.ARRAY_BUFFER, nb);
    gl.bufferData(gl.ARRAY_BUFFER, m.normals, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.BYTE, true, 4, 0);
    const ib = gl.createBuffer()!;
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, m.index, gl.STATIC_DRAW);
    gl.bindVertexArray(null);
    this.bufs = [pb, nb, ib];
    this.count = m.tris * 3;
  }

  draw() {
    const gl = this.gl;
    gl.bindVertexArray(this.vao);
    gl.drawElements(gl.TRIANGLES, this.count, gl.UNSIGNED_INT, 0);
    gl.bindVertexArray(null);
  }

  dispose() {
    for (const b of this.bufs) this.gl.deleteBuffer(b);
    this.gl.deleteVertexArray(this.vao);
  }
}
