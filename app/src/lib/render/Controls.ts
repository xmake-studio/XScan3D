// Mouse, touchpad and keyboard for the viewport.
//
// Keys are read by physical position (KeyboardEvent.code), so WASD works in
// any keyboard layout -- a Russian layout included -- exactly like the old
// viewer's scan-code handling.

import type { Renderer } from "./Renderer";
import type { Vec3 } from "./math";

const MOVES: Record<string, Vec3> = {
  KeyW: [1, 0, 0], ArrowUp: [1, 0, 0],
  KeyS: [-1, 0, 0], ArrowDown: [-1, 0, 0],
  KeyD: [0, 1, 0], ArrowRight: [0, 1, 0],
  KeyA: [0, -1, 0], ArrowLeft: [0, -1, 0],
  KeyE: [0, 0, 1], Space: [0, 0, 1],
  KeyQ: [0, 0, -1], ControlLeft: [0, 0, -1], ControlRight: [0, 0, -1],
};

export interface ControlEvents {
  onSpeed?: (mmPerSec: number) => void;
  onUserMove?: () => void;
}

export class Controls {
  private held = new Set<string>();
  private drag: { x: number; y: number; button: number; shift: boolean; alt: boolean; moved: boolean } | null = null;
  private hover = false;
  private disposers: (() => void)[] = [];

  constructor(private r: Renderer, private ev: ControlEvents = {}) {
    const c = r.canvas;
    const on = <K extends keyof HTMLElementEventMap>(el: HTMLElement | Window, name: K, fn: (e: HTMLElementEventMap[K]) => void, opts?: AddEventListenerOptions) => {
      el.addEventListener(name, fn as EventListener, opts);
      this.disposers.push(() => el.removeEventListener(name, fn as EventListener, opts));
    };
    on(c, "pointerdown", (e) => this.down(e));
    on(c, "pointermove", (e) => this.move(e));
    on(c, "pointerup", (e) => this.up(e));
    on(c, "pointercancel", (e) => this.up(e));
    on(c, "pointerenter", () => (this.hover = true));
    on(c, "pointerleave", () => (this.hover = false));
    on(c, "wheel", (e) => this.wheel(e), { passive: false });
    on(c, "dblclick", (e) => this.dbl(e));
    on(c, "contextmenu", (e) => e.preventDefault());
    on(window, "keydown", (e) => this.key(e, true));
    on(window, "keyup", (e) => this.key(e, false));
    on(window, "blur", () => this.held.clear());
    r.setFlyInput(() => this.flyVector());
  }

  private flyVector(): { v: Vec3; fast: boolean } {
    const v: Vec3 = [0, 0, 0];
    for (const k of this.held) {
      const m = MOVES[k];
      if (m) for (let i = 0; i < 3; i++) v[i] += m[i];
    }
    return { v, fast: this.held.has("ShiftLeft") || this.held.has("ShiftRight") };
  }

  private typing(): boolean {
    const a = document.activeElement as HTMLElement | null;
    if (!a) return false;
    return a.tagName === "INPUT" || a.tagName === "TEXTAREA" || a.isContentEditable || a.getAttribute("role") === "slider";
  }

  private key(e: KeyboardEvent, down: boolean) {
    if (this.r.camera.mode !== "fly") return;
    if (down && (this.typing() || document.querySelector("[data-modal-open]"))) return;
    const relevant = e.code in MOVES || e.code.startsWith("Shift");
    if (!relevant) return;
    if (down && !this.hover && document.activeElement !== this.r.canvas) return;
    if (down) {
      this.held.add(e.code);
      this.r.requestRender();
      this.ev.onUserMove?.();
      e.preventDefault();
    } else {
      this.held.delete(e.code);
    }
  }

  private down(e: PointerEvent) {
    this.r.canvas.focus();
    this.r.canvas.setPointerCapture(e.pointerId);
    this.drag = { x: e.clientX, y: e.clientY, button: e.button, shift: e.shiftKey, alt: e.altKey, moved: false };
    this.r.camera.stopCoast();
    this.r.setInteracting(true);
  }

  private move(e: PointerEvent) {
    const d = this.drag;
    if (!d) return;
    const dx = e.clientX - d.x, dy = e.clientY - d.y;
    if (!dx && !dy) return;
    d.x = e.clientX;
    d.y = e.clientY;
    d.moved = true;
    const cam = this.r.camera;
    if (cam.mode === "fly") {
      cam.lookBy(-dx * 0.22, dy * 0.22);
    } else {
      const pan = d.button === 1 || d.button === 2 || (d.button === 0 && (d.shift || d.alt || e.shiftKey));
      if (pan) cam.panBy(dx, dy, this.r.canvas.clientHeight);
      else cam.orbitBy(-dx * 0.32, dy * 0.32);
    }
    this.ev.onUserMove?.();
    this.r.requestRender();
  }

  private up(e: PointerEvent) {
    if (!this.drag) return;
    try {
      this.r.canvas.releasePointerCapture(e.pointerId);
    } catch {
      /* already released */
    }
    this.drag = null;
    this.r.setInteracting(false);
  }

  private wheel(e: WheelEvent) {
    e.preventDefault();
    const cam = this.r.camera;
    const unit = e.deltaMode === 1 ? 33 : e.deltaMode === 2 ? 400 : 1;
    const dx = e.deltaX * unit, dy = e.deltaY * unit;
    if (cam.mode === "fly") {
      cam.flySpeed = Math.min(50000, Math.max(50, cam.flySpeed * Math.pow(1.15, -dy / 100)));
      this.ev.onSpeed?.(cam.flySpeed);
      return;
    }
    // The wheel zooms -- that is what it is for. A touchpad's horizontal
    // swipe pans, as does Shift+wheel, and a pinch zooms more finely.
    // Orbiting stays on dragging.
    if (e.shiftKey && !e.ctrlKey) {
      cam.panBy(-(dx || dy) * 0.6, 0, this.r.canvas.clientHeight);
    } else if (Math.abs(dx) > Math.abs(dy) * 1.5) {
      cam.panBy(-dx * 0.6, 0, this.r.canvas.clientHeight);
    } else {
      cam.zoomBy(Math.pow(e.ctrlKey ? 1.006 : 1.0016, dy));
    }
    this.ev.onUserMove?.();
    this.r.setInteracting(true);
    clearTimeout((this as unknown as { wt: number }).wt);
    (this as unknown as { wt: number }).wt = window.setTimeout(() => this.r.setInteracting(false), 140);
    this.r.requestRender();
  }

  private dbl(e: MouseEvent) {
    const rect = this.r.canvas.getBoundingClientRect();
    const p = this.r.pick(e.clientX - rect.left, e.clientY - rect.top);
    const cam = this.r.camera;
    if (!p) return;
    if (cam.mode === "orbit") {
      const eye = cam.eye();
      const dist = Math.max(300, Math.hypot(eye[0] - p[0], eye[1] - p[1], eye[2] - p[2]) * 0.6);
      cam.animateTo({ target: p, distance: dist }, 650);
    } else {
      // Flying: glide most of the way towards the point.
      const eye = cam.eye();
      const to: Vec3 = [eye[0] + (p[0] - eye[0]) * 0.6, eye[1] + (p[1] - eye[1]) * 0.6, eye[2] + (p[2] - eye[2]) * 0.6];
      const f = cam.forward();
      cam.animateTo({ target: [to[0] + f[0] * 100, to[1] + f[1] * 100, to[2] + f[2] * 100] }, 650);
    }
    this.ev.onUserMove?.();
    this.r.requestRender();
  }

  dispose() {
    for (const d of this.disposers) d();
    this.r.setFlyInput(null);
  }
}
