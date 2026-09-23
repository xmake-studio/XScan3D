// One camera, two ways of driving it -- the same two the old viewer had:
//
//   orbit  drag swings around a pivot, wheel zooms, right/middle/shift-drag
//          slides the pivot, touchpad scroll orbits and pinch zooms;
//   fly    drag looks around, WASD / arrows walk, Space/E up, Q/Ctrl down,
//          Shift faster, wheel sets the speed.
//
// Both share one pose representation (pivot, distance, azimuth, elevation),
// so switching modes never jumps the view: flying just keeps the pivot a
// short step in front of the eye.

import { clamp, easeInOutCubic, lookAt, perspective, type Mat4, type Vec3 } from "./math";

export type CamMode = "orbit" | "fly";

const FLY_DISTANCE = 100;

interface Tween {
  from: { target: Vec3; distance: number; azimuth: number; elevation: number };
  to: { target: Vec3; distance: number; azimuth: number; elevation: number };
  start: number;
  duration: number;
}

export class Camera {
  target: Vec3 = [0, 0, 0];
  distance = 4000;
  azimuth = -55; // degrees
  elevation = 28; // degrees
  fovY = (52 * Math.PI) / 180;
  mode: CamMode = "orbit";
  flySpeed = 1500; // mm/s
  sceneRadius = 5000;

  private orbitDistance = 4000;
  private vel = { az: 0, el: 0, px: 0, py: 0 };
  private tween: Tween | null = null;
  private flyVel: Vec3 = [0, 0, 0];

  /** Direction from pivot to eye. */
  private dir(): Vec3 {
    const el = (this.elevation * Math.PI) / 180;
    const az = (this.azimuth * Math.PI) / 180;
    return [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
  }

  eye(): Vec3 {
    const d = this.dir();
    return [this.target[0] + d[0] * this.distance, this.target[1] + d[1] * this.distance, this.target[2] + d[2] * this.distance];
  }

  forward(): Vec3 {
    const d = this.dir();
    return [-d[0], -d[1], -d[2]];
  }

  view(): Mat4 {
    return lookAt(this.eye(), this.target, [0, 0, 1]);
  }

  proj(aspect: number): Mat4 {
    const far = Math.max(this.sceneRadius * 8, this.distance * 4, 50000);
    const near = this.mode === "fly" ? 10 : clamp(this.distance * 0.002, 5, 200);
    return perspective(this.fovY, aspect, near, far);
  }

  setMode(mode: CamMode) {
    if (mode === this.mode) return;
    const eye = this.eye();
    if (mode === "fly") {
      this.orbitDistance = this.distance;
      this.distance = FLY_DISTANCE;
    } else {
      this.distance = this.orbitDistance || 4000;
    }
    this.mode = mode;
    this.placeEye(eye);
    this.vel = { az: 0, el: 0, px: 0, py: 0 };
    this.flyVel = [0, 0, 0];
  }

  private placeEye(eye: Vec3) {
    const f = this.forward();
    this.target = [eye[0] + f[0] * this.distance, eye[1] + f[1] * this.distance, eye[2] + f[2] * this.distance];
  }

  // --- orbit gestures -----------------------------------------------------

  orbitBy(dAz: number, dEl: number) {
    this.tween = null;
    this.azimuth += dAz;
    this.elevation = clamp(this.elevation + dEl, -89.5, 89.5);
    this.vel.az = dAz;
    this.vel.el = dEl;
  }

  /** Pan in screen space; dx/dy in pixels, `h` the viewport height. */
  panBy(dx: number, dy: number, h: number) {
    this.tween = null;
    const scale = (2 * this.distance * Math.tan(this.fovY / 2)) / h;
    const f = this.forward();
    let rx = f[1], ry = -f[0];
    const rl = Math.hypot(rx, ry) || 1;
    rx /= rl;
    ry /= rl;
    const ux = ry * f[2], uy = -rx * f[2], uz = rx * f[1] - ry * f[0];
    this.target = [
      this.target[0] - (rx * dx - ux * dy) * scale,
      this.target[1] - (ry * dx - uy * dy) * scale,
      this.target[2] + uz * dy * scale,
    ];
    this.vel.px = dx;
    this.vel.py = dy;
  }

  zoomBy(factor: number) {
    this.tween = null;
    if (this.mode === "fly") return;
    this.distance = clamp(this.distance * factor, 50, this.sceneRadius * 12 + 20000);
  }

  // --- fly gestures -------------------------------------------------------

  lookBy(dAz: number, dEl: number) {
    this.tween = null;
    const eye = this.eye();
    this.azimuth += dAz;
    this.elevation = clamp(this.elevation + dEl, -89.5, 89.5);
    this.placeEye(eye);
  }

  /** Walks by the held keys: `input` is (forward, right, up), each -1..1. */
  fly(input: Vec3, fast: boolean, dt: number): boolean {
    const f = this.forward();
    let rx = f[1], ry = -f[0];
    const rl = Math.hypot(rx, ry) || 1;
    rx /= rl;
    ry /= rl;
    const want: Vec3 = [
      f[0] * input[0] + rx * input[1],
      f[1] * input[0] + ry * input[1],
      f[2] * input[0] + input[2],
    ];
    const wl = Math.hypot(want[0], want[1], want[2]);
    const speed = this.flySpeed * (fast ? 4 : 1);
    const target: Vec3 = wl > 0 ? [(want[0] / wl) * speed, (want[1] / wl) * speed, (want[2] / wl) * speed] : [0, 0, 0];
    // Ease the velocity towards what the keys ask for, so starts and stops
    // feel smooth rather than jerky.
    const k = 1 - Math.exp(-dt * 10);
    for (let i = 0; i < 3; i++) this.flyVel[i] += (target[i] - this.flyVel[i]) * k;
    const moving = Math.hypot(...this.flyVel) > 1;
    if (moving) {
      this.target = [this.target[0] + this.flyVel[0] * dt, this.target[1] + this.flyVel[1] * dt, this.target[2] + this.flyVel[2] * dt];
    } else {
      this.flyVel = [0, 0, 0];
    }
    return moving;
  }

  // --- animation ----------------------------------------------------------

  /** Continues momentum after a drag. Returns true while still moving. */
  coast(dt: number, h: number): boolean {
    if (this.tween) {
      const t = clamp((performance.now() - this.tween.start) / this.tween.duration, 0, 1);
      const e = easeInOutCubic(t);
      const a = this.tween.from, b = this.tween.to;
      this.target = [a.target[0] + (b.target[0] - a.target[0]) * e, a.target[1] + (b.target[1] - a.target[1]) * e, a.target[2] + (b.target[2] - a.target[2]) * e];
      this.distance = a.distance + (b.distance - a.distance) * e;
      this.azimuth = a.azimuth + (b.azimuth - a.azimuth) * e;
      this.elevation = a.elevation + (b.elevation - a.elevation) * e;
      if (t >= 1) this.tween = null;
      return true;
    }
    if (this.mode === "fly") return false;
    const decay = Math.exp(-dt * 7);
    const moving = Math.abs(this.vel.az) + Math.abs(this.vel.el) > 0.01 || Math.abs(this.vel.px) + Math.abs(this.vel.py) > 0.05;
    if (!moving) {
      this.vel = { az: 0, el: 0, px: 0, py: 0 };
      return false;
    }
    const s = dt * 60;
    this.azimuth += this.vel.az * s * 0.9;
    this.elevation = clamp(this.elevation + this.vel.el * s * 0.9, -89.5, 89.5);
    if (this.vel.px || this.vel.py) {
      const px = this.vel.px, py = this.vel.py;
      this.panBy(px * s * 0.9, py * s * 0.9, h);
      this.vel.px = px;
      this.vel.py = py;
    }
    this.vel.az *= decay;
    this.vel.el *= decay;
    this.vel.px *= decay;
    this.vel.py *= decay;
    return true;
  }

  stopCoast() {
    this.vel = { az: 0, el: 0, px: 0, py: 0 };
  }

  animateTo(to: Partial<{ target: Vec3; distance: number; azimuth: number; elevation: number }>, duration = 700) {
    const from = { target: [...this.target] as Vec3, distance: this.distance, azimuth: this.azimuth, elevation: this.elevation };
    const dest = {
      target: to.target ?? from.target,
      distance: this.mode === "fly" ? FLY_DISTANCE : to.distance ?? from.distance,
      azimuth: to.azimuth ?? from.azimuth,
      elevation: to.elevation ?? from.elevation,
    };
    // Take the short way round.
    while (dest.azimuth - from.azimuth > 180) dest.azimuth -= 360;
    while (dest.azimuth - from.azimuth < -180) dest.azimuth += 360;
    if (this.mode === "fly" && to.distance !== undefined) {
      // Flying: frame by placing the eye where the orbit camera would be.
      const el = (dest.elevation * Math.PI) / 180, az = (dest.azimuth * Math.PI) / 180;
      const d: Vec3 = [Math.cos(el) * Math.cos(az), Math.cos(el) * Math.sin(az), Math.sin(el)];
      const eye: Vec3 = [dest.target[0] + d[0] * to.distance, dest.target[1] + d[1] * to.distance, dest.target[2] + d[2] * to.distance];
      dest.target = [eye[0] - d[0] * FLY_DISTANCE, eye[1] - d[1] * FLY_DISTANCE, eye[2] - d[2] * FLY_DISTANCE];
      this.orbitDistance = to.distance;
    }
    this.vel = { az: 0, el: 0, px: 0, py: 0 };
    this.tween = { from, to: dest, start: performance.now(), duration };
  }

  get animating(): boolean {
    return this.tween !== null;
  }
}
