// Colour ramps shared with the backend's exporter (src-tauri/src/export.rs
// PALETTE and jobs.rs SCAN_COLORS) -- keep the two in step.

export const HEIGHT_STOPS: [number, number, number][] = [
  [0.231, 0.169, 0.71],
  [0.184, 0.486, 0.965],
  [0.122, 0.761, 0.761],
  [0.62, 0.878, 0.29],
  [1.0, 0.812, 0.247],
];

export const SCAN_COLORS: [number, number, number][] = [
  [0.039, 0.518, 1.0],
  [1.0, 0.584, 0.0],
  [0.204, 0.78, 0.349],
  [1.0, 0.176, 0.333],
  [0.686, 0.322, 0.871],
  [0.188, 0.69, 0.78],
  [1.0, 0.8, 0.0],
  [0.639, 0.518, 0.369],
];

/** 256 RGBA texels sampled from the stops, for a 1-D lookup texture. */
export function rampTexels(stops = HEIGHT_STOPS): Uint8Array {
  const out = new Uint8Array(256 * 4);
  for (let i = 0; i < 256; i++) {
    const t = i / 255;
    const x = t * (stops.length - 1);
    const k = Math.min(Math.floor(x), stops.length - 2);
    const f = x - k;
    for (let c = 0; c < 3; c++) {
      const v = stops[k][c] + (stops[k + 1][c] - stops[k][c]) * f;
      out[i * 4 + c] = Math.round(v * 255);
    }
    out[i * 4 + 3] = 255;
  }
  return out;
}

export function rampCss(stops = HEIGHT_STOPS): string {
  const parts = stops.map((s, i) => {
    const [r, g, b] = s.map((v) => Math.round(v * 255));
    return `rgb(${r} ${g} ${b}) ${(i / (stops.length - 1)) * 100}%`;
  });
  return `linear-gradient(90deg, ${parts.join(", ")})`;
}

export function scanColorCss(i: number): string {
  const [r, g, b] = SCAN_COLORS[i % SCAN_COLORS.length].map((v) => Math.round(v * 255));
  return `rgb(${r} ${g} ${b})`;
}
