// Mirrors of the backend's serde types (camelCase on the wire).

export type Lang = "ru" | "en";
export type ThemePref = "system" | "light" | "dark";
export type ColorMode = "height" | "distance" | "mono" | "scan";
export type CamMode = "orbit" | "fly";
export type Quality = "medium" | "high" | "max";

export interface Mount {
  lidarRotation: number;
  lidarReverse: boolean;
  emitterSpacing: number;
  scanTilt: number;
  scanHalf: "both" | "a" | "b";
  flipUpright: boolean;
  beamOffset: number;
  minRange: number;
  maxRange: number;
  microstepAuto: boolean;
  rangeCorrection: boolean;
  rangeError: number[] | null;
}

export interface Settings {
  language: "system" | Lang;
  theme: ThemePref;
  libraryDir: string | null;
  scan: { duration: number; startDelay: number; stepped: boolean; dwellMs: number; sweepDeg: number };
  device: { autoConnect: boolean; dtr: boolean; knownSerials: string[]; calibPending: boolean };
  mount: Mount;
  view: { colorMode: ColorMode; pointSize: number; grid: boolean; edl: boolean; camera: CamMode; flySpeed: number };
  mesh: { enabled: boolean; quality: Quality; trim: number; budget: number; smooth: number };
  merge: { mode: "auto" | "small" | "hint"; yawHint: number; voxel: number; ignoreFoliage: boolean };
  flyHintSeen: boolean;
}

export type ScanKind = "device" | "bin" | "ply";

export interface ScanMeta {
  id: string;
  name: string | null;
  created: string;
  kind: ScanKind;
  file: string;
  points: number;
  duration: number;
  sweepDeg: number | null;
  complete: boolean;
  group: string | null;
  pose: number[];
  microstep: number[] | null;
  sourceName: string | null;
  bytes: number;
}

export interface GroupMeta {
  id: string;
  name: string | null;
  created: string;
}

export interface LibraryDto {
  root: string;
  index: { version: number; scans: ScanMeta[]; groups: GroupMeta[] };
}

export interface DeviceConfig {
  degrees: number;
  time: number;
  mode: number;
  steps: number;
  settleMs: number;
  captureMs: number;
}

export interface PortInfo {
  device: string;
  description: string;
  vid: number | null;
  pid: number | null;
  serial: string | null;
  product: string | null;
  isScanner: boolean;
}

export type DevicePhase = "searching" | "connecting" | "ready" | "silent" | "error";

/** The calibration in the scanner's flash: "" when no scanner is connected. */
export type DeviceCalibState = "" | "reading" | "synced" | "writing" | "failed" | "unsupported";

export interface DeviceStatus {
  phase: DevicePhase | "";
  port: string | null;
  state: number;
  platform: number;
  dropped: number;
  config: DeviceConfig | null;
  error: string | null;
  bytesIn: number;
  ports: PortInfo[];
  calib: DeviceCalibState;
}

export type ScanPhase = "starting" | "preparing" | "scanning" | "finishing";

export interface ScanProgress {
  id: string;
  seq: number;
  phase: ScanPhase;
  progress: number;
  platform: number;
  sweepDeg: number;
  points: number;
  elapsed: number;
  remaining: number;
  duration: number;
}

export interface Bootstrap {
  settings: Settings;
  library: LibraryDto;
  device: DeviceStatus;
  version: string;
  logDir: string;
  rangeAt3m: number;
  rangeDefault: number[];
}

export type RegStage = "structure" | "floorPlan" | "orientation" | "refine" | "polish";
export type Verdict = "good" | "check" | "poor";
export type Reason = "lowOverlap" | "featureless" | "highResidual" | "someOverlap" | "nearlyFeatureless" | "elevatedResidual";

export type MergeEvent =
  | { kind: "progress"; unit: string[]; index: number; total: number; stage: RegStage; step: number; steps: number }
  | { kind: "pending"; unit: string[]; poses: [string, number[]][]; verdict: Verdict; reasons: Reason[]; overlap: number; rmse: number; stability: number }
  | { kind: "accepted"; unit: string[]; auto: boolean }
  | { kind: "rejected"; unit: string[] }
  | { kind: "done"; group: string | null; merged: number }
  | { kind: "failed"; code: string; detail: string | null };

export type MeshStage = "thinning" | "normals" | "solving" | "cleaning";

export interface MeshInfo {
  input: number;
  used: number;
  voxel: number;
  spacing: number;
  depth: number;
  cellMm: number;
  verts: number;
  tris: number;
  seconds: number;
}

export type MeshEvent =
  | { kind: "progress"; stage: MeshStage }
  | { kind: "done"; key: string; info: MeshInfo }
  | { kind: "failed"; detail: string };

export type CalibStage = "microstep" | "range" | "planes" | "rollSpacing" | "tiltSearch" | "allThree" | "rangeRefit" | "done";

export interface CalibScores {
  planes: number;
  up: number;
  horizon: number;
  down: number;
}

export interface CalibResult {
  rotation: number;
  spacing: number;
  tilt: number;
  rangeError: number[] | null;
  rangeErrorBefore: number[] | null;
  rangeAt3mBefore: number;
  rangeAt3mAfter: number;
  before: CalibScores;
  after: CalibScores;
  stride: number;
}

export type CalibEvent =
  | { kind: "progress"; stage: CalibStage; fraction: number }
  | { kind: "done"; result: CalibResult; scan: string }
  | { kind: "failed"; code: string; detail: string | null };

/** A row of the library list: a merged group, or a loose scan. */
export interface LibraryItem {
  id: string;
  kind: "scan" | "group";
  created: string;
  scans: ScanMeta[];
  name: string | null;
}
