// GLSL for the viewer. Everything draws into an offscreen target with two
// colour attachments -- premultiplied colour, and view-space depth (0 where
// nothing was drawn) -- and a final pass composites onto the background with
// Eye-Dome Lighting, which is what gives a bare point cloud its sense of
// shape without normals.

export const POINT_VS = /* glsl */ `#version 300 es
precision highp float;
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aRange;
layout(location = 2) in float aOrder;
layout(location = 3) in vec3 aRgb;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform float uPointSize;   // world size, mm
uniform float uProjScale;   // pixels per unit at distance 1
uniform float uMinPx;
uniform float uMaxPx;
uniform int uColorMode;     // 0 height, 1 distance, 2 mono, 3 flat, 4 rgb
uniform vec2 uHeightRange;
uniform vec2 uRangeRange;
uniform vec3 uFlatColor;
uniform vec3 uMonoColor;
uniform float uReveal;      // points past this sweep fraction are hidden
uniform float uLiveCount;   // > 0: live cloud, the newest points glow
uniform float uHighlight;
uniform vec3 uHighlightColor;
uniform float uClipTop;     // world z above which points are cut away
uniform sampler2D uRamp;

out vec3 vColor;
out float vDepth;

void main() {
  float order = aOrder / 65535.0;
  vec4 world = uModel * vec4(aPos, 1.0);
  if (order > uReveal || world.z > uClipTop) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    gl_PointSize = 0.0;
    vColor = vec3(0.0);
    vDepth = 0.0;
    return;
  }
  vec4 view = uView * world;
  gl_Position = uProj * view;
  float d = max(-view.z, 1.0);
  gl_PointSize = clamp(uPointSize * uProjScale / d, uMinPx, uMaxPx);

  vec3 c;
  if (uColorMode == 0) {
    float t = clamp((world.z - uHeightRange.x) / max(uHeightRange.y - uHeightRange.x, 1.0), 0.0, 1.0);
    c = texture(uRamp, vec2(t * 0.992 + 0.004, 0.5)).rgb;
  } else if (uColorMode == 1) {
    float t = clamp((aRange - uRangeRange.x) / max(uRangeRange.y - uRangeRange.x, 1.0), 0.0, 1.0);
    c = texture(uRamp, vec2(t * 0.992 + 0.004, 0.5)).rgb;
  } else if (uColorMode == 2) {
    c = uMonoColor;
  } else if (uColorMode == 3) {
    c = uFlatColor;
  } else {
    c = aRgb;
  }
  c = mix(c, uHighlightColor, uHighlight);

  // The sweep front glows as a scan is revealed.
  float front = clamp(1.0 - (uReveal - order) * 14.0, 0.0, 1.0);
  c += front * front * 0.55;
  // So do the newest points of a scan still arriving.
  if (uLiveCount > 0.0) {
    float age = uLiveCount - float(gl_VertexID);
    c += exp(-age / 2600.0) * 0.6;
  }
  vColor = c;
  vDepth = d;
}
`;

export const POINT_FS = /* glsl */ `#version 300 es
precision highp float;
in vec3 vColor;
in float vDepth;
layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outDepth;
void main() {
  vec2 p = gl_PointCoord * 2.0 - 1.0;
  float r2 = dot(p, p);
  if (r2 > 1.0) discard;
  outColor = vec4(vColor, 1.0);
  outDepth = vec4(vDepth, 0.0, 0.0, 1.0);
}
`;

export const MESH_VS = /* glsl */ `#version 300 es
precision highp float;
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
out vec3 vWorld;
out vec3 vNormal;
out float vDepth;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vec4 view = uView * world;
  gl_Position = uProj * view;
  vWorld = world.xyz;
  vNormal = mat3(uModel) * aNormal;
  vDepth = -view.z;
}
`;

// Studio lighting: hemispheric ambient plus key, fill and rim, lit on both
// faces because a scanned room is an open shell seen from either side.
export const MESH_FS = /* glsl */ `#version 300 es
precision highp float;
in vec3 vWorld;
in vec3 vNormal;
in float vDepth;
uniform vec3 uEye;
uniform int uColorMode;   // 0 clay, 1 height
uniform vec2 uHeightRange;
uniform vec3 uClay;
uniform float uClipTop;
uniform sampler2D uRamp;
layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outDepth;
void main() {
  if (vWorld.z > uClipTop) discard;
  vec3 N = normalize(vNormal);
  vec3 V = normalize(uEye - vWorld);
  if (dot(N, V) < 0.0) N = -N;
  vec3 base = uClay;
  if (uColorMode == 1) {
    float t = clamp((vWorld.z - uHeightRange.x) / max(uHeightRange.y - uHeightRange.x, 1.0), 0.0, 1.0);
    base = texture(uRamp, vec2(t * 0.992 + 0.004, 0.5)).rgb;
  }
  vec3 Lkey = normalize(vec3(-0.45, 0.35, 0.82));
  vec3 Lfill = normalize(vec3(0.7, -0.5, 0.3));
  float up = N.z * 0.5 + 0.5;
  vec3 amb = mix(vec3(0.20, 0.19, 0.19), vec3(0.34, 0.36, 0.40), up);
  float dk = max(dot(N, Lkey), 0.0);
  float df = max(dot(N, Lfill), 0.0);
  float head = max(dot(N, V), 0.0);
  vec3 H = normalize(Lkey + V);
  float spec = pow(max(dot(N, H), 0.0), 48.0) * 0.10;
  float rim = pow(1.0 - head, 3.0) * 0.18;
  vec3 rgb = base * (amb + vec3(1.0, 0.96, 0.90) * dk * 0.55 + vec3(0.55, 0.62, 0.78) * df * 0.22 + head * 0.25)
           + spec + rim;
  outColor = vec4(rgb, 1.0);
  outDepth = vec4(-vDepth, 0.0, 0.0, 1.0);  // negative: mesh, lighter EDL
}
`;

// A ground grid: anti-aliased lines every 10 cm and 1 m, fading with
// distance so it reads as a floor rather than a pattern.
export const GRID_VS = /* glsl */ `#version 300 es
precision highp float;
layout(location = 0) in vec2 aPos;
uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uCenter;
uniform float uHalf;
out vec2 vXY;
out float vDist;
void main() {
  vec3 w = vec3(uCenter.xy + aPos * uHalf, uCenter.z);
  vXY = w.xy;
  vec4 view = uView * vec4(w, 1.0);
  vDist = length(aPos);
  gl_Position = uProj * view;
}
`;

export const GRID_FS = /* glsl */ `#version 300 es
precision highp float;
in vec2 vXY;
in float vDist;
uniform vec3 uLineColor;
uniform float uOpacity;
layout(location = 0) out vec4 outColor;
float lines(vec2 p, float step, float width) {
  vec2 g = abs(fract(p / step - 0.5) - 0.5) / fwidth(p / step);
  return 1.0 - min(min(g.x, g.y) / width, 1.0);
}
void main() {
  float minor = lines(vXY, 100.0, 1.0) * 0.35;
  float major = lines(vXY, 1000.0, 1.2);
  float a = max(minor, major);
  float fade = 1.0 - smoothstep(0.35, 1.0, vDist);
  a *= fade * uOpacity;
  if (a < 0.003) discard;
  outColor = vec4(uLineColor * a, a);
}
`;

// The scan plane: a soft translucent disc in the lidar's vertical plane,
// turning with the platform while a sweep runs.
export const FAN_VS = /* glsl */ `#version 300 es
precision highp float;
layout(location = 0) in vec2 aPos;  // unit disc: x along the plane, y up
uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uOrigin;
uniform float uAngle;
uniform float uRadius;
out vec2 vP;
void main() {
  vP = aPos;
  vec3 dir = vec3(cos(uAngle), sin(uAngle), 0.0);
  vec3 w = uOrigin + dir * aPos.x * uRadius + vec3(0.0, 0.0, 1.0) * aPos.y * uRadius;
  gl_Position = uProj * uView * vec4(w, 1.0);
}
`;

export const FAN_FS = /* glsl */ `#version 300 es
precision highp float;
in vec2 vP;
uniform vec3 uColor;
uniform float uOpacity;
layout(location = 0) out vec4 outColor;
void main() {
  float r = length(vP);
  if (r > 1.0) discard;
  float body = (1.0 - r) * 0.22;
  float ring = smoothstep(0.93, 0.985, r) * (1.0 - smoothstep(0.985, 1.0, r)) * 0.9;
  float a = (body + ring) * uOpacity;
  outColor = vec4(uColor * a, a);
}
`;

// Small glowing discs marking where each scanner stood.
export const MARKER_VS = /* glsl */ `#version 300 es
precision highp float;
layout(location = 0) in vec3 aPos;
uniform mat4 uView;
uniform mat4 uProj;
uniform float uSize;
void main() {
  gl_Position = uProj * uView * vec4(aPos, 1.0);
  gl_PointSize = uSize;
}
`;

export const MARKER_FS = /* glsl */ `#version 300 es
precision highp float;
uniform vec3 uColor;
layout(location = 0) out vec4 outColor;
void main() {
  vec2 p = gl_PointCoord * 2.0 - 1.0;
  float r = length(p);
  if (r > 1.0) discard;
  float core = 1.0 - smoothstep(0.30, 0.42, r);
  float halo = (1.0 - r) * 0.55;
  float a = max(core, halo);
  vec3 c = mix(uColor, vec3(1.0), core * 0.85);
  outColor = vec4(c * a, a);
}
`;

export const COMPOSITE_VS = /* glsl */ `#version 300 es
precision highp float;
out vec2 vUv;
void main() {
  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  vUv = p;
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
`;

export const COMPOSITE_FS = /* glsl */ `#version 300 es
precision highp float;
in vec2 vUv;
uniform sampler2D uColor;
uniform sampler2D uDepth;
uniform vec2 uTexel;
uniform float uEdl;         // strength, 0 disables
uniform float uRadius;      // pixels
uniform vec3 uBgTop;
uniform vec3 uBgBottom;
uniform vec3 uBgGlow;
uniform float uOutline;     // silhouette darkening on the background
out vec4 outColor;

const vec2 N8[8] = vec2[8](
  vec2(1.0, 0.0), vec2(0.7071, 0.7071), vec2(0.0, 1.0), vec2(-0.7071, 0.7071),
  vec2(-1.0, 0.0), vec2(-0.7071, -0.7071), vec2(0.0, -1.0), vec2(0.7071, -0.7071));

void main() {
  vec4 c = texture(uColor, vUv);
  float raw = texture(uDepth, vUv).r;
  float dist = abs(raw);

  // Background: a soft vertical gradient with a faint glow at the centre.
  float g = smoothstep(0.0, 1.0, vUv.y);
  vec3 bg = mix(uBgBottom, uBgTop, g);
  float r = length((vUv - vec2(0.5, 0.55)) * vec2(1.3, 1.0));
  bg += uBgGlow * (1.0 - smoothstep(0.0, 0.85, r));

  if (dist > 0.0 && uEdl > 0.0) {
    float ld = log2(dist);
    float sum = 0.0;
    for (int i = 0; i < 8; i++) {
      float n = abs(texture(uDepth, vUv + N8[i] * uRadius * uTexel).r);
      if (n > 0.0) sum += max(0.0, ld - log2(n));
    }
    float strength = raw < 0.0 ? uEdl * 0.35 : uEdl;
    float shade = exp(-sum / 8.0 * 260.0 * strength);
    c.rgb *= shade;
  } else if (dist == 0.0 && uOutline > 0.0) {
    float hits = 0.0;
    for (int i = 0; i < 8; i += 2) {
      if (texture(uDepth, vUv + N8[i] * 1.5 * uTexel).r > 0.0) hits += 1.0;
    }
    bg *= 1.0 - uOutline * hits / 4.0;
  }
  outColor = vec4(c.rgb + bg * (1.0 - c.a), 1.0);
}
`;

// Thumbnails and picking read the offscreen targets back; this blits the
// composited image into an RGBA8 target for readPixels.
export const BLIT_FS = COMPOSITE_FS;
