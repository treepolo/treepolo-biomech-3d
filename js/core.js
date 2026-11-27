/* ===== 向量、矩陣與雙四元數（Dual Quaternion）工具 ===== */
const V = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  len: a => Math.hypot(a[0], a[1], a[2]),
  norm(a) { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; },
  lerp: (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]
};

const UP = [0, 0, 1];
const D2R = Math.PI / 180;

function orth(z, y) { const d = V.dot(z, y); return V.norm(V.sub(z, V.mul(y, d))); }
function frm(y, zh) { const z = orth(zh, y); return { x: V.cross(y, z), y, z }; }
function smooth01(a, b, x) { if (b === a) return x >= b ? 1 : 0; const t = Math.max(0, Math.min(1, (x - a) / (b - a))); return t * t * (3 - 2 * t); }
function median(a) { const s = Array.from(a).sort((p, q) => p - q); return s[s.length >> 1]; }
function rotAbout(v, ax, ang) {
  const c = Math.cos(ang), s = Math.sin(ang);
  return V.add(V.add(V.mul(v, c), V.mul(V.cross(ax, v), s)), V.mul(ax, V.dot(ax, v) * (1 - c)));
}
function interp1(xs, ys, x) {
  if (x <= xs[0]) return ys[0];
  for (let i = 1; i < xs.length; i++) if (x <= xs[i]) {
    const t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[i - 1] + (ys[i] - ys[i - 1]) * t;
  }
  return ys[ys.length - 1];
}

const M3 = {
  mul(a, b) {
    const o = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
    return o;
  },
  Ry(a) { const c = Math.cos(a), s = Math.sin(a); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]; },
  Rx(a) { const c = Math.cos(a), s = Math.sin(a); return [[1, 0, 0], [0, c, -s], [0, s, c]]; },
  Rz(a) { const c = Math.cos(a), s = Math.sin(a); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; },
  mirrorZ(m) { return [[m[0][0], m[0][1], -m[0][2]], [m[1][0], m[1][1], -m[1][2]], [-m[2][0], -m[2][1], m[2][2]]]; }
};

function frameMul(F, R) {
  const cols = [F.x, F.y, F.z];
  const o = [0, 1, 2].map(j => V.add(V.add(V.mul(cols[0], R[0][j]), V.mul(cols[1], R[1][j])), V.mul(cols[2], R[2][j])));
  return { x: o[0], y: o[1], z: o[2] };
}

const m3 = F => [[F.x[0], F.y[0], F.z[0]], [F.x[1], F.y[1], F.z[1]], [F.x[2], F.y[2], F.z[2]]];
const m3f = M => ({ x: [M[0][0], M[1][0], M[2][0]], y: [M[0][1], M[1][1], M[2][1]], z: [M[0][2], M[1][2], M[2][2]] });
const m3m = (A, B) => A.map(r => [0, 1, 2].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const m3t = (A, B) => [0, 1, 2].map(i => [0, 1, 2].map(j => A[0][i] * B[0][j] + A[1][i] * B[1][j] + A[2][i] * B[2][j]));
const rz = M3.Rz, rx = M3.Rx, ry = M3.Ry;

function rotMat(a, ang) {
  const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c, [x, y, z] = a;
  return [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c]
  ];
}

function toWorld(seg, p) {
  const F = seg.F, S = seg.S;
  return [
    seg.O[0] + F.x[0] * S[0] * p[0] + F.y[0] * S[1] * p[1] + F.z[0] * S[2] * p[2],
    seg.O[1] + F.x[1] * S[0] * p[0] + F.y[1] * S[1] * p[1] + F.z[1] * S[2] * p[2],
    seg.O[2] + F.x[2] * S[0] * p[0] + F.y[2] * S[1] * p[1] + F.z[2] * S[2] * p[2]
  ];
}

/* ===== 雙四元數（Dual Quaternion）工具 =====
   R: row-major 3x3 (9 floats at R[o..o+8]), t: [tx, ty, tz]
   輸出寫入 dq[d..d+7]: [rw, rx, ry, rz, dw, dx, dy, dz] */
function dqFromRT(R00, R01, R02, R10, R11, R12, R20, R21, R22, tx, ty, tz, out, d) {
  let rw, rx_, ry_, rz_;
  const tr = R00 + R11 + R22;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1.0) * 2;
    rw = 0.25 * s;
    rx_ = (R21 - R12) / s;
    ry_ = (R02 - R20) / s;
    rz_ = (R10 - R01) / s;
  } else if (R00 > R11 && R00 > R22) {
    const s = Math.sqrt(1.0 + R00 - R11 - R22) * 2;
    rw = (R21 - R12) / s;
    rx_ = 0.25 * s;
    ry_ = (R01 + R10) / s;
    rz_ = (R02 + R20) / s;
  } else if (R11 > R22) {
    const s = Math.sqrt(1.0 + R11 - R00 - R22) * 2;
    rw = (R02 - R20) / s;
    rx_ = (R01 + R10) / s;
    ry_ = 0.25 * s;
    rz_ = (R12 + R21) / s;
  } else {
    const s = Math.sqrt(1.0 + R22 - R00 - R11) * 2;
    rw = (R10 - R01) / s;
    rx_ = (R02 + R20) / s;
    ry_ = (R12 + R21) / s;
    rz_ = 0.25 * s;
  }
  const nl = Math.hypot(rw, rx_, ry_, rz_) || 1;
  rw /= nl; rx_ /= nl; ry_ /= nl; rz_ /= nl;
  // qd = 0.5 * (0, tx, ty, tz) * qr
  const dw = -0.5 * (tx * rx_ + ty * ry_ + tz * rz_);
  const dx =  0.5 * (tx * rw  + ty * rz_ - tz * ry_);
  const dy =  0.5 * (-tx * rz_ + ty * rw  + tz * rx_);
  const dz =  0.5 * (tx * ry_ - ty * rx_ + tz * rw);
  out[d]     = rw; out[d + 1] = rx_; out[d + 2] = ry_; out[d + 3] = rz_;
  out[d + 4] = dw; out[d + 5] = dx;  out[d + 6] = dy;  out[d + 7] = dz;
}

/* 平滑與微分 */
function binom(arr, p) {
  const n = arr.length;
  let c = Float64Array.from(arr);
  if (p <= 0 || n < 3) return c;
  let x = new Float64Array(n);
  for (let k = 0; k < p; k++) {
    x[0] = c[0]; x[n - 1] = c[n - 1];
    for (let i = 1; i < n - 1; i++) x[i] = 0.25 * c[i - 1] + 0.5 * c[i] + 0.25 * c[i + 1];
    const t = c; c = x; x = t;
  }
  return c;
}
function cdiff(a, t) {
  const n = a.length, o = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const i0 = Math.max(0, i - 1), i1 = Math.min(n - 1, i + 1);
    o[i] = (a[i1] - a[i0]) / Math.max(1e-5, t[i1] - t[i0]);
  }
  return o;
}
function polyLen(pts) { let L = 0; for (let i = 1; i < pts.length; i++) L += V.len(V.sub(pts[i], pts[i - 1])); return L; }
function uclLen(tab, flex) {
  const f = Math.max(0, Math.min(140, flex)) / 2, i = Math.floor(f), t = f - i;
  const g = a => a[i] + (a[Math.min(70, i + 1)] - a[i]) * t;
  return { ant: g(tab.ant), post: g(tab.post), pb: g(tab.pb) };
}
function b64buf(b) { const s = atob(b), u = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i); return u.buffer; }
