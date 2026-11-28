/* ===== 視覺層：Scale-then-DQS 雙四元數蒙皮 + 保層次骨面包繞約束 ===== */

function unpackSkinnedMesh(m, buf) {
  const nv = m.nv, nf = m.nf;
  const q16 = new Uint16Array(buf, m.off, nv * 3);
  const pos = new Float32Array(nv * 3);
  const lo = m.lo, q = m.q;
  for (let i = 0; i < nv; i++) {
    pos[i * 3]     = lo[0] + q16[i * 3]     * q;
    pos[i * 3 + 1] = lo[1] + q16[i * 3 + 1] * q;
    pos[i * 3 + 2] = lo[2] + q16[i * 3 + 2] * q;
  }
  const n8 = new Int8Array(buf, m.off + nv * 6, nv * 3);
  const nrm = new Float32Array(nv * 3);
  for (let i = 0; i < nv * 3; i++) nrm[i] = n8[i] / 127.0;
  const idx = new Uint8Array(buf, m.off + nv * 9, nv * 4).slice();
  const w   = new Uint8Array(buf, m.off + nv * 13, nv * 4).slice();
  const tFac = new Uint8Array(buf, m.off + nv * 17, nv).slice();
  const faces = new Uint16Array(buf, m.offF, nf * 3).slice();
  return { nv, nf, hasT: !!m.hasT, pos, nrm, idx, w, tFac, faces };
}

/* 腳底肌腱／足內在肌：位於 MTP 之前的腳掌頂點平滑轉綁到趾骨虛擬肢段 */
function remapToesDQS(mu, man) {
  const n = man.segs.length;
  for (const [S, h] of [['R', 0], ['L', 1]]) {
    const b = BIND[S + '_foot'];
    if (!b) continue;
    const fi = b.i, Ob = b.Ob, toe = n + h, mtp = getMTP(man.meta, S);
    const lo = mtp.x - 0.012, wd = 0.024;
    for (let p = 0; p < mu.nv; p++) {
      const x = mu.pos[p * 3] - Ob[0];
      const t = Math.max(0, Math.min(1, (x - lo) / wd));
      if (t <= 0) continue;
      const ts = t * t * (3 - 2 * t);
      for (let k = 0; k < 4; k++) {
        if (mu.idx[p * 4 + k] !== fi || !mu.w[p * 4 + k]) continue;
        const mv = Math.round(mu.w[p * 4 + k] * ts);
        if (!mv) break;
        let free = -1, same = -1;
        for (let j = 0; j < 4; j++) {
          if (mu.idx[p * 4 + j] === toe && mu.w[p * 4 + j]) same = j;
          else if (!mu.w[p * 4 + j]) free = j;
        }
        const dst = same >= 0 ? same : free;
        if (dst < 0) break;
        if (same >= 0) mu.w[p * 4 + dst] += mv;
        else { mu.idx[p * 4 + dst] = toe; mu.w[p * 4 + dst] = mv; }
        mu.w[p * 4 + k] -= mv;
        break;
      }
    }
  }
}

/* ===== 解析骨面包繞（胸廓肋籠、骨盆、肱骨頭）：維持綁定姿態深度層次 r0 ===== */
function poseWraps(po, meta) {
  const spec = (meta && meta.wrap) || [];
  return spec.map(s => {
    const sg = po[s.seg];
    return sg ? { O: sg.O, x: sg.F.x, y: sg.F.y, z: sg.F.z, sx: sg.S[0], sy: sg.S[1], sz: sg.S[2], c: s.c, r: s.r } : null;
  });
}

function wrapRad(x, y, z, w) {
  const dx = x - w.O[0], dy = y - w.O[1], dz = z - w.O[2];
  const lx = (w.x[0] * dx + w.x[1] * dy + w.x[2] * dz) / w.sx - w.c[0];
  const ly = (w.y[0] * dx + w.y[1] * dy + w.y[2] * dz) / w.sy - w.c[1];
  const lz = (w.z[0] * dx + w.z[1] * dy + w.z[2] * dz) / w.sz - w.c[2];
  return Math.hypot(lx / w.r[0], ly / w.r[1], lz / w.r[2]);
}

function wrapPush(x, y, z, w, rMin) {
  const dx = x - w.O[0], dy = y - w.O[1], dz = z - w.O[2];
  const lx = (w.x[0] * dx + w.x[1] * dy + w.x[2] * dz) / w.sx - w.c[0];
  const ly = (w.y[0] * dx + w.y[1] * dy + w.y[2] * dz) / w.sy - w.c[1];
  const lz = (w.z[0] * dx + w.z[1] * dy + w.z[2] * dz) / w.sz - w.c[2];
  const rad = Math.hypot(lx / w.r[0], ly / w.r[1], lz / w.r[2]);
  if (rad >= rMin || rad < 1e-4) return null;
  const sc = rMin / rad;
  const fx = (lx * sc + w.c[0]) * w.sx, fy = (ly * sc + w.c[1]) * w.sy, fz = (lz * sc + w.c[2]) * w.sz;
  return [
    w.O[0] + w.x[0] * fx + w.y[0] * fy + w.z[0] * fz,
    w.O[1] + w.x[1] * fx + w.y[1] * fy + w.z[1] * fz,
    w.O[2] + w.x[2] * fx + w.y[2] * fy + w.z[2] * fz
  ];
}

const _WRAP_EX = /^(diaphragm|psoas major|psoas minor|iliacus|quadratus lumborum|transversus thoracis|intercostal)/;

function initMeshWraps(items, pn, matsN, meta) {
  const wrN = poseWraps(pn, meta);
  if (!wrN.length) return;
  for (const item of items) {
    const mu = item.mu;
    if (!mu) continue;
    mu._vwV = null;
    if (item.key && _WRAP_EX.test(item.key)) continue;
    if (item.rigid) continue;
    const tmpP = new Float32Array(mu.nv * 3), tmpN = new Float32Array(mu.nv * 3);
    skinMeshDQS(mu, matsN, tmpP, tmpN, null);
    const vv = [], vw = [], vr = [];
    for (let v = 0; v < mu.nv; v++) {
      // Three.js (x, y, z) -> lab (x, -z, y)
      const x = tmpP[v * 3], y = -tmpP[v * 3 + 2], z = tmpP[v * 3 + 1];
      for (let wi = 0; wi < wrN.length; wi++) {
        const w = wrN[wi];
        if (!w) continue;
        const r0 = wrapRad(x, y, z, w);
        if (r0 >= 0.96 && r0 <= 1.25) {
          vv.push(v); vw.push(wi); vr.push(r0);
        }
      }
    }
    if (vv.length) {
      mu._vwV = new Uint16Array(vv);
      mu._vwW = new Uint8Array(vw);
      mu._vwR = new Float32Array(vr);
    }
  }
}

/* ===== Scale-then-DQS 雙四元數蒙皮（輸出 Three.js 座標） ===== */
function skinMeshDQS(mu, mp, outP, outN, wr) {
  const { nv, pos, nrm, idx, w } = mu;
  const { A, dq, scM } = mp;
  let wrapped = false;

  for (let v = 0; v < nv; v++) {
    const v3 = v * 3, v4 = v * 4;
    const px = pos[v3], py = pos[v3 + 1], pz = pos[v3 + 2];
    const nx = nrm[v3], ny = nrm[v3 + 1], nz = nrm[v3 + 2];
    const w0 = w[v4];

    if (w0 === 255) {
      // 單骨剛體快速路徑（完全精準貼合骨面）
      const o = idx[v4] * 12;
      const X = A[o] * px + A[o + 1] * py + A[o + 2] * pz + A[o + 9];
      const Y = A[o + 3] * px + A[o + 4] * py + A[o + 5] * pz + A[o + 10];
      const Z = A[o + 6] * px + A[o + 7] * py + A[o + 8] * pz + A[o + 11];
      const NX = A[o] * nx + A[o + 1] * ny + A[o + 2] * nz;
      const NY = A[o + 3] * nx + A[o + 4] * ny + A[o + 5] * nz;
      const NZ = A[o + 6] * nx + A[o + 7] * ny + A[o + 8] * nz;
      const nl = Math.hypot(NX, NY, NZ) || 1;
      outP[v3] = X; outP[v3 + 1] = Z; outP[v3 + 2] = -Y;
      outN[v3] = NX / nl; outN[v3 + 1] = NZ / nl; outN[v3 + 2] = -NY / nl;
      continue;
    }

    // 1. 綁定空間加權縮放 + LBS 凸包基準 + 雙四元數混合
    let vx = 0, vy = 0, vz = 0;
    let lx = 0, ly = 0, lz = 0;
    let rw = 0, rx_ = 0, ry_ = 0, rz_ = 0;
    let dw = 0, dx = 0, dy = 0, dz = 0;
    let r0w = 0, r0x = 0, r0y = 0, r0z = 0, hasRef = false, minDot = 1.0;

    for (let k = 0; k < 4; k++) {
      const wk = w[v4 + k];
      if (!wk) continue;
      const f = wk / 255.0;
      const bi = idx[v4 + k];
      const so = bi * 12;
      vx += f * (scM[so] * px + scM[so + 1] * py + scM[so + 2] * pz + scM[so + 9]);
      vy += f * (scM[so + 3] * px + scM[so + 4] * py + scM[so + 5] * pz + scM[so + 10]);
      vz += f * (scM[so + 6] * px + scM[so + 7] * py + scM[so + 8] * pz + scM[so + 11]);
      lx += f * (A[so] * px + A[so + 1] * py + A[so + 2] * pz + A[so + 9]);
      ly += f * (A[so + 3] * px + A[so + 4] * py + A[so + 5] * pz + A[so + 10]);
      lz += f * (A[so + 6] * px + A[so + 7] * py + A[so + 8] * pz + A[so + 11]);

      const d = bi * 8;
      let sgn = f;
      if (!hasRef) {
        r0w = dq[d]; r0x = dq[d + 1]; r0y = dq[d + 2]; r0z = dq[d + 3];
        hasRef = true;
      } else {
        const dp = r0w * dq[d] + r0x * dq[d + 1] + r0y * dq[d + 2] + r0z * dq[d + 3];
        const ad = Math.abs(dp);
        if (ad < minDot) minDot = ad;
        if (dp < 0) sgn = -f;
      }
      rw += sgn * dq[d];     rx_ += sgn * dq[d + 1]; ry_ += sgn * dq[d + 2]; rz_ += sgn * dq[d + 3];
      dw += sgn * dq[d + 4]; dx  += sgn * dq[d + 5]; dy  += sgn * dq[d + 6]; dz  += sgn * dq[d + 7];
    }

    // 2. 雙四元數正規化並變換頂點與法線（搭配大轉角螺旋膨脹保護，杜絕 C 型弧線飛出）
    const invL = 1.0 / (Math.hypot(rw, rx_, ry_, rz_) || 1.0);
    rw *= invL; rx_ *= invL; ry_ *= invL; rz_ *= invL;
    dw *= invL; dx  *= invL; dy  *= invL; dz  *= invL;

    const tx = 2.0 * (-dw * rx_ + dx * rw - dy * rz_ + dz * ry_);
    const ty = 2.0 * (-dw * ry_ + dx * rz_ + dy * rw - dz * rx_);
    const tz = 2.0 * (-dw * rz_ - dx * ry_ + dy * rx_ + dz * rw);

    const cx1 = ry_ * vz - rz_ * vy + rw * vx;
    const cy1 = rz_ * vx - rx_ * vz + rw * vy;
    const cz1 = rx_ * vy - ry_ * vx + rw * vz;
    let X = vx + 2.0 * (ry_ * cz1 - rz_ * cy1) + tx;
    let Y = vy + 2.0 * (rz_ * cx1 - rx_ * cz1) + ty;
    let Z = vz + 2.0 * (rx_ * cy1 - ry_ * cx1) + tz;

    // 螺旋膨脹保護：當 DQS 與 LBS 偏差超過 1.2 cm（大轉角遠距離雙骨混合）時平滑限幅至 1.2 cm，消除 C 型飛弧同時保留近處保體積抗扭扁
    const bx = X - lx, by = Y - ly, bz = Z - lz;
    const bd = Math.hypot(bx, by, bz);
    if (bd > 0.012) {
      const cap = 0.012 / bd;
      X = lx + bx * cap; Y = ly + by * cap; Z = lz + bz * cap;
    }

    const ncx = ry_ * nz - rz_ * ny + rw * nx;
    const ncy = rz_ * nx - rx_ * nz + rw * ny;
    const ncz = rx_ * ny - ry_ * nx + rw * nz;
    const NX = nx + 2.0 * (ry_ * ncz - rz_ * ncy);
    const NY = ny + 2.0 * (rz_ * ncx - rx_ * ncz);
    const NZ = nz + 2.0 * (rx_ * ncy - ry_ * ncx);

    outP[v3] = X; outP[v3 + 1] = Z; outP[v3 + 2] = -Y;
    outN[v3] = NX; outN[v3 + 1] = NZ; outN[v3 + 2] = -NY;
  }

  // 3. 保層次骨面包繞約束
  if (wr && mu._vwV) {
    const vV = mu._vwV, vW = mu._vwW, vR = mu._vwR;
    for (let i = 0; i < vV.length; i++) {
      const v = vV[i], w2 = wr[vW[i]];
      if (!w2) continue;
      const q = wrapPush(outP[v * 3], -outP[v * 3 + 2], outP[v * 3 + 1], w2, vR[i]);
      if (q) {
        outP[v * 3] = q[0]; outP[v * 3 + 1] = q[2]; outP[v * 3 + 2] = -q[1];
        wrapped = true;
      }
    }
  }
  return wrapped;
}
