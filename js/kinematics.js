/* ===== 運動學：OBP 18 標記點 + 25 關節角 → 39 剛體骨骼 + 2 虛擬趾骨 ===== */

function limbs(lm, T) {
  const rear = { hip: lm.rear_hip, knee: lm.rear_knee_jc, ankle: lm.rear_ankle_jc };
  const lead = { hip: lm.lead_hip, knee: lm.lead_knee_jc, ankle: lm.lead_ankle_jc };
  const thr = { sh: lm.shoulder_jc, el: lm.elbow_jc, wr: lm.wrist_jc, hd: lm.hand_jc };
  const glv = { sh: lm.glove_shoulder_jc, el: lm.glove_elbow_jc, wr: lm.glove_wrist_jc, hd: lm.glove_hand_jc };
  const A = Object.assign({}, rear, thr), B = Object.assign({}, lead, glv);
  return T === 'R' ? { R: A, L: B } : { L: A, R: B };
}

function detectHand(fr, iPK, iFP) {
  const a = fr[iPK].lm;
  let ant = V.sub(a.lead_knee_jc, a.lead_hip);
  ant = V.norm([ant[0], ant[1], 0]);
  const right = V.cross(ant, UP);
  const d = V.dot(V.sub(a.shoulder_jc, a.glove_shoulder_jc), right);
  const b = fr[iFP].lm;
  let f = V.sub(b.lead_ankle_jc, b.rear_ankle_jc);
  f = V.norm([f[0], f[1], 0]);
  return { T: d >= 0 ? 'R' : 'L', fwd: f };
}

function calcScales(fr, M, T) {
  const g = fn => median(fr.map(f => fn(limbs(f.lm, T), f.lm)));
  const sc = {};
  for (const S of ['R', 'L']) {
    const m = M[S];
    sc[S] = {
      F: g(l => V.len(V.sub(l[S].hip, l[S].knee))) / m.lenF,
      T: g(l => V.len(V.sub(l[S].knee, l[S].ankle))) / m.lenT,
      H: g(l => V.len(V.sub(l[S].sh, l[S].el))) / m.lenH,
      A: g(l => V.len(V.sub(l[S].el, l[S].wr))) / m.lenA,
      W: g(l => V.len(V.sub(l[S].wr, l[S].hd))) / m.lenW
    };
  }
  sc.th = g((l, lm) => V.len(V.sub(lm.thorax_prox, lm.thorax_dist))) / M.thlen;
  sc.hipw = g(l => V.len(V.sub(l.R.hip, l.L.hip))) / M.hipw;
  sc.pel = (sc.hipw + (sc.R.F + sc.L.F) / 2) / 2;
  sc.rawW = { hipw: g(l => V.len(V.sub(l.R.hip, l.L.hip))) };
  sc.lum = g((l, lm) => V.len(V.sub(lm.thorax_dist, V.mul(V.add(l.R.hip, l.L.hip), 0.5))));
  sc.shw = g(l => V.len(V.sub(l.R.sh, l.L.sh)));
  sc.cs = sc.th;
  const v = [sc.R.F, sc.L.F, sc.R.T, sc.L.T, sc.R.H, sc.L.H].sort((x, y) => x - y);
  sc.csf = (v[2] + v[3]) / 2;
  return sc;
}

function ankleMin(fr, T) {
  const o = { R: 1e9, L: 1e9 };
  for (const f of fr) {
    const l = limbs(f.lm, T);
    for (const S of ['R', 'L']) o[S] = Math.min(o[S], l[S].ankle[2]);
  }
  return o;
}

/* ===== 文獻估計模型（非實測） ===== */
const LIT = {
  pron: { FC: 33, MER: 3, BR: 3, MIR: 29, glove: 0 },
  gen: { phi: [0, 30, 60, 90, 120, 150], UR: [0, 4, 17, 30, 41, 51], PT: [0, 0, 3, 8, 19, 31], IR: [0, 0, 0, -3, -12, -25] },
  pit: { ratio: 4.2, IR_FC: 1, IR_MER: 15 }
};

function pronManzi(i, E) {
  const k = [E.fp10, E.mer, E.br, E.mir], v = [LIT.pron.FC, LIT.pron.MER, LIT.pron.BR, LIT.pron.MIR];
  if (i <= k[0]) return v[0];
  if (i >= k[3]) return v[3];
  for (let j = 1; j < 4; j++) if (i <= k[j]) { const t = smooth01(k[j - 1], k[j], i); return v[j - 1] + (v[j] - v[j - 1]) * t; }
  return v[3];
}

function scapAngles(phi, sigma) {
  const g = LIT.gen;
  const URg = interp1(g.phi, g.UR, phi), PT = interp1(g.phi, g.PT, phi), IRg = interp1(g.phi, g.IR, phi);
  const URp = phi / (1 + LIT.pit.ratio), IRp = LIT.pit.IR_FC + (LIT.pit.IR_MER - LIT.pit.IR_FC) * sigma;
  return { UR: URg * (1 - sigma) + URp * sigma, IR: IRg * (1 - sigma) + IRp * sigma, PT };
}

/* ===== 綁定姿態座標框 ===== */
let BIND = {};
function bindInit(man) {
  BIND = {};
  man.segs.forEach((s, i) => {
    BIND[s.n] = { i, Ob: s.Ob, Fx: s.Fb[0], Fy: s.Fb[1], Fz: s.Fb[2] };
  });
}
function bindLoc(seg, p) {
  const b = BIND[seg], d = V.sub(p, b.Ob);
  return [V.dot(d, b.Fx), V.dot(d, b.Fy), V.dot(d, b.Fz)];
}

/* ===== 足部預處理（動態規劃求踝角與著地朝向鎖定） ===== */
function getMTP(meta, S) {
  const m = meta && meta[S] && meta[S].mtp;
  return m ? { x: m[0], y: m[1] } : { x: 0.095, y: -0.055 };
}

function footZ(tib, th) {
  const s = tib.y[2], wt = 1 - smooth01(0.45, 0.8, Math.abs(s));
  const zh = [Math.cos(th), Math.sin(th), 0];
  const v = V.add(V.mul(zh, 1 - wt), V.mul(tib.z, wt));
  return V.len(v) > 0.3 ? V.norm(v) : zh;
}

function footPre(fr, T, ctx) {
  const N = fr.length, out = {};
  const thp = new Float64Array(N);
  {
    let pv = 0;
    for (let i = 0; i < N; i++) {
      const a = fr[i].lm.rear_hip, b = fr[i].lm.lead_hip;
      let t = Math.atan2(b[1] - a[1], b[0] - a[0]);
      while (t - pv > Math.PI) t -= 2 * Math.PI;
      while (t - pv < -Math.PI) t += 2 * Math.PI;
      thp[i] = t; pv = t;
    }
  }
  let offS = 0, offC = 0, offW = 0;
  const raw = {};
  for (const S of ['R', 'L']) {
    const th = new Float64Array(N), r = new Float64Array(N);
    let prev = 0;
    for (let i = 0; i < N; i++) {
      const q = limbs(fr[i].lm, T)[S];
      const u = V.norm(V.sub(q.knee, q.hip)), w = V.norm(V.sub(q.ankle, q.knee));
      let n = V.cross(u, w); const sn = V.len(n); n = V.mul(V.norm(n), -1);
      const hz = Math.hypot(n[0], n[1]);
      let a = Math.atan2(n[1], n[0]);
      while (a - prev > Math.PI) a -= 2 * Math.PI;
      while (a - prev < -Math.PI) a += 2 * Math.PI;
      th[i] = a; prev = a;
      r[i] = smooth01(0.12, 0.4, sn) * smooth01(0.35, 0.7, hz);
      const g = r[i] * r[i] * r[i];
      offS += g * Math.sin(a - thp[i]); offC += g * Math.cos(a - thp[i]); offW += g;
    }
    raw[S] = { th, r };
  }
  const off = offW > 1 ? Math.atan2(offS, offC) : 0;
  for (const S of ['R', 'L']) {
    const th = raw[S].th, r = raw[S].r, wf = new Float64Array(N);
    for (let i = 0; i < N; i++) {
      const q = limbs(fr[i].lm, T)[S];
      const gz = ctx.mz ? ctx.mz(q.ankle[0], q.ankle[1]) + ctx.fg[S].L0 * ctx.sc.csf : ctx.aMin[S];
      wf[i] = Math.max(0, Math.min(1, 1 - (q.ankle[2] - gz) / 0.12));
    }
    const sg = 8, ths = new Float64Array(N);
    for (let i = 0; i < N; i++) {
      let sw = 0, sa = 0;
      for (let j = Math.max(0, i - 3 * sg); j <= Math.min(N - 1, i + 3 * sg); j++) {
        const g = Math.exp(-((j - i) * (j - i)) / (2 * sg * sg)) * r[j] * r[j];
        sw += g; sa += g * th[j];
      }
      let pr = thp[i] + off;
      pr += 2 * Math.PI * Math.round(((sw > 1e-6 ? sa / sw : pr) - pr) / (2 * Math.PI));
      ths[i] = (sa + 0.4 * pr) / (sw + 0.4);
    }
    let w1 = Array.from(wf);
    for (let k = 0; k < 3; k++) {
      const w2 = w1.slice();
      for (let i = 1; i < N - 1; i++) w2[i] = 0.25 * w1[i - 1] + 0.5 * w1[i] + 0.25 * w1[i + 1];
      w1 = w2;
    }
    const hh = new Float64Array(N);
    for (let i = 0; i < N; i++) {
      const q = limbs(fr[i].lm, T)[S];
      hh[i] = q.ankle[2] - (ctx.mz ? ctx.mz(q.ankle[0], q.ankle[1]) + ctx.fg[S].L0 * ctx.sc.csf : ctx.aMin[S]);
    }
    const cm = new Uint8Array(N);
    { let on = false; for (let i = 0; i < N; i++) { if (!on && hh[i] < 0.05) on = true; else if (on && hh[i] > 0.09) on = false; cm[i] = on ? 1 : 0; } }
    const yc = new Float64Array(N), has = new Uint8Array(N);
    for (let i = 0; i < N;) {
      if (!cm[i]) { i++; continue; }
      let j = i; while (j < N && cm[j]) j++;
      if (j - i >= 8) {
        let sw = 0, sa = 0, s0 = 0, a0 = 0;
        for (let k = i; k < j; k++) { a0 += ths[k]; s0++; if (k < i + 36) { const g = r[k] * r[k]; sw += g; sa += g * th[k]; } }
        const c = sw > 0.5 ? sa / sw : a0 / s0;
        for (let k = Math.max(0, i - 12); k < Math.min(N, j + 12); k++) { yc[k] = c; has[k] = 1; }
      }
      i = j;
    }
    let cw = new Float64Array(N);
    for (let i = 0; i < N; i++) cw[i] = cm[i] && has[i] ? 1 : 0;
    for (let k = 0; k < 5; k++) {
      const c2 = Float64Array.from(cw);
      for (let i = 1; i < N - 1; i++) c2[i] = 0.25 * cw[i - 1] + 0.5 * cw[i] + 0.25 * cw[i + 1];
      cw = c2;
    }
    const thf = new Float64Array(N), wff = new Array(N);
    for (let i = 0; i < N; i++) {
      let d = has[i] ? yc[i] - ths[i] : 0;
      d -= 2 * Math.PI * Math.round(d / (2 * Math.PI));
      thf[i] = ths[i] + cw[i] * d;
      wff[i] = Math.max(w1[i], cw[i]);
    }
    const cs = ctx.sc.csf, G = ctx.fg[S], GMIN = -1.1, GMAX = 1.5, DG = 0.05, K = Math.round((GMAX - GMIN) / DG) + 1, REST = 0.35;
    const pts = G.n3, np = pts.length, tps = G.tp.filter((_, j) => j % 3 === 0), nt = tps.length;
    const wc = new Float64Array(N);
    for (let i = 0; i < N; i++) wc[i] = 1 - smooth01(0.05, 0.11, hh[i]);
    const cost = new Float64Array(N * K);
    for (let i = 0; i < N; i++) {
      const q = limbs(fr[i].lm, T)[S], tb = ctx.tibF[S][i], yk = tb.y, zf = footZ(tb, thf[i]);
      const B = frm(yk, zf);
      for (let k = 0; k < K; k++) {
        const a = GMIN + k * DG, c = Math.cos(a), sn = Math.sin(a);
        const fx = [B.x[0] * c - B.y[0] * sn, B.x[1] * c - B.y[1] * sn, B.x[2] * c - B.y[2] * sn];
        const fy = [B.y[0] * c + B.x[0] * sn, B.y[1] * c + B.x[1] * sn, B.y[2] * c + B.x[2] * sn];
        const fz = B.z;
        let m = 1e9;
        for (let j = 0; j < np; j++) {
          const p = pts[j];
          const x = q.ankle[0] + cs * (fx[0] * p[0] + fy[0] * p[1] + fz[0] * p[2]);
          const y = q.ankle[1] + cs * (fx[1] * p[0] + fy[1] * p[1] + fz[1] * p[2]);
          const z = q.ankle[2] + cs * (fx[2] * p[0] + fy[2] * p[1] + fz[2] * p[2]);
          const d = z - ctx.mz(x, y); if (d < m) m = d;
        }
        let mt = 1e9;
        for (let j = 0; j < nt; j++) {
          const p = tps[j];
          const x = q.ankle[0] + cs * (fx[0] * p[0] + fy[0] * p[1] + fz[0] * p[2]);
          const y = q.ankle[1] + cs * (fx[1] * p[0] + fy[1] * p[1] + fz[1] * p[2]);
          const z = q.ankle[2] + cs * (fx[2] * p[0] + fy[2] * p[1] + fz[2] * p[2]);
          const d = z - ctx.mz(x, y); if (d < mt) mt = d;
        }
        const dt = Math.max(0, -mt), pen = Math.max(0, -m - 0.003), lift = Math.max(0, m);
        cost[i * K + k] = 8 * (dt / 0.05) ** 2 + 10 * (pen / 0.005) ** 2 + wc[i] * (lift / 0.015) ** 2 + (1 - wc[i]) * 8 * (a - REST) ** 2;
      }
    }
    const JM = 2, LAMB = 0.5, ACC = new Float64Array(N * K), BP = new Int16Array(N * K);
    for (let k = 0; k < K; k++) ACC[k] = cost[k];
    for (let i = 1; i < N; i++) for (let k = 0; k < K; k++) {
      let best = 1e18, bj = k;
      for (let j = Math.max(0, k - JM); j <= Math.min(K - 1, k + JM); j++) {
        const d = (k - j) * DG / 0.05;
        const v = ACC[(i - 1) * K + j] + LAMB * d * d;
        if (v < best) { best = v; bj = j; }
      }
      ACC[i * K + k] = best + cost[i * K + k]; BP[i * K + k] = bj;
    }
    const psi = new Float64Array(N);
    {
      let kb = 0, bv = 1e18;
      for (let k = 0; k < K; k++) if (ACC[(N - 1) * K + k] < bv) { bv = ACC[(N - 1) * K + k]; kb = k; }
      for (let i = N - 1; i >= 0; i--) { psi[i] = GMIN + kb * DG; if (i > 0) kb = BP[i * K + kb]; }
    }
    const clrAt = (i, a) => {
      const q = limbs(fr[i].lm, T)[S], tb = ctx.tibF[S][i], yk = tb.y, zf = footZ(tb, thf[i]);
      const B = frm(yk, zf), c = Math.cos(a), sn = Math.sin(a);
      const fx = [B.x[0] * c - B.y[0] * sn, B.x[1] * c - B.y[1] * sn, B.x[2] * c - B.y[2] * sn];
      const fy = [B.y[0] * c + B.x[0] * sn, B.y[1] * c + B.x[1] * sn, B.y[2] * c + B.x[2] * sn];
      const fz = B.z;
      let m = 1e9;
      for (const p of pts) {
        const x = q.ankle[0] + cs * (fx[0] * p[0] + fy[0] * p[1] + fz[0] * p[2]);
        const y = q.ankle[1] + cs * (fx[1] * p[0] + fy[1] * p[1] + fz[1] * p[2]);
        const z = q.ankle[2] + cs * (fx[2] * p[0] + fy[2] * p[1] + fz[2] * p[2]);
        const d = z - ctx.mz(x, y); if (d < m) m = d;
      }
      return m;
    };
    for (let it = 0; it < 6; it++) {
      const p2 = Float64Array.from(psi);
      for (let i = 1; i < N - 1; i++) {
        const a = 0.25 * psi[i - 1] + 0.5 * psi[i] + 0.25 * psi[i + 1];
        if (Math.abs(a - psi[i]) < 1e-5) continue;
        const c0 = clrAt(i, psi[i]);
        if (clrAt(i, a) >= Math.min(c0, -0.003) - 1e-4) p2[i] = a;
      }
      psi.set(p2);
    }
    out[S] = { th: thf, wf: wff, psi };
  }
  return out;
}

/* ===== 脊柱逐節比例 ===== */
const SPN_R = {
  fe: [0.132, 0.155, 0.150, 0.160, 0.118, 0.037, 0.039, 0.039, 0.030, 0.019, 0.017, 0.015, 0.015, 0.009, 0.016, 0.024, 0.028],
  lb: [0.037, 0.082, 0.104, 0.101, 0.081, 0.067, 0.073, 0.057, 0.050, 0.045, 0.051, 0.041, 0.036, 0.038, 0.046, 0.045, 0.046],
  ar: [0.039, 0.040, 0.040, 0.038, 0.036, 0.020, 0.035, 0.060, 0.073, 0.085, 0.087, 0.079, 0.078, 0.074, 0.070, 0.074, 0.072]
};
const CUM = {};
for (const k in SPN_R) {
  const r = SPN_R[k], t = r.reduce((a, b) => a + b, 0);
  let c = 0; CUM[k] = r.map(x => (c += x / t));
}
const SPN_CI = { lumbar4: 0, lumbar3: 1, lumbar2: 2, lumbar1: 3, lumbar0: 4 };
for (let k = 11; k >= 1; k--) SPN_CI['thoracic' + k] = 5 + (11 - k);

/* ===== 髖點校正（HIP_K = 1.5） ===== */
const HIP_K = 1.5;
function pelvisBasis(hr, hl, tilt) {
  const zR = V.norm(V.sub(hr, hl));
  const y0 = orth(UP, zR);
  const yP = rotAbout(y0, zR, -tilt * D2R);
  const zP = orth(zR, yP);
  return [zP, V.cross(yP, zP), yP];
}
function thighBasis(hip, knee, ank, zP) {
  const zT = V.norm(V.sub(hip, knee)), u = V.norm(V.sub(knee, hip)), w = V.norm(V.sub(ank, knee));
  const n0 = V.cross(u, w), sn = V.len(n0), n = V.mul(V.norm(n0), -1);
  const wk = smooth01(0.12, 0.4, sn);
  const zk = orth(V.add(V.mul(n, wk), V.mul(zP, 1 - wk)), zT);
  return [zk, V.cross(zT, zk), zT];
}
function cardanXYZ(P, T) {
  const d = (i, j) => V.dot(P[i], T[j]);
  return [
    Math.atan2(-d(1, 2), d(2, 2)) / D2R,
    Math.asin(Math.max(-1, Math.min(1, d(0, 2)))) / D2R,
    Math.atan2(-d(0, 1), d(0, 0)) / D2R
  ];
}
function kneeFlex(h, k, a) {
  return 180 - Math.acos(Math.max(-1, Math.min(1, V.dot(V.norm(V.sub(h, k)), V.norm(V.sub(a, k)))))) / D2R;
}
const angWrap = d => ((d + 540) % 360 + 360) % 360 - 180;

function correctHips(fr, T) {
  const SG = { R: [-1, -1, -1], L: [-1, 1, 1] };
  for (const f of fr) {
    if (f.lm0) continue;
    const lm = f.lm, ja = f.ja;
    const hrO = T === 'R' ? lm.rear_hip : lm.lead_hip, hlO = T === 'R' ? lm.lead_hip : lm.rear_hip;
    const tilt = isFinite(ja.pelvis_angle_x) ? ja.pelvis_angle_x : 0;
    const Pb = pelvisBasis(hrO, hlO, tilt);
    const Dd = V.len(V.sub(hrO, hlO)) / 0.72, mh = V.mul(V.add(hrO, hlO), 0.5);
    const M = V.add(mh, V.add(V.mul(Pb[1], 0.19 * Dd), V.mul(Pb[2], 0.30 * Dd)));
    const shrink = h => V.add(M, V.mul(V.sub(h, M), 1 / HIP_K));
    const hrN = shrink(hrO), hlN = shrink(hlO), Pn = pelvisBasis(hrN, hlN, tilt);
    f.lm0 = { rear_hip: lm.rear_hip, lead_hip: lm.lead_hip };
    const jd = {};
    for (const S of ['R', 'L']) {
      const h0 = S === 'R' ? hrO : hlO, h1 = S === 'R' ? hrN : hlN, rl = S === T ? 'rear' : 'lead';
      const kn = lm[rl + '_knee_jc'], an = lm[rl + '_ankle_jc'];
      const a0 = cardanXYZ(Pb, thighBasis(h0, kn, an, Pb[0])), a1 = cardanXYZ(Pn, thighBasis(h1, kn, an, Pn[0]));
      jd[rl + '_hip_angle_x'] = SG[S][0] * angWrap(a1[0] - a0[0]);
      jd[rl + '_hip_angle_y'] = SG[S][1] * angWrap(a1[1] - a0[1]);
      jd[rl + '_knee_angle_x'] = kneeFlex(h1, kn, an) - kneeFlex(h0, kn, an);
    }
    lm.rear_hip = T === 'R' ? hrN : hlN;
    lm.lead_hip = T === 'R' ? hlN : hrN;
    f.jd = jd;
    const jc = Object.assign({}, ja);
    for (const k in jd) if (isFinite(jc[k])) jc[k] += jd[k];
    f.jc = jc;
  }
}

/* ===== 實測軸向角（髖 X-Y-Z Cardan / 肩 Z-Y-Z Euler） ===== */
function hipAxialZ(pel, ja, rl, S, long) {
  if (!ja) return null;
  const a = ja[rl + '_hip_angle_x'], b = ja[rl + '_hip_angle_y'], c = ja[rl + '_hip_angle_z'];
  if (!isFinite(a) || !isFinite(b) || !isFinite(c)) return null;
  const sg = S === 'R' ? [-1, -1, -1] : [-1, 1, 1];
  const R = M3.mul(M3.mul(M3.Rx(sg[0] * a * D2R), M3.Ry(sg[1] * b * D2R)), M3.Rz(sg[2] * c * D2R));
  const col0 = V.add(V.add(V.mul(pel.z, R[0][0]), V.mul(pel.x, R[1][0])), V.mul(pel.y, R[2][0]));
  return orth(col0, long);
}

function shoulderAxialZ(th, ja, pref, S, long) {
  if (!ja) return null;
  const a = ja[pref + '_angle_x'], b = ja[pref + '_angle_y'], c = ja[pref + '_angle_z'];
  if (!isFinite(a) || !isFinite(b) || !isFinite(c)) return null;
  const sg = S === 'R' ? -1 : 1;
  const R = M3.mul(M3.mul(M3.Rz(sg * a * D2R), M3.Ry(sg * b * D2R)), M3.Rz(sg * c * D2R));
  const col0 = V.add(V.add(V.mul(th.z, R[0][0]), V.mul(th.x, R[1][0])), V.mul(th.y, R[2][0]));
  return orth(col0, long);
}

function pronSigns(meta) {
  const o = {};
  for (const S of ['R', 'L']) {
    const m = meta[S], t = m.thumb_loc, a = m.axis_loc;
    const R1 = rotMat(a, Math.PI / 2), R2 = rotMat(a, -Math.PI / 2);
    const ap = R => R[0][0] * t[0] + R[0][1] * t[1] + R[0][2] * t[2];
    o[S] = ap(R1) > ap(R2) ? 1 : -1;
  }
  return o;
}

/* ===== 單幀完整骨骼姿態求解 ===== */
function pose(lm, ctx, ja, fi) {
  const { T, sc, aMin, fwd, segInfo, E, meta, neutral } = ctx;
  const l = limbs(lm, T);
  const out = {};
  const Pd = lm.thorax_dist, Pp = lm.thorax_prox;
  const yT = V.norm(V.sub(Pp, Pd));
  let zr = V.norm(V.sub(l.R.sh, l.L.sh));
  if (lm.thorax_ap) {
    const za = orth(V.sub(lm.thorax_ap, Pp), yT);
    zr = V.dot(za, zr) >= 0 ? za : V.mul(za, -1);
  }
  const th = frm(yT, zr);
  const mh = V.mul(V.add(l.R.hip, l.L.hip), 0.5);
  const zP = V.norm(V.sub(l.R.hip, l.L.hip));
  const tilt = ja && isFinite(ja.pelvis_angle_x) ? ja.pelvis_angle_x : 0.5 * Math.acos(Math.max(-1, Math.min(1, V.dot(yT, UP)))) * 180 / Math.PI;
  const y0 = orth(UP, zP), yP = rotAbout(y0, zP, -tilt * D2R);
  const pel = { y: yP }; pel.z = orth(zP, yP); pel.x = V.cross(pel.y, pel.z);
  const sT = sc.th, cs = sc.cs, csf = sc.csf;

  out.thorax = { O: Pd, F: th, S: [sT, sT, sT] };
  out.pelvis = { O: mh, F: pel, S: [sc.pel, sc.pel, Math.max(0.88 * sc.pel, Math.min(1.2 * sc.pel, sc.hipw))] };
  const mc = segInfo.meshC7;
  out.head = { O: V.add(Pd, V.add(V.add(V.mul(th.x, mc[0] * sT), V.mul(th.y, mc[1] * sT)), V.mul(th.z, mc[2] * sT))), F: null, S: [sT, sT, sT] };
  {
    const xt = th.x, xh = orth(fwd, th.y);
    const ang = Math.max(-1.3, Math.min(1.3, Math.atan2(V.dot(V.cross(xt, xh), th.y), V.dot(xt, xh))));
    const x2 = rotAbout(xt, th.y, ang);
    out.head.F = { x: x2, y: th.y, z: V.cross(x2, th.y) };
  }

  // ---- 連續三次 Hermite 脊柱（L5 → T2） ----
  {
    const Rr = m3t(m3(pel), m3(th));
    const l_ = Math.asin(Math.max(-1, Math.min(1, Rr[2][1])));
    const a_ = Math.atan2(-Rr[2][0], Rr[2][2]);
    const f_ = Math.atan2(-Rr[0][1], Rr[1][1]);
    const Rp = m3(pel);
    const place = (name, ci) => m3f(m3m(Rp, m3m(m3m(rz(CUM.fe[ci] * f_), rx(CUM.lb[ci] * l_)), ry(CUM.ar[ci] * a_))));
    const SP = segInfo.spn;
    const loc = (n, v) => { const Fb = SP.Fb[n]; return [0, 1, 2].map(k => Fb[k][0] * v[0] + Fb[k][1] * v[1] + Fb[k][2] * v[2]); };
    const wv = (sg, lv) => V.add(V.add(V.mul(sg.F.x, sg.S[0] * lv[0]), V.mul(sg.F.y, sg.S[1] * lv[1])), V.mul(sg.F.z, sg.S[2] * lv[2]));
    const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    const CHAIN = ['lumbar4', 'lumbar3', 'lumbar2', 'lumbar1', 'lumbar0', 'thoracic11', 'thoracic10', 'thoracic9', 'thoracic8', 'thoracic7', 'thoracic6', 'thoracic5', 'thoracic4', 'thoracic3', 'thoracic2', 'thoracic1'];
    const pb = nm => SP.C[nm] || SP.Ob[nm];
    if (!SP._H) {
      const p0b = pb('lumbar4'), p1b = pb('thoracic1'), spb = V.len(sub3(p1b, p0b));
      const m0b = V.mul(SP.Fb.pelvis[1], spb), m1b = V.mul(SP.Fb.thorax[1], spb);
      const U = CHAIN.map(nm => (pb(nm)[1] - p0b[1]) / (p1b[1] - p0b[1]));
      const off = CHAIN.map((nm, i) => {
        const u = U[i], u2 = u * u, u3 = u2 * u;
        const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
        const hb = V.add(V.add(V.mul(p0b, h00), V.mul(m0b, h10)), V.add(V.mul(p1b, h01), V.mul(m1b, h11)));
        const d = loc(nm, sub3(pb(nm), hb));
        return [d[0], d[1], 0];
      });
      SP._H = { U, off, spb };
    }
    const { U, off, spb } = SP._H;
    const pc0 = V.add(out.pelvis.O, wv(out.pelvis, loc('pelvis', sub3(pb('lumbar4'), SP.Ob.pelvis))));
    const pTop = V.add(out.thorax.O, wv(out.thorax, loc('thorax', sub3(pb('thoracic1'), SP.Ob.thorax))));
    const span = V.len(V.sub(pTop, pc0)), m0 = V.mul(pel.y, span), m1 = V.mul(th.y, span);
    for (let i = 0; i < CHAIN.length; i++) {
      const nm = CHAIN[i], u = U[i], u2 = u * u, u3 = u2 * u;
      const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
      const dh00 = 6 * u2 - 6 * u, dh10 = 3 * u2 - 4 * u + 1, dh01 = -6 * u2 + 6 * u, dh11 = 3 * u2 - 2 * u;
      const posH = V.add(V.add(V.mul(pc0, h00), V.mul(m0, h10)), V.add(V.mul(pTop, h01), V.mul(m1, h11)));
      const tg = V.norm(V.add(V.add(V.mul(pc0, dh00), V.mul(m0, dh10)), V.add(V.mul(pTop, dh01), V.mul(m1, dh11))));
      const F0 = place(nm, SPN_CI[nm]);
      const ny = V.dot(F0.y, tg) > 0.5 ? tg : F0.y, nz = orth(F0.z, ny);
      const F = { x: V.cross(ny, nz), y: ny, z: nz };
      const sg = { O: null, F, S: [sT, sT, sT] };
      const pc = V.add(posH, wv(sg, off[i]));
      sg.O = V.sub(pc, wv(sg, loc(nm, sub3(pb(nm), SP.Ob[nm]))));
      out[nm] = sg;
    }
    out.lumbar4.k = span / (spb * sT);
  }

  const sigma = neutral || !E ? 0 : smooth01(E.fp100, E.mer, fi);
  const ribEll = meta.rib_ell || { c: [0.008, 0.145, 0], r: [0.118, 0.185, 0.145] };

  for (const S of ['R', 'L']) {
    const q = l[S], s = sc[S], isT = S === T, mS = meta[S];
    const clampS = (v, m) => Math.max(m * 0.85, Math.min(m * 1.15, v));
    // ---- 下肢（Z-Anatomy 精準股骨髁中心樞軸 + 逐幀精準骨長：零關節裂縫 + 髕骨嚴格貼合股骨滑車弧面） ----
    const u = V.norm(V.sub(q.knee, q.hip)), w = V.norm(V.sub(q.ankle, q.knee));
    let n = V.cross(u, w); const sn = V.len(n); n = V.mul(V.norm(n), -1);
    const wk = smooth01(0.12, 0.4, sn);
    const femY = V.norm(V.sub(q.hip, q.knee));
    const zk = hipAxialZ(pel, ja, isT ? 'rear' : 'lead', S, femY) || orth(V.add(V.mul(n, wk), V.mul(pel.z, 1 - wk)), femY);
    const fem = frm(femY, zk);
    const sFf = neutral ? s.F : clampS(V.len(V.sub(q.hip, q.knee)) / mS.lenF, s.F);
    out[S + '_femur'] = { O: q.hip, F: fem, S: [cs, sFf, cs] };

    const tibY = V.norm(V.sub(q.knee, q.ankle));
    const tib = frm(tibY, zk);
    const sTf = neutral ? s.T : clampS(V.len(V.sub(q.knee, q.ankle)) / mS.lenT, s.T);
    out[S + '_tibia'] = { O: q.knee, F: tib, S: [cs, sTf, cs] };

    // 髕骨：綁定原點 Ob = knee（股骨髁旋轉中心），隨膝屈曲沿股骨滑車弧面滑動 0.58 * kneeFlex（絕不浮空、絕不穿骨）
    const flexSign = V.dot(V.cross(fem.y, tib.y), fem.z) >= 0 ? -1 : 1;
    const kneeFlexRad = Math.acos(Math.max(-1, Math.min(1, V.dot(fem.y, tib.y))));
    const patF = frameMul(fem, M3.Rz(flexSign * 0.58 * kneeFlexRad));
    out[S + '_patella'] = { O: q.knee, F: patF, S: [cs, cs, cs] };

    // ---- 足部與趾骨背屈避讓 ----
    let wf, zf = tib.z;
    if (!neutral && ctx.fpre) {
      const fp = ctx.fpre[S], a = fp.th[fi]; wf = fp.wf[fi];
      zf = [Math.cos(a), Math.sin(a), 0];
    } else {
      const gz = aMin[S]; wf = Math.max(0, Math.min(1, 1 - (q.ankle[2] - gz) / 0.12));
    }
    const yf = V.norm(V.add(V.mul(UP, wf), V.mul(tib.y, 1 - wf)));
    let ff = frm(yf, zf);
    const rotF = (F0, al) => {
      if (Math.abs(al) <= 1e-3) return F0;
      const c = Math.cos(al), sn2 = Math.sin(al);
      return { x: V.add(V.mul(F0.x, c), V.mul(F0.y, -sn2)), y: V.add(V.mul(F0.y, c), V.mul(F0.x, sn2)), z: F0.z };
    };
    if (!neutral && ctx.fpre) ff = rotF(frm(tib.y, footZ(tib, ctx.fpre[S].th[fi])), ctx.fpre[S].psi[fi]);
    const mtp = getMTP(meta, S);
    if (!neutral) {
      const fpS = ctx.fpre ? ctx.fpre[S] : null;
      let bt = 0;
      if (fpS && ctx.mz && ctx.fg[S].tp && ctx.fg[S].tp.length) {
        if (ctx.btS && ctx.btS[S]) bt = ctx.btS[S][fi];
        else {
          const tp = ctx.fg[S].tp;
          const clrT = b => {
            const c = Math.cos(b), sn2 = Math.sin(b); let m = 1e9;
            for (const p of tp) {
              const dx = p[0] - mtp.x, dy = p[1] - mtp.y; if (dx < 0.015) continue;
              const lx = mtp.x + c * dx - sn2 * dy, ly = mtp.y + sn2 * dx + c * dy, lz = p[2];
              const x = q.ankle[0] + csf * (ff.x[0] * lx + ff.y[0] * ly + ff.z[0] * lz);
              const y = q.ankle[1] + csf * (ff.x[1] * lx + ff.y[1] * ly + ff.z[1] * lz);
              const z = q.ankle[2] + csf * (ff.x[2] * lx + ff.y[2] * ly + ff.z[2] * lz);
              const d = z - ctx.mz(x, y); if (d < m) m = d;
            }
            return m;
          };
          if (clrT(0) < -0.003) {
            const solve = (sg, mx) => {
              let lo = 0, found = null;
              for (let b = 0.05; b <= mx; b += 0.05) { if (clrT(sg * b) >= -0.003) { found = b; break; } lo = b; }
              if (found === null) return null;
              let l2 = lo, h2 = found;
              for (let k = 0; k < 8; k++) { const mid = (l2 + h2) / 2; if (clrT(sg * mid) >= -0.003) h2 = mid; else l2 = mid; }
              return h2;
            };
            const bp = solve(1, 0.9), bn = solve(-1, 0.7);
            if (bp === null && bn === null) bt = clrT(0.9) >= clrT(-0.7) ? 0.9 : -0.7;
            else if (bn === null || (bp !== null && bp <= bn)) bt = bp;
            else bt = -bn;
          }
        }
        out[S + '_btRaw'] = bt;
      }
    }
    out[S + '_foot'] = { O: q.ankle, F: ff, S: [csf, csf, csf] };
    out[S + '_toeBend'] = (!neutral && ctx.fpre) ? (out[S + '_btRaw'] || 0) : 0;

    // ---- 上肢：實測肩角軸向 + 肘屈曲 + 逐幀精準骨長 ----
    const ua = V.norm(V.sub(q.el, q.sh)), wa = V.norm(V.sub(q.wr, q.el));
    let na = V.cross(ua, wa); const sa = V.len(na); na = V.norm(na);
    const we = smooth01(0.12, 0.4, sa), humY = V.norm(V.sub(q.sh, q.el));
    const za = shoulderAxialZ(th, ja, isT ? 'shoulder' : 'glove_shoulder', S, humY) || V.add(V.mul(na, we), V.mul(th.z, 1 - we));
    const hu = frm(humY, za);
    const sHf = neutral ? s.H : clampS(V.len(V.sub(q.sh, q.el)) / mS.lenH, s.H);
    out[S + '_humerus'] = { O: q.sh, F: hu, S: [cs, sHf, cs] };
    const ul = frm(V.norm(V.sub(q.el, q.wr)), hu.z);
    const sAf = neutral ? s.A : clampS(V.len(V.sub(q.el, q.wr)) / mS.lenA, s.A);
    out[S + '_ulna'] = { O: q.el, F: ul, S: [cs, sAf, cs] };
    out[S + '_elbowFlex'] = Math.acos(Math.max(-1, Math.min(1, V.dot(ua, wa)))) * 180 / Math.PI;

    // ---- 肩胛骨：2-DOF 肩胛-胸廓橢球貼合約束（使用 Z-Anatomy 精測 TS/AI 與肋籠橢球） ----
    const ay = V.dot(ua, th.y);
    const phi = Math.acos(Math.max(-1, Math.min(1, -ay))) / D2R;
    const sg = isT ? sigma : 0;
    const A = scapAngles(phi, sg);
    let irS = A.IR, ptS = A.PT;
    if (!neutral) {
      const tsL = mS.scap_ts || [-0.067, -0.025, S === 'R' ? -0.105 : 0.105];
      const aiL = mS.scap_ai || [-0.050, -0.133, S === 'R' ? -0.071 : 0.071];
      const shTh = [V.dot(V.sub(q.sh, Pd), th.x) / sT, V.dot(V.sub(q.sh, Pd), th.y) / sT, V.dot(V.sub(q.sh, Pd), th.z) / sT];
      const rc = ribEll.c, rr = ribEll.r;
      const ell = p => ((p[0] - rc[0]) / rr[0]) ** 2 + ((p[1] - rc[1]) / rr[1]) ** 2 + ((p[2] - rc[2]) / rr[2]) ** 2;
      const mv = (R, v) => [R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2], R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2], R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]];
      const ev = (ir, pt) => {
        let Rm = M3.mul(M3.mul(M3.Ry(ir * D2R), M3.Rx(-A.UR * D2R)), M3.Rz(pt * D2R));
        if (S === 'L') Rm = M3.mirrorZ(Rm);
        return [ell(V.add(shTh, mv(Rm, tsL))) - 1.05, ell(V.add(shTh, mv(Rm, aiL))) - 1.05];
      };
      for (let it = 0; it < 8; it++) {
        const [r1, r2] = ev(irS, ptS);
        if (Math.abs(r1) < 1e-3 && Math.abs(r2) < 1e-3) break;
        const eps = 0.5, [r1i, r2i] = ev(irS + eps, ptS), [r1p, r2p] = ev(irS, ptS + eps);
        const j11 = (r1i - r1) / eps, j12 = (r1p - r1) / eps, j21 = (r2i - r2) / eps, j22 = (r2p - r2) / eps, lam = 0.002;
        const a11 = j11 * j11 + j21 * j21 + lam, a12 = j11 * j12 + j21 * j22, a22 = j12 * j12 + j22 * j22 + lam;
        const b1 = -(j11 * r1 + j21 * r2), b2 = -(j12 * r1 + j22 * r2), det = a11 * a22 - a12 * a12 || 1e-9;
        irS = Math.max(-55, Math.min(55, irS + (a22 * b1 - a12 * b2) / det));
        ptS = Math.max(-30, Math.min(45, ptS + (a11 * b2 - a12 * b1) / det));
      }
    }
    let dR = M3.mul(M3.mul(M3.Ry(irS * D2R), M3.Rx(-A.UR * D2R)), M3.Rz(ptS * D2R));
    if (S === 'L') dR = M3.mirrorZ(dR);
    out[S + '_scap'] = { O: q.sh, F: frameMul(th, dR), S: [sT, sT, sT] };
    out[S + '_scapAng'] = { phi, UR: A.UR, IR: irS, PT: ptS };

    // ---- 鎖骨（SC → AC，以 th.y 為參考軸避免共線奇異） ----
    const SCw = toWorld(out.thorax, bindLoc('thorax', mS.SC));
    const ACw = toWorld(out[S + '_scap'], bindLoc(S + '_scap', mS.AC));
    const dv = V.sub(ACw, SCw), dl = V.len(dv) || 1e-6;
    const cy = V.mul(dv, 1 / dl), cx = orth(th.y, cy);
    out[S + '_clav'] = { O: SCw, F: { x: cx, y: cy, z: V.cross(cx, cy) }, S: [sT, Math.max(0.8, Math.min(1.25, dl / mS.lenC)), sT] };

    // ---- 前臂旋前（橈骨繞 Hr→Hu 軸旋轉） ----
    const manzi = neutral ? -90 : (isT ? pronManzi(fi, E) : LIT.pron.glove);
    const dth = (90 + manzi) * D2R;
    const rF = frameMul(ul, rotMat(mS.axis_loc, ctx.pronSign[S] * dth));
    out[S + '_radius'] = { O: toWorld(out[S + '_ulna'], bindLoc(S + '_ulna', mS.Hr)), F: rF, S: [cs, sAf, cs] };
    out[S + '_pron'] = manzi;

    // ---- 手（與足部相同，採等比縮放 csf，避免手套側標記點偏移把手指拉長變形） ----
    const wrW = toWorld(out[S + '_radius'], bindLoc(S + '_radius', mS.wr));
    out[S + '_hand'] = { O: wrW, F: frm(V.norm(V.sub(q.wr, q.hd)), rF.z), S: [csf, csf, csf] };
    out[S + '_wristErr'] = V.len(V.sub(wrW, q.wr));
  }
  return out;
}

/* ===== 中立解剖姿勢（供 L0 計算） ===== */
function neutralLm(M, sc, T) {
  const b2l = (X, Y, Z) => [X, -Z, Y];
  const lm = {}, hw = sc.rawW.hipw / 2, mh = [0, 1.0, 0];
  const place = (S, sgn) => {
    const m = M[S], s = sc[S];
    const hip = [0, mh[1], sgn * hw];
    const knee = [0, hip[1] - m.lenF * s.F, sgn * hw];
    const ankle = [0, knee[1] - m.lenT * s.T, sgn * hw];
    return { hip, knee, ankle };
  };
  const R = place('R', 1), L = place('L', -1);
  const Pd = [0, mh[1] + (sc.lum || 0.282 * sc.th), 0], Pp = [0, Pd[1] + M.thlen * sc.th, 0];
  const shy = Pd[1] + 0.230 * sc.th, shz = sc.shw ? sc.shw / 2 : 0.165 * sc.th;
  const arm = (S, sgn) => {
    const m = M[S], s = sc[S];
    const sh = [-0.017 * sc.th, shy, sgn * shz];
    const el = [sh[0], sh[1] - m.lenH * s.H, sh[2] + sgn * 0.01];
    const wr = [el[0], el[1] - m.lenA * s.A, el[2]];
    const hd = [wr[0], wr[1] - m.lenW * s.W, wr[2]];
    return { sh, el, wr, hd };
  };
  const AR = arm('R', 1), AL = arm('L', -1);
  const set = (name, p) => lm[name] = b2l(p[0], p[1], p[2]);
  const rear = T === 'R' ? R : L, lead = T === 'R' ? L : R, thr = T === 'R' ? AR : AL, glv = T === 'R' ? AL : AR;
  set('rear_hip', rear.hip); set('rear_knee_jc', rear.knee); set('rear_ankle_jc', rear.ankle);
  set('lead_hip', lead.hip); set('lead_knee_jc', lead.knee); set('lead_ankle_jc', lead.ankle);
  set('shoulder_jc', thr.sh); set('elbow_jc', thr.el); set('wrist_jc', thr.wr); set('hand_jc', thr.hd);
  set('glove_shoulder_jc', glv.sh); set('glove_elbow_jc', glv.el); set('glove_wrist_jc', glv.wr); set('glove_hand_jc', glv.hd);
  set('thorax_dist', Pd); set('thorax_prox', Pp);
  return lm;
}

/* ===== 肢段蒙皮變換：輸出仿射矩陣 A + Scale-then-DQS 雙四元數 dq 與縮放矩陣 scM ===== */
function segMats(po, man) {
  const n = man.segs.length;
  const A = new Float64Array((n + 2) * 12);
  const dq = new Float64Array((n + 2) * 8);
  const scM = new Float64Array((n + 2) * 12);

  for (let i = 0; i < n; i++) {
    const s = man.segs[i], sg = po[s.n], b = BIND[s.n];
    const F = [sg.F.x, sg.F.y, sg.F.z], B = [b.Fx, b.Fy, b.Fz], S = sg.S;
    const o = i * 12;
    // 仿射矩陣 A = [L | t]
    for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) {
      A[o + r * 3 + k] = S[0] * F[0][r] * B[0][k] + S[1] * F[1][r] * B[1][k] + S[2] * F[2][r] * B[2][k];
    }
    for (let r = 0; r < 3; r++) {
      A[o + 9 + r] = sg.O[r] - (A[o + r * 3] * b.Ob[0] + A[o + r * 3 + 1] * b.Ob[1] + A[o + r * 3 + 2] * b.Ob[2]);
    }
    // 純剛體旋轉 R = F * B^T 與剛體平移 t = O - R * Ob → 雙四元數
    const R00 = F[0][0] * B[0][0] + F[1][0] * B[1][0] + F[2][0] * B[2][0];
    const R01 = F[0][0] * B[0][1] + F[1][0] * B[1][1] + F[2][0] * B[2][1];
    const R02 = F[0][0] * B[0][2] + F[1][0] * B[1][2] + F[2][0] * B[2][2];
    const R10 = F[0][1] * B[0][0] + F[1][1] * B[1][0] + F[2][1] * B[2][0];
    const R11 = F[0][1] * B[0][1] + F[1][1] * B[1][1] + F[2][1] * B[2][1];
    const R12 = F[0][1] * B[0][2] + F[1][1] * B[1][2] + F[2][1] * B[2][2];
    const R20 = F[0][2] * B[0][0] + F[1][2] * B[1][0] + F[2][2] * B[2][0];
    const R21 = F[0][2] * B[0][1] + F[1][2] * B[1][1] + F[2][2] * B[2][1];
    const R22 = F[0][2] * B[0][2] + F[1][2] * B[1][2] + F[2][2] * B[2][2];
    const tx = sg.O[0] - (R00 * b.Ob[0] + R01 * b.Ob[1] + R02 * b.Ob[2]);
    const ty = sg.O[1] - (R10 * b.Ob[0] + R11 * b.Ob[1] + R12 * b.Ob[2]);
    const tz = sg.O[2] - (R20 * b.Ob[0] + R21 * b.Ob[1] + R22 * b.Ob[2]);
    dqFromRT(R00, R01, R02, R10, R11, R12, R20, R21, R22, tx, ty, tz, dq, i * 8);

    // 綁定空間局部縮放 M_s = B^T * diag(S) * B，偏移 b_s = Ob - M_s * Ob（確保 w=1 時 DQS 與 A*v 誤差為 0.00 mm）
    for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) {
      scM[o + r * 3 + k] = S[0] * B[0][r] * B[0][k] + S[1] * B[1][r] * B[1][k] + S[2] * B[2][r] * B[2][k];
    }
    for (let r = 0; r < 3; r++) {
      scM[o + 9 + r] = b.Ob[r] - (scM[o + r * 3] * b.Ob[0] + scM[o + r * 3 + 1] * b.Ob[1] + scM[o + r * 3 + 2] * b.Ob[2]);
    }
  }

  // 腳趾虛擬肢段（n = R_toes, n+1 = L_toes）
  ['R', 'L'].forEach((S, h) => {
    const fi = BIND[S + '_foot'].i, o = fi * 12, ot = (n + h) * 12, b = BIND[S + '_foot'];
    const sg = po[S + '_foot'], mtp = getMTP(man.meta, S);
    const beta = po[S + '_toeBend'] || 0, c = Math.cos(beta), s = Math.sin(beta);
    const Rb = [[c, -s, 0], [s, c, 0], [0, 0, 1]], p = [mtp.x, mtp.y, 0];
    for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) {
      A[ot + r * 3 + k] = A[o + r * 3] * Rb[0][k] + A[o + r * 3 + 1] * Rb[1][k] + A[o + r * 3 + 2] * Rb[2][k];
    }
    const Rp = [0, 1, 2].map(r => Rb[r][0] * p[0] + Rb[r][1] * p[1] + Rb[r][2] * p[2]);
    const RO = [0, 1, 2].map(r => Rb[r][0] * b.Ob[0] + Rb[r][1] * b.Ob[1] + Rb[r][2] * b.Ob[2]);
    const cc = [p[0] - Rp[0] - RO[0], p[1] - Rp[1] - RO[1], p[2] - Rp[2] - RO[2]];
    const Ow = [0, 1, 2].map(r => A[o + 9 + r] + A[o + r * 3] * b.Ob[0] + A[o + r * 3 + 1] * b.Ob[1] + A[o + r * 3 + 2] * b.Ob[2]);
    for (let r = 0; r < 3; r++) A[ot + 9 + r] = Ow[r] + A[o + r * 3] * cc[0] + A[o + r * 3 + 1] * cc[1] + A[o + r * 3 + 2] * cc[2];
    // DQS for toes (uniform scale csf)
    const invS = 1 / (sg.S[0] || 1);
    const R00 = A[ot] * invS, R01 = A[ot + 1] * invS, R02 = A[ot + 2] * invS;
    const R10 = A[ot + 3] * invS, R11 = A[ot + 4] * invS, R12 = A[ot + 5] * invS;
    const R20 = A[ot + 6] * invS, R21 = A[ot + 7] * invS, R22 = A[ot + 8] * invS;
    const tx = A[ot + 9] + (A[ot] * b.Ob[0] + A[ot + 1] * b.Ob[1] + A[ot + 2] * b.Ob[2]) - (R00 * b.Ob[0] + R01 * b.Ob[1] + R02 * b.Ob[2]);
    const ty = A[ot + 10] + (A[ot + 3] * b.Ob[0] + A[ot + 4] * b.Ob[1] + A[ot + 5] * b.Ob[2]) - (R10 * b.Ob[0] + R11 * b.Ob[1] + R12 * b.Ob[2]);
    const tz = A[ot + 11] + (A[ot + 6] * b.Ob[0] + A[ot + 7] * b.Ob[1] + A[ot + 8] * b.Ob[2]) - (R20 * b.Ob[0] + R21 * b.Ob[1] + R22 * b.Ob[2]);
    dqFromRT(R00, R01, R02, R10, R11, R12, R20, R21, R22, tx, ty, tz, dq, (n + h) * 8);
    for (let k = 0; k < 12; k++) scM[ot + k] = scM[o + k];
  });

  return { A, dq, scM };
}
