/* ===== 計算層（與視覺 DQS 蒙皮解耦）：解剖 MTU 路徑長度、應變、速度、加速度 ===== */

function evalMtuPts(mtu, A, outPts) {
  const n = mtu.length;
  if (outPts) outPts.length = 0;
  let L = 0, px = 0, py = 0, pz = 0;
  for (let i = 0; i < n; i++) {
    const p = mtu[i];
    const o = p[0] * 12;
    const b = BIND[MAN.segs[p[0]].n];
    // p[1..3] are in segment bind-local coords -> convert to bind world pos first
    const wx0 = b.Ob[0] + b.Fx[0] * p[1] + b.Fy[0] * p[2] + b.Fz[0] * p[3];
    const wy0 = b.Ob[1] + b.Fx[1] * p[1] + b.Fy[1] * p[2] + b.Fz[1] * p[3];
    const wz0 = b.Ob[2] + b.Fx[2] * p[1] + b.Fy[2] * p[2] + b.Fz[2] * p[3];
    const x = A[o] * wx0 + A[o + 1] * wy0 + A[o + 2] * wz0 + A[o + 9];
    const y = A[o + 3] * wx0 + A[o + 4] * wy0 + A[o + 5] * wz0 + A[o + 10];
    const z = A[o + 6] * wx0 + A[o + 7] * wy0 + A[o + 8] * wz0 + A[o + 11];
    if (outPts) outPts.push([x, y, z]);
    if (i > 0) L += Math.hypot(x - px, y - py, z - pz);
    px = x; py = y; pz = z;
  }
  return L;
}

async function computeBio(pr) {
  const P = st.P, N = P.N, times = P.fr.map(f => f.t), pass = st.smooth;
  const bio = {};
  let ti = 0;
  if (!P.wraps) {
    P.wraps = P.poses.map(po => poseWraps(po, MAN.meta));
    P.wrapsN = poseWraps(P.pn, MAN.meta);
    initMeshWraps(TIS, P.pn, P.matsN, MAN.meta);
    initMeshWraps(TENDS, P.pn, P.matsN, MAN.meta);
  }
  for (const t of TIS) {
    const raw = new Float64Array(N);
    let L0;
    if (t.kind === 'm') {
      const mtu = t.m.mtu;
      for (let i = 0; i < N; i++) raw[i] = evalMtuPts(mtu, P.mats[i].A, null) * 100;
      L0 = evalMtuPts(mtu, P.matsN.A, null) * 100;
    } else {
      for (let i = 0; i < N; i++) raw[i] = uclLen(MAN.uclTab, P.poses[i][t.side + '_elbowFlex'])[t.tab] / 10;
      L0 = uclLen(MAN.uclTab, 0)[t.tab] / 10;
    }
    let rMin = Infinity, rMax = -Infinity;
    for (let i = 0; i < N; i++) { if (raw[i] < rMin) rMin = raw[i]; if (raw[i] > rMax) rMax = raw[i]; }
    const iso = (rMax - rMin) < 1e-3 || (t.kind === 'm' && t.rigid);
    if (iso) raw.fill(L0);
    const L = iso ? raw : binom(raw, Math.min(pass, 2));
    const v = iso ? new Float64Array(N) : binom(cdiff(L, times), pass);
    const a = iso ? new Float64Array(N) : binom(cdiff(v, times), pass);
    let mV = 0, mA = 0, mS = 0;
    for (let i = 0; i < N; i++) {
      mV = Math.max(mV, Math.abs(v[i]));
      mA = Math.max(mA, Math.abs(a[i]));
      mS = Math.max(mS, Math.abs((L[i] - L0) / (L0 || 1) * 100));
    }
    bio[t.id] = {
      L, v, a, L0, iso,
      mV: Math.max(mV, 1), mA: Math.max(mA, 20), mS: Math.max(mS, 3),
      min: Math.min(...L), max: Math.max(...L)
    };
    if (pr && (++ti % 32 === 0)) await pr(0.3 + 0.65 * ti / TIS.length, '計算肌肉 MTU 長度與速度 ' + ti + '/' + TIS.length);
  }
  P.bio = bio;
}

function sample(id, f) {
  const d = st.P.bio[id], N = d.L.length - 1;
  f = Math.max(0, Math.min(N, f));
  const i0 = Math.floor(f), i1 = Math.min(N, i0 + 1), u = f - i0;
  const g = a => a[i0] + (a[i1] - a[i0]) * u;
  const L = g(d.L);
  return { L, v: g(d.v), a: g(d.a), strain: (L - d.L0) / (d.L0 || 1) * 100, d };
}
