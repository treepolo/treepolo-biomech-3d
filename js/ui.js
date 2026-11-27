/* ===== UI、資料載入、圖表與啟動流程 ===== */

const $ = id => document.getElementById(id);
const GRP = { hip: '髖與大腿', knee: '膝伸屈', shank: '小腿與足', trunk: '軀幹', shoulder: '肩帶與肩', arm: '上臂', fore: '前臂與手', neck: '頸與頭', ucl: 'UCL' };
const SEGZH = { pelvis: '骨盆', thorax: '胸廓', head: '頭頸', femur: '股骨', tibia: '脛腓骨', patella: '髕骨', foot: '足', clav: '鎖骨', scap: '肩胛骨', humerus: '肱骨', ulna: '尺骨', radius: '橈骨', hand: '手' };
const segzh = n => {
  if (/^lumbar/.test(n)) return '腰椎 L' + (+n.slice(6) + 1);
  if (/^thoracic/.test(n)) return '胸椎 T' + (+n.slice(8) + 1);
  const m = n.match(/^([RL])_(.+)$/);
  return m ? (m[1] === 'R' ? '右' : '左') + (SEGZH[m[2]] || m[2]) : (SEGZH[n] || n);
};

let MAN, PD, BIN, MBIN, TBIN;
let MUS = {}, TIS = [], TENDS = [];

function ld(p, msg) {
  const b = $('ld-bar');
  if (b) {
    b.style.width = Math.max(0, Math.min(100, p)).toFixed(1) + '%';
    $('ld-pct').textContent = Math.round(p) + '%';
    if (msg) $('ld-msg').textContent = msg;
  }
  return new Promise(r => requestAnimationFrame(() => setTimeout(r, 0)));
}

const st = {
  grfComp: 'z', grfOn: true, abs: false, func: 'all', q: '', pair: 'one',
  skin: 'c', P: null, f: 0, playing: false, spd: 0.25, side: 'both', cam: 'side',
  color: 'vel', vis: 'all', group: 'hip', sel: null, smooth: 3, sort: 'default',
  tab: 'charts', boneOp: 1, musOp: 1, lit: true, showBone: true, showTend: true, dragging: false
};
try { const s = localStorage.getItem('skin2'); if (s) st.skin = s; } catch (_) {}

/* ===== 內建球資料 / CSV 匯入 ===== */
const LMN = ['rear_hip', 'lead_hip', 'rear_knee_jc', 'lead_knee_jc', 'rear_ankle_jc', 'lead_ankle_jc', 'shoulder_jc', 'elbow_jc', 'wrist_jc', 'hand_jc', 'glove_shoulder_jc', 'glove_elbow_jc', 'glove_wrist_jc', 'glove_hand_jc', 'thorax_prox', 'thorax_dist'];
const PITCHES = { ids: [] };
const LVZH = { college: '大學', milb: '小聯盟', independent: '獨立聯盟', high_school: '高中', pro: '職業' };

function embeddedFrames(k) {
  const p = PD.pitches[k || 0];
  const A = new Float32Array(b64buf(p.lm)), B = new Float32Array(b64buf(p.ja));
  const nl = PD.names.length * 3, nj = PD.jn.length, N = p.t.length, fr = [];
  for (let i = 0; i < N; i++) {
    const lm = {};
    PD.names.forEach((n, q) => lm[n] = [A[i * nl + q * 3], A[i * nl + q * 3 + 1], A[i * nl + q * 3 + 2]]);
    const ja = {};
    PD.jn.forEach((n, q) => ja[n] = B[i * nj + q]);
    fr.push({ t: p.t[i], lm, ja });
  }
  return { frames: fr, ev: p.ev, id: p.id, mph: p.mph, mass: p.mass, height: p.height, throws: p.throws, fp: new Float32Array(b64buf(p.fp)) };
}

function pitchOptions(extra) {
  const o = PD.pitches.slice().sort((a, b) => b.mph - a.mph).map((p, k) =>
    `<option value="${p.id}">#${k + 1} ${p.mph.toFixed(1)} mph · ${p.throws === 'L' ? '左' : '右'}投 · ${LVZH[p.level] || p.level}（${p.id}）</option>`
  );
  return o.concat((extra || []).filter(x => !PD.pitches.some(p => p.id === x)).map(x => `<option value="${x}">匯入 ${x}</option>`)).join('');
}

async function pickPitch(id) {
  const k = PD.pitches.findIndex(p => p.id === id);
  st.playing = false; $('txt-play').textContent = '播放';
  document.body.classList.add('loading');
  await ld(0, '載入 ' + id + '…');
  try {
    if (k >= 0) await loadPitch(embeddedFrames(k));
    else await loadRaw(id);
  } finally {
    document.body.classList.remove('loading');
  }
  $('pitch-select').value = id;
}

function parseCSV(text) {
  const L = text.replace(/\r/g, '').trim().split('\n');
  if (L.length < 2) return null;
  const h = L[0].split(',').map(s => s.trim()), rows = [];
  for (let i = 1; i < L.length; i++) {
    if (!L[i].trim()) continue;
    const c = L[i].split(','), o = {};
    for (let k = 0; k < h.length; k++) o[h[k]] = h[k] === 'session_pitch' ? (c[k] || '').trim() : (c[k] === '' || c[k] === undefined ? NaN : parseFloat(c[k]));
    rows.push(o);
  }
  return { headers: h, rows };
}

const raw = { lm: null, ja: null };
function handleFiles(files) {
  [...files].forEach(f => {
    const r = new FileReader();
    r.onload = () => {
      const p = parseCSV(r.result); if (!p) return;
      const h = p.headers;
      if (h.includes('rear_hip_x') && h.includes('thorax_prox_x')) { raw.lm = p; $('status-lm').textContent = '✓ landmarks.csv'; $('lbl-lm').classList.add('ok'); }
      else if (h.includes('pelvis_angle_x') && h.includes('rear_hip_angle_x')) { raw.ja = p; $('status-ja').textContent = '✓ joint_angles.csv'; $('lbl-ja').classList.add('ok'); }
      else { $('import-msg').textContent = '無法辨識欄位：' + f.name; return; }
      assemble();
    };
    r.readAsText(f);
  });
}

function assemble() {
  if (!raw.lm) { $('import-msg').textContent = '已載入 joint_angles，請再匯入 landmarks.csv'; return; }
  const ids = [...new Set(raw.lm.rows.map(r => r.session_pitch))];
  $('pitch-select').innerHTML = pitchOptions(ids);
  PITCHES.ids = ids; loadRaw(ids[0]);
}

async function loadRaw(id) {
  const rows = raw.lm.rows.filter(r => r.session_pitch === id && isFinite(r.time));
  rows.sort((a, b) => a.time - b.time);
  const jm = new Map();
  if (raw.ja) raw.ja.rows.forEach(r => { if (r.session_pitch === id) jm.set(Math.round(r.time * 1e4), r); });
  const need = LMN.filter(n => !(n + '_x' in rows[0]));
  if (need.length) { $('import-msg').textContent = 'landmarks 缺欄位：' + need.join(', '); return; }
  const jn = PD.jn;
  const fr = rows.map(r => {
    const lm = {};
    LMN.concat(['thorax_ap', 'centerofmass']).forEach(n => { if (n + '_x' in r) lm[n] = [r[n + '_x'], r[n + '_y'], r[n + '_z']]; });
    const j = jm.get(Math.round(r.time * 1e4)), ja = {};
    jn.forEach(n => ja[n] = j && n in j ? j[n] : NaN);
    return { t: r.time, lm, ja };
  });
  const r0 = rows[0];
  await loadPitch({ frames: fr, ev: { pkh: r0.pkh_time, fp_10: r0.fp_10_time, fp_100: r0.fp_100_time, MER: r0.MER_time, BR: r0.BR_time, MIR: r0.MIR_time }, id });
  $('import-msg').textContent = '已載入 ' + id + '（' + fr.length + ' 幀）';
}

/* ===== 建立組織清單 ===== */
function tissueFlags(m) {
  const k = [...new Set(m.ch.map(c => c.replace(/^[RL]_/, '')))];
  const has = x => k.includes(x);
  const lit = [], f = [];
  if (has('scap') || has('clav')) lit.push('肩胛骨/鎖骨：文獻估計');
  if ((has('radius') || has('hand')) && k.some(x => ['ulna', 'humerus', 'scap', 'clav', 'thorax'].includes(x) || /^thoracic/.test(x))) lit.push('前臂旋前：文獻估計');
  if (k.includes('pelvis') && k.some(x => x === 'thorax' || /^thoracic/.test(x))) f.push('脊柱逐節分攤：文獻比例');
  if (m.rigid || k.length <= 1) f.push('單一剛體：無長度變化');
  if (has('hand') && !lit.length) f.push('手指無運動資料');
  if (has('foot')) f.push('足/趾關節簡化');
  if (has('head') && k.length > 1) f.push('頭頸為單一剛體');
  return { lit, approx: f };
}

function buildTissueList() {
  TIS = [];
  for (const m of MAN.muscles) {
    if (m.base === 'external anal sphincter') continue;
    const fl = tissueFlags(m);
    TIS.push({
      id: m.id, key: m.base, name: (m.side === 'R' ? '右 ' : m.side === 'L' ? '左 ' : '') + m.zh,
      side: m.side, grp: m.grp, m, mu: MUS[m.id], kind: 'm',
      lit: fl.lit.length > 0, litTags: fl.lit, notes: fl.approx, rigid: !!m.rigid
    });
  }
  for (const S of ['R', 'L']) {
    for (const [k, n] of [['ant', 'UCL 前束-前帶'], ['post', 'UCL 前束-後帶'], ['pb', 'UCL 後束']]) {
      TIS.push({ id: S + '_ucl_' + k, key: 'ucl_' + k, name: (S === 'R' ? '右 ' : '左 ') + n, side: S, grp: 'ucl', kind: 'u', tab: k, lit: false, litTags: [], notes: [] });
    }
  }
  TENDS = [];
  for (const td of (MAN.tendons || [])) {
    const mu = unpackSkinnedMesh(td, TBIN);
    remapToesDQS(mu, MAN);
    TENDS.push({ id: td.id, key: td.base, name: td.zh, side: td.side, cat: td.cat, td, mu, rigid: td.ch && td.ch.length <= 1 });
  }
}

const role = S => S === 'C' ? 'center' : S === st.P.ctx.T ? 'throw' : 'glove';

async function loadPitch(d, opt) {
  st.busy = true;
  const F0 = opt ? opt.from : 0, F1 = opt ? opt.to : 100, pr = (f, m) => ld(F0 + (F1 - F0) * f, m);
  await pr(0, '分析動作資料…');
  const fr = d.frames, N = fr.length, evT = d.ev;
  const idx = t => { let b = 0, bd = 1e9; for (let i = 0; i < N; i++) { const dd = Math.abs(fr[i].t - t); if (dd < bd) { bd = dd; b = i; } } return b; };
  const E = { pkh: idx(evT.pkh), fp10: idx(evT.fp_10), fp100: idx(evT.fp_100), mer: idx(evT.MER), br: idx(evT.BR), mir: idx(evT.MIR) };
  const h = detectHand(fr, E.pkh, E.fp100);
  correctHips(fr, h.T);
  const sc = calcScales(fr, MAN.meta, h.T);
  const aMin = ankleMin(fr, h.T);
  const segInfo = { meshC7: MAN.meta.c7loc, spn: { C: {}, Ob: {}, Fb: {} } };
  MAN.segs.forEach(s => { segInfo.spn.Ob[s.n] = s.Ob; segInfo.spn.Fb[s.n] = s.Fb; if (s.C) segInfo.spn.C[s.n] = s.C; });
  const ps = pronSigns(MAN.meta);

  {
    const n = Math.max(3, Math.floor(E.pkh * 0.5));
    let a = 0, b = 0;
    for (let i = 0; i < n; i++) { a += fr[i].lm.rear_ankle_jc[2]; b += fr[i].lm.rear_ankle_jc[1]; }
    MD.dz = a / n - footParts(MAN.segs.find(x => x.n === h.T + '_foot')).L0 * sc.csf;
    MD.dy = b / n;
  }
  {
    const lead = h.T === 'R' ? 'L' : 'R';
    const L0 = footParts(MAN.segs.find(x => x.n === lead + '_foot')).L0 * sc.csf;
    const a0 = E.fp100, a1 = Math.max(a0 + 10, E.mer), zs = [];
    let xm = 0, c = 0;
    for (let i = a0; i <= Math.min(N - 1, a1); i++) { const p = fr[i].lm.lead_ankle_jc; zs.push(p[2] - L0); xm += p[0]; c++; }
    MD.slope = 1 / 12;
    if (c >= 8) {
      zs.sort((p, q) => p - q);
      const g = zs[Math.floor(0.2 * (zs.length - 1))]; xm /= c;
      const run = xm - MD.fl;
      if (run > 0.3) MD.slope = Math.max(1 / 22, Math.min(1 / 5, (MD.dz - g) / run));
    }
  }
  const fg = {};
  for (const S of ['R', 'L']) { const q = footParts(MAN.segs.find(x => x.n === S + '_foot')); fg[S] = { n3: q.n3, tp: q.tp, L0: q.L0 }; }
  const ctx = { fg, T: h.T, sc, aMin, fwd: h.fwd, segInfo, E, meta: MAN.meta, pronSign: ps, mz: moundZ };
  { const p0 = []; for (let i = 0; i < N; i++) p0.push(pose(fr[i].lm, ctx, fr[i].jc || fr[i].ja, i)); ctx.tibF = { R: p0.map(p => p.R_tibia.F), L: p0.map(p => p.L_tibia.F) }; }
  ctx.fpre = footPre(fr, h.T, ctx);
  let poses = [];
  for (let i = 0; i < N; i++) { poses.push(pose(fr[i].lm, ctx, fr[i].jc || fr[i].ja, i)); if (i % 80 === 79) await pr(0.05 + 0.2 * i / N, '計算骨骼姿態…'); }
  {
    ctx.btS = {};
    const BW = [0, 1, 2].map(k => Math.exp(-(k * k) / (2 * 1.44)));
    for (const S of ['R', 'L']) {
      const r = poses.map(p => p[S + '_btRaw']); if (r.some(x => x === undefined)) continue;
      const mx = r.map((_, i) => { let m = 0; for (let k = -2; k <= 2; k++) { const j = Math.max(0, Math.min(N - 1, i + k)); if (Math.abs(r[j]) > Math.abs(m)) m = r[j]; } return m; });
      ctx.btS[S] = mx.map((_, i) => { let s = 0, w = 0; for (let k = -2; k <= 2; k++) { const j = Math.max(0, Math.min(N - 1, i + k)), ww = BW[Math.abs(k)]; s += ww * mx[j]; w += ww; } return s / w; });
    }
    poses = []; for (let i = 0; i < N; i++) poses.push(pose(fr[i].lm, ctx, fr[i].jc || fr[i].ja, i));
  }
  const nl = neutralLm(MAN.meta, sc, h.T), nAnk = Math.min(nl.rear_ankle_jc[2], nl.lead_ankle_jc[2]);
  const pn = pose(nl, { T: h.T, sc, aMin: { R: nAnk, L: nAnk }, fwd: [1, 0, 0], segInfo, E: null, meta: MAN.meta, pronSign: ps, neutral: true }, null, 0);
  await pr(0.27, '計算 DQS 雙四元數變換…');
  const mats = poses.map(p => segMats(p, MAN));
  const matsN = segMats(pn, MAN);
  st.P = { d, fr, N, E, ctx, poses, pn, mats, matsN, id: d.id, fp: d.fp && d.fp.length === N * 18 ? d.fp : null, bw: d.mass ? d.mass * 9.80665 : null, mph: d.mph, hz: Math.round((N - 1) / (fr[N - 1].t - fr[0].t)), bio: {} };

  const err = {};
  for (const S of ['R', 'L']) {
    const m = MAN.meta[S];
    const mk = (a, seg, loc) => fr.reduce((s, f, i) => {
      const po = poses[i], q = limbs(f.lm, h.T)[S];
      const w = toWorld(po[S + '_' + seg], bindLoc(S + '_' + seg, m[loc]));
      return s + V.len(V.sub(w, q[a]));
    }, 0) / N * 1000;
    err[S] = { knee: mk('knee', 'femur', 'knee'), ankle: mk('ankle', 'tibia', 'ankle'), elbow: mk('el', 'humerus', 'el'), wrist: poses.reduce((s, p) => s + p[S + '_wristErr'], 0) / N * 1000 };
  }
  st.P.err = err;

  await computeBio(pr);
  await pr(0.92, '建立 3D 場景…');
  buildSceneForPitch();
  if (d.mph) $('import-msg').textContent = '內建 ' + d.id + '（' + d.mph.toFixed(1) + ' mph，' + N + ' 幀 360 Hz＋力板 1080 Hz）';
  if (st.sel && !TIS.find(t => t.id === st.sel)) st.sel = null;
  populateSelect(); refreshTblFuncs(); renderChips(); renderMarkers(); updateVerifyStatic(); updateEventBtns();
  await pr(0.98, '完成…');
  st.busy = false;
  setFrame(Math.min(st.f, N - 1));
}

/* ===== 組織篩選、著色、圖表與表格 ===== */
const FUNC_RULES = [[/gluteus maximus|semimembranosus|semitendinosus|long head of biceps femoris|adductor magnus/, '髖伸'], [/iliacus|psoas major|rectus femoris|sartorius|pectineus|tensor fasciae/, '髖屈'], [/gluteus medius|gluteus minimus|tensor fasciae|sartorius|piriformis/, '髖外展'], [/adductor|gracilis|pectineus/, '髖內收'], [/piriformis|obturator|gemellus|quadratus femoris|gluteus maximus/, '髖外旋'], [/vastus|rectus femoris/, '膝伸'], [/semimembranosus|semitendinosus|biceps femoris|gastrocnemius|popliteus|gracilis|sartorius|plantaris/, '膝屈'], [/gastrocnemius|soleus|plantaris|tibialis posterior|flexor (digitorum|hallucis) longus|fibularis (longus|brevis)/, '踝蹠屈'], [/tibialis anterior|extensor (digitorum|hallucis) longus|fibularis tertius/, '踝背屈'], [/ of foot|digitorum brevis|abductor hallucis|flexor accessorius|hallucis brevis|adductor hallucis/, '足內在肌'], [/oblique|rectus abdominis|transversus abdominis|pyramidalis|quadratus lumborum/, '軀幹屈曲/旋轉'], [/iliocostalis|longissimus|spinalis|multifidus|semispinalis|rotator|interspinales|intertransvers|serratus posterior|levatores costarum|quadratus lumborum/, '脊柱伸展/穩定'], [/diaphragm|intercostal|transversus thoracis|serratus posterior|levatores costarum/, '呼吸'], [/coccygeus|iliococcygeus|pubococcygeus|puborectalis|anal sphincter/, '骨盆底'], [/trapezius|rhomboid|levator scapulae|serratus anterior|pectoralis minor|subclavius/, '肩胛穩定'], [/deltoid|supraspinatus|coracobrachialis|clavicular part of pectoralis/, '肩上舉/外展'], [/latissimus|teres major|pectoralis major|spinal part of deltoid|coracobrachialis|long head of triceps/, '肩伸/內收'], [/subscapularis|pectoralis major|latissimus|teres major/, '肩內旋'], [/infraspinatus|teres minor|spinal part of deltoid/, '肩外旋'], [/supraspinatus|infraspinatus|teres minor|subscapularis/, '旋轉袖'], [/biceps brachii|(^|\s)brachialis|brachioradialis|pronator teres/, '肘屈'], [/triceps|anconeus/, '肘伸'], [/pronator/, '前臂旋前'], [/supinator|biceps brachii/, '前臂旋後'], [/flexor carpi|palmaris|abductor pollicis longus/, '腕屈'], [/extensor carpi|abductor pollicis longus/, '腕伸'], [/flexor digitorum (superficialis|profundus)|flexor pollicis longus|superficialis/, '手指屈'], [/extensor digitorum|extensor indicis|extensor digiti minimi|extensor pollicis/, '手指伸'], [/ of hand|pollicis brevis|opponens pollicis|adductor pollicis/, '手內在肌'], [/sternocleidomastoid|scalenus|splenius|semispinalis (capitis|cervicis)|longus colli|longus capitis|rectus capitis|obliquus capitis|spinalis cervicis|longissimus capitis|iliocostalis cervicis|cervical|interspinales cervicis/, '頸部運動'], [/hyoid|digastric|mylohyoid|geniohyoid/, '舌骨/吞嚥'], [/masseter|temporalis|pterygoid/, '咀嚼'], [/frontalis|occipitalis|temporoparietalis|orbicularis|zygomaticus|risorius|buccinator|mentalis|corrugator|procerus|nasalis|depressor|levator labii|levator anguli|platysma|depressor septi/, '表情']];

function funcOf(t) {
  if (t.kind === 'u') return ['肘外翻穩定'];
  const k = t.key || '', o = [];
  for (const [re, n] of FUNC_RULES) if (re.test(k) && !o.includes(n)) o.push(n);
  return o.length ? o : ['其他'];
}

function visible(t) {
  const sOK = st.side === 'both' || role(t.side) === 'center' || role(t.side) === st.side;
  if (!sOK) return false;
  if (t.lit && !st.lit && !selIds().includes(t.id)) return false;
  if (st.vis === 'all') return true;
  if (st.vis === 'group') return (t.grp === st.group || st.group === 'all') && (st.func === 'all' || funcOf(t).includes(st.func));
  return selIds().includes(t.id);
}

function colorVal(t, s) {
  const d = s.d;
  if (st.color === 'vel') return st.abs ? Math.abs(s.v) / d.mV : s.v / d.mV;
  if (st.color === 'acc') return st.abs ? Math.abs(s.a) / d.mA : s.a / d.mA;
  return s.strain / d.mS;
}

function selIds() {
  if (!st.sel) return [];
  const t = TIS.find(x => x.id === st.sel);
  if (!t) return [];
  const r = [t.id];
  if (st.side === 'both' && t.side !== 'C') {
    const o = TIS.find(x => x.key === t.key && x.side !== 'C' && x.side !== t.side);
    if (o) r.push(o.id);
  }
  if (r.length === 2 && TIS.find(x => x.id === r[0]).side !== st.P.ctx.T) r.reverse();
  return r;
}

function remapSel() {
  const t = TIS.find(x => x.id === st.sel);
  if (!t || t.side === 'C') return;
  const T = st.P.ctx.T, want = st.side === 'glove' ? (T === 'R' ? 'L' : 'R') : T;
  const o = TIS.find(x => x.key === t.key && x.side === want);
  if (o) st.sel = o.id;
}

function fmt(v, d = 1) { return (v >= 0 ? '' : '−') + Math.abs(v).toFixed(d); }
function phaseName(i) {
  const E = st.P.E;
  if (i < E.pkh) return '站立 / 抬腿起始';
  if (i < E.fp10) return '抬腿 → 跨步';
  if (i < E.fp100) return '落地';
  if (i < E.mer) return '加速（到最大外旋）';
  if (i < E.br) return '手臂加速（到出手）';
  if (i < E.mir) return '出手 → 最大內旋';
  return '跟隨動作';
}

function setFrame(f) { const N = st.P.N - 1; st.f = Math.max(0, Math.min(N, f)); updateFrame(); }

function updateFrame() {
  if (st.busy) return;
  const P = st.P, N = P.N - 1, i = Math.round(st.f);
  $('lbl-time').textContent = P.fr[i].t.toFixed(3);
  $('lbl-frame').textContent = i + '/' + N;
  $('hud-phase').textContent = phaseName(i);
  const u = st.f / N * 100;
  $('scrubber-fill').style.width = u + '%';
  $('scrubber-thumb').style.left = u + '%';
  updateScene(); followCam();
  {
    const ids = selIds(), S = ids.map(id => sample(id, st.f)), J = fn => S.length ? S.map(fn).join(' / ') : '—';
    $('kpi-L').textContent = J(s => s.L.toFixed(2));
    $('kpi-L0').textContent = J(s => s.d.L0.toFixed(1));
    $('kpi-str').textContent = J(s => (s.strain >= 0 ? '+' : '') + s.strain.toFixed(1) + '%');
    $('kpi-v').textContent = J(s => fmt(s.v, 1));
    $('kpi-v-state').textContent = S.length ? 'cm/s · ' + S.map(s => Math.abs(s.v) < 1 ? '等長' : s.v > 0 ? '離心拉長' : '向心縮短').join(' / ') : 'cm/s';
    $('kpi-a').textContent = J(s => fmt(s.a, 0));
    document.querySelector('.kpis').classList.toggle('two', S.length > 1);
  }
  {
    const ja = P.fr[i].jc || P.fr[i].ja;
    if (ja) {
      const f3 = p => [1, 2, 3].map(k => (ja[p + (k === 1 ? 'x' : k === 2 ? 'y' : 'z')] || 0).toFixed(1)).join('° / ') + '°';
      $('ja-pel').textContent = f3('pelvis_angle_');
      $('ja-rhip').textContent = f3('rear_hip_angle_');
      $('ja-lhip').textContent = f3('lead_hip_angle_');
      $('ja-knee').textContent = (ja.rear_knee_angle_x || 0).toFixed(1) + '° / ' + (ja.lead_knee_angle_x || 0).toFixed(1) + '°';
    }
  }
  drawCharts(); drawGrf();
  if (st.tab === 'table') updateTable();
  if (st.tab === 'verify') updateJA();
}

function drawSeries(cv, arr, o) {
  const r = cv.getBoundingClientRect(), dpr = Math.min(devicePixelRatio || 1, 2);
  const W = Math.max(10, r.width), H = Math.max(10, r.height);
  if (cv.width !== Math.round(W * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const c = cv.getContext('2d');
  c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, W, H);
  const kk = o.k || 1, N = arr.length - 1, padL = 6, padR = 6, padT = 18, padB = 6;
  let lo = Infinity, hi = -Infinity;
  for (const v of arr) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (o.arr2) for (const v of o.arr2) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (o.zero) { const m = Math.max(Math.abs(lo), Math.abs(hi), 1e-6); lo = -m; hi = m; }
  else if (o.ref != null) { lo = Math.min(lo, o.ref); hi = Math.max(hi, o.ref); }
  const minSp = o.minSpan || 0;
  if (hi - lo < minSp) { const mid = (lo + hi) / 2; lo = mid - minSp / 2; hi = mid + minSp / 2; }
  const pad = (hi - lo) * 0.08 || 1; lo -= pad; hi += pad;
  const X = i => padL + i / N * (W - padL - padR), Y = v => padT + (1 - (v - lo) / (hi - lo)) * (H - padT - padB);
  const line = cssv('--line'), ink2 = cssv('--ink2'), col = cssv(o.col);
  c.strokeStyle = line; c.lineWidth = 1;
  for (let k = 0; k < 4; k++) { const y = padT + k / 3 * (H - padT - padB); c.beginPath(); c.moveTo(padL, y); c.lineTo(W - padR, y); c.stroke(); }
  const E = st.P.E;
  c.font = '700 8px ui-monospace,monospace'; c.textAlign = 'left';
  [['PKH', E.pkh], ['FP', E.fp100], ['MER', E.mer], ['BR', E.br], ['MIR', E.mir]].forEach(([n, i]) => {
    c.strokeStyle = line; c.setLineDash([2, 3]); c.beginPath(); c.moveTo(X(i * kk), padT); c.lineTo(X(i * kk), H - padB); c.stroke();
    c.setLineDash([]); c.fillStyle = ink2; c.fillText(n, X(i * kk) + 2, H - padB - 2);
  });
  if (o.zero) { c.strokeStyle = ink2; c.globalAlpha = 0.6; c.beginPath(); c.moveTo(padL, Y(0)); c.lineTo(W - padR, Y(0)); c.stroke(); c.globalAlpha = 1; }
  if (o.ref != null) { c.strokeStyle = ink2; c.setLineDash([4, 3]); c.beginPath(); c.moveTo(padL, Y(o.ref)); c.lineTo(W - padR, Y(o.ref)); c.stroke(); c.setLineDash([]); }
  c.beginPath(); for (let i = 0; i <= N; i++) { const x = X(i), y = Y(arr[i]); i ? c.lineTo(x, y) : c.moveTo(x, y); }
  c.strokeStyle = col; c.lineWidth = 1.8; c.lineJoin = 'round'; c.stroke();
  if (o.arr2) {
    c.beginPath(); for (let i = 0; i <= N; i++) { const x = X(i), y = Y(o.arr2[i]); i ? c.lineTo(x, y) : c.moveTo(x, y); }
    c.strokeStyle = ink2; c.setLineDash([5, 3]); c.lineWidth = 1.6; c.stroke(); c.setLineDash([]);
  }
  const x = X(st.f * kk); c.strokeStyle = cssv('--ink'); c.lineWidth = 1; c.beginPath(); c.moveTo(x, padT - 4); c.lineTo(x, H - padB); c.stroke();
  const yv = Y(arr[Math.round(st.f * kk)]); c.fillStyle = col; c.beginPath(); c.arc(x, yv, 3.5, 0, 7); c.fill(); c.strokeStyle = cssv('--bg'); c.lineWidth = 1.5; c.stroke();
  c.fillStyle = cssv('--ink'); c.textAlign = 'right'; c.font = '700 10px ui-monospace,monospace';
  c.fillText((arr[Math.round(st.f * kk)]).toFixed(o.dec) + (o.arr2 ? ' / ' + o.arr2[Math.round(st.f * kk)].toFixed(o.dec) : ''), W - 8, 12);
}

function grfSeries() {
  const P = st.P; if (!P.fp) return null;
  const n = P.N * 3, key = st.grfComp + '|' + P.id;
  if (P._gk === key) return P._gs;
  const f = P.fp, bw = P.bw, R = new Float64Array(n), L = new Float64Array(n);
  const comp = (o, i) => { const x = f[i * 6 + o], y = f[i * 6 + o + 1], z = f[i * 6 + o + 2]; return st.grfComp === 'z' ? z : st.grfComp === 'x' ? x : st.grfComp === 'y' ? y : Math.hypot(x, y, z); };
  for (let i = 0; i < n; i++) { R[i] = comp(0, i) / bw * 100; L[i] = comp(3, i) / bw * 100; }
  P._gk = key; P._gs = { R, L }; return P._gs;
}
function grfAt(i3) {
  const f = st.P.fp, i = Math.max(0, Math.min(st.P.N * 3 - 1, Math.round(i3)));
  return [[f[i * 6], f[i * 6 + 1], f[i * 6 + 2]], [f[i * 6 + 3], f[i * 6 + 4], f[i * 6 + 5]]];
}
function drawCharts() {
  const ids = selIds(), ctx = ['cv-len', 'cv-vel', 'cv-acc'].map($);
  if (!ids.length) { ctx.forEach(cv => { const c = cv.getContext('2d'); c.setTransform(1, 0, 0, 1, 0, 0); c.clearRect(0, 0, cv.width, cv.height); }); return; }
  const d = st.P.bio[ids[0]], e = ids[1] ? st.P.bio[ids[1]] : null;
  drawSeries(ctx[0], d.L, { col: '--acc', ref: d.L0, dec: 2, minSpan: 0.5, arr2: e && e.L });
  drawSeries(ctx[1], d.v, { col: '--pos', zero: true, dec: 1, minSpan: 2, arr2: e && e.v });
  drawSeries(ctx[2], d.a, { col: '--warn', zero: true, dec: 0, minSpan: 40, arr2: e && e.a });
}
function drawGrf() {
  const g = grfSeries(), cv = $('cv-grf');
  $('grf-card').classList.toggle('hidden', !g); if (!g) return;
  drawSeries(cv, g.R, { col: '--acc', zero: st.grfComp !== 'z' && st.grfComp !== 'm', dec: 0, k: 3, arr2: g.L });
}

function pickList() {
  const T = st.P.ctx.T, q = (st.pq || '').trim().toLowerCase();
  return TIS.filter(t => (t.side === T || t.side === 'C' || (t.kind === 'u' && t.side === T)) && (st.group === 'all' || t.grp === st.group) && (st.func === 'all' || funcOf(t).includes(st.func)) && (!q || (t.name + ' ' + t.key).toLowerCase().includes(q)));
}
function fillPicker() {
  const sel = $('muscle-select'); if (!sel) return;
  const T = st.P.ctx.T, base = TIS.filter(t => (t.side === T || t.side === 'C') && (st.group === 'all' || t.grp === st.group));
  const fs = []; base.forEach(t => funcOf(t).forEach(f => { if (!fs.includes(f)) fs.push(f); }));
  if (st.func !== 'all' && !fs.includes(st.func)) st.func = 'all';
  $('tp-func').innerHTML = '<option value="all">全部功能（' + fs.length + '）</option>' + fs.map(f => `<option ${f === st.func ? 'selected' : ''}>${f}</option>`).join('');
  $('tp-region').value = st.group;
  const L = pickList();
  let h = '<option value="">— 不選擇 —（共 ' + L.length + ' 條）</option>';
  if (st.group === 'all' && !st.pq && st.func === 'all') {
    for (const gk of Object.keys(GRP)) {
      const sub = L.filter(t => t.grp === gk); if (!sub.length) continue;
      h += `<optgroup label="${GRP[gk]}（${sub.length}）">` + sub.map(t => `<option value="${t.id}">${t.name.replace(/^[右左] /, '')}${t.lit ? ' ◇' : ''}</option>`).join('') + '</optgroup>';
    }
  } else h += L.map(t => `<option value="${t.id}">${t.name.replace(/^[右左] /, '')}${t.lit ? ' ◇' : ''}</option>`).join('');
  sel.innerHTML = h;
  const t = TIS.find(x => x.id === st.sel);
  if (t) { const o = TIS.find(x => x.key === t.key && (x.side === T || x.side === 'C')) || t; sel.value = o.id; }
  const box = $('tp-res');
  if (st.pq) {
    box.innerHTML = L.slice(0, 8).map(t => `<button data-id="${t.id}">${t.name.replace(/^[右左] /, '')}<small>${GRP[t.grp]}</small></button>`).join('') || '<i>沒有符合的組織</i>';
    box.classList.remove('hidden');
    box.querySelectorAll('button').forEach(b => b.onclick = () => { st.pq = ''; $('tp-q').value = ''; selectT(b.dataset.id); fillPicker(); });
  } else box.classList.add('hidden');
}
function populateSelect() {
  st.pq = '';
  $('tp-region').innerHTML = '<option value="all">全部部位</option>' + Object.keys(GRP).map(k => `<option value="${k}">${GRP[k]}</option>`).join('');
  fillPicker(); renderInfo();
}
function renderInfo() {
  const t = TIS.find(x => x.id === st.sel);
  if (!t) {
    $('m-title').textContent = '（未選擇組織）'; $('m-tags').innerHTML = '';
    $('m-orig').textContent = '—'; $('m-ins').textContent = '—'; $('m-via').textContent = '—';
    $('m-note').textContent = '從上方選單或「組織總覽表」點選一條組織；選單第一項可取消選取。';
    return;
  }
  $('m-title').textContent = st.side === 'both' && t.side !== 'C' ? t.name.replace(/^[右左] /, '') + '（雙側）' : t.name + (t.side === 'C' ? '' : role(t.side) === 'throw' ? '（投球側）' : '（手套側）');
  const tags = [GRP[t.grp]]; if (t.kind === 'u') tags.push('查表長度');
  $('m-tags').innerHTML = tags.map(x => `<span class="tag">${x}</span>`).concat((t.litTags || []).map(x => `<span class="tag lit">${x}</span>`), (t.notes || []).map(x => `<span class="tag warn">${x}</span>`)).join('');
  const loc = p => `局部 (前${fmt(p[0] * 100)}, 上${fmt(p[1] * 100)}, 右${fmt(p[2] * 100)}) cm`;
  if (t.kind === 'm') {
    const m = t.m, mtu = m.mtu;
    const pA = mtu[0], pB = mtu[mtu.length - 1];
    $('m-orig').textContent = segzh(MAN.segs[pA[0]].n) + '｜' + loc(pA.slice(1));
    $('m-ins').textContent = segzh(MAN.segs[pB[0]].n) + '｜' + loc(pB.slice(1));
    const via = mtu.slice(1, -1).map(p => segzh(MAN.segs[p[0]].n));
    $('m-via').textContent = via.length ? via.join(' → ') : '（起止點直連；視覺採 DQS 雙四元數平滑蒙皮）';
    const hasT = m.nf > m.nfB;
    const lines = [`形狀：Z-Anatomy 真實解剖網格（${m.nv} 頂點 / ${m.nf} 面${hasT ? '，含真實解剖肌腱段' : ''}），Scale-then-DQS 雙四元數蒙皮；長度由獨立解耦的解剖 MTU 路徑計算。`];
    if (t.litTags.length) lines.push('◇ ' + t.litTags.join('；') + '：此長度含文獻估計的運動，不是實測。');
    if (t.rigid) lines.push('此肌肉兩端都在同一剛體上，本模型沒有該處的關節資料，所以長度不變。');
    $('m-note').textContent = lines.join(' ');
  } else {
    $('m-orig').textContent = '肱骨 內上髁（Z-Anatomy 網格量得）';
    $('m-ins').textContent = '尺骨 sublime tubercle（Z-Anatomy 網格量得）';
    $('m-via').textContent = '—';
    $('m-note').textContent = '長度＝本專案 UCL 3D 模型的查表值（含足印、無外翻負荷），隨肘屈曲角變化。';
  }
  $('m-via-row').classList.toggle('hidden', false);
}
function refreshTblFuncs() {
  const fs = []; TIS.filter(t => st.group === 'all' || t.grp === st.group).forEach(t => funcOf(t).forEach(f => { if (!fs.includes(f)) fs.push(f); }));
  if (st.func !== 'all' && !fs.includes(st.func)) st.func = 'all';
  $('tbl-func').innerHTML = '<option value="all">功能：全部（' + fs.length + '）</option>' + fs.map(x => `<option${x === st.func ? ' selected' : ''}>${x}</option>`).join('');
}
function syncSel() { refreshTblFuncs(); renderChips(); fillPicker(); updateTable(); updateFrame(); }
function renderChips() {
  const c = $('func-chips');
  c.innerHTML = `<button data-g="all" class="${st.group === 'all' ? 'on' : ''}">全部</button>` + Object.keys(GRP).map(k => `<button data-g="${k}" class="${st.group === k ? 'on' : ''}">${GRP[k]}</button>`).join('');
  c.querySelectorAll('button').forEach(b => b.onclick = () => { st.group = b.dataset.g; syncSel(); });
}
function updateTable() {
  const rows = TIS.filter(t => (st.side === 'both' || role(t.side) === 'center' || role(t.side) === st.side) && (st.group === 'all' || t.grp === st.group) && (st.func === 'all' || funcOf(t).includes(st.func)) && (!st.q || (t.name + ' ' + t.key).toLowerCase().includes(st.q)) && (st.lit || !t.lit || selIds().includes(t.id))).map(t => ({ t, s: sample(t.id, st.f) }));
  if (st.sort === 'strain') rows.sort((a, b) => Math.abs(b.s.strain) - Math.abs(a.s.strain));
  else if (st.sort === 'vel') rows.sort((a, b) => Math.abs(b.s.v) - Math.abs(a.s.v));
  else if (st.sort === 'acc') rows.sort((a, b) => Math.abs(b.s.a) - Math.abs(a.s.a));
  $('tbl-body').innerHTML = rows.map(({ t, s }) => {
    const hex = '#' + gradCol(colorVal(t, s)).getHexString();
    return `<tr data-id="${t.id}" class="${selIds().includes(t.id) ? 'sel' : ''}"><td><span class="dot" style="background:${hex}"></span>${t.name}</td><td>${s.L.toFixed(1)}</td><td>${(s.strain >= 0 ? '+' : '') + s.strain.toFixed(1)}</td><td>${fmt(s.v, 1)}</td><td>${fmt(s.a, 0)}</td></tr>`;
  }).join('');
  $('tbl-body').querySelectorAll('tr').forEach(r => r.onclick = () => selectT(r.dataset.id));
}
function selectT(id) { st.sel = id || null; remapSel(); syncPicker(); renderInfo(); updateFrame(); }
function syncPicker() {
  const s = $('muscle-select'); if (!s) return;
  const t = TIS.find(x => x.id === st.sel);
  let v = '';
  if (t) {
    const T = st.P.ctx.T, o = TIS.find(x => x.key === t.key && (x.side === T || x.side === 'C')) || t;
    v = o.id;
    if (!Array.from(s.options).some(q => q.value === v)) { st.group = 'all'; st.func = 'all'; syncSel(); }
  }
  s.value = v;
}
function renderMarkers() {
  const P = st.P, N = P.N - 1, E = P.E;
  $('scrubber-markers').innerHTML = [['PKH', E.pkh], ['FP10', E.fp10], ['FP100', E.fp100], ['MER', E.mer], ['BR', E.br], ['MIR', E.mir]].map(([n, i], k) => `<div class="mk" style="left:${i / N * 100}%"><i class="${k % 2 ? 'lo' : ''}">${n}</i></div>`).join('');
}
function updateEventBtns() { document.querySelectorAll('.ev').forEach(b => b.disabled = false); }

function updateVerifyStatic() {
  const P = st.P, sc = P.ctx.sc, m = MAN.meta, T = P.ctx.T, O = T === 'R' ? 'L' : 'R';
  $('v-pitch').textContent = P.id;
  $('v-hand').textContent = T === 'R' ? '右投 (RHP)' : '左投 (LHP)';
  $('v-hz').textContent = P.hz + ' Hz / ' + P.N + ' 幀';
  $('v-pw').textContent = (sc.rawW.hipw * 100).toFixed(1) + ' / ' + (m.hipw * 100).toFixed(1) + ' cm';
  const L = (S, k) => (m[S][k] * sc[S][{ lenF: 'F', lenT: 'T', lenH: 'H', lenA: 'A' }[k]] * 100).toFixed(1);
  $('v-rseg').textContent = L(T, 'lenF') + ' / ' + L(T, 'lenT') + ' cm';
  $('v-lseg').textContent = L(O, 'lenF') + ' / ' + L(O, 'lenT') + ' cm';
  $('v-rarm').textContent = L(T, 'lenH') + ' / ' + L(T, 'lenA') + ' cm';
  $('v-larm').textContent = L(O, 'lenH') + ' / ' + L(O, 'lenA') + ' cm';
  $('v-scales').innerHTML = [
    ['胸廓（整體）', sc.th], ['骨盆（寬 / 高深）', sc.hipw, sc.pel],
    ['投球側 股骨 / 脛骨', sc[T].F, sc[T].T], ['投球側 上臂 / 前臂 / 手', sc[T].H, sc[T].A, sc[T].W],
    ['手套側 股骨 / 脛骨', sc[O].F, sc[O].T], ['手套側 上臂 / 前臂 / 手', sc[O].H, sc[O].A, sc[O].W]
  ].map(r => `${r[0]}：<b>${r.slice(1).map(x => '×' + x.toFixed(2)).join(' / ')}</b>`).join('<br>');
  const e = P.err, fm = x => x.toFixed(1) + ' mm';
  $('v-err').innerHTML = ['R', 'L'].map(S => `${S === T ? '投球側' : '手套側'}：膝 <b>${fm(e[S].knee)}</b>｜踝 <b>${fm(e[S].ankle)}</b>｜肘 <b>${fm(e[S].elbow)}</b>｜腕 <b>${fm(e[S].wrist)}</b>`).join('<br>');
  $('v-notes').innerHTML =
    '<u>3D 解剖資產與演算法（v2.0）：</u>本版採用 <b>Z-Anatomy (CC BY-SA 4.0)</b> 高精解剖骨骼、肌肉（含每條肌肉原生解剖肌腱段、真實肩胛下肌與獨立髕骨）與獨立肌腱／腱膜／支持帶／韌帶（阿基里斯腱、髂脛束、胸腰筋膜、腹白線、腕踝支持帶、十字韌帶等）。視覺層全面升級為 <b>Scale-then-DQS 雙四元數蒙皮</b>（搭配骨面距離＋網格拉普拉斯平滑權重與保層次骨面包繞）；生物力學計算層完全解耦，由骨面起止點解剖 MTU 路徑獨立計算 L(t)、ΔL%、dL/dt 與 d²L/dt²。<br>' +
    '<u>髖點校正（估算）：</u>Driveline 的髖點由「ASIS 距離×1.5」算出；此處假設前後、左右、上下三個偏移都被放大 1.5 倍並除回。髖 x/y 與膝角依同一套座標軸前後重算的差值校正，髖 z 與 centerofmass 維持 Driveline 原值。<br>' +
    '<u>實測與文獻估計（◇）：</u>骨盆、胸廓、頭、肱骨（含實測肩角 Z-Y-Z 軸向）、股骨（含實測髖角 X-Y-Z 軸向）、脛骨、足、手來自 OBP 18 標記點與 25 條關節角；肩胛骨（2-DOF 肩胛-胸廓橢球貼合）、鎖骨與前臂旋前（Manzi 2023）為文獻估計。脊柱 T2–T12 與 L1–L5 採單一連續三次 Hermite 曲線；髕骨沿股骨滑車弧面隨膝屈曲滑動。<br>' +
    '<u>授權：</u>OBP CC BY-NC-SA 4.0（Driveline Baseball）；Z-Anatomy CC BY-SA 4.0（Gauthier Kervyn，衍生自 BodyParts3D CC BY-SA 2.1 JP, DBCLS）。';
}

function updateJA() {
  const j = st.P.fr[Math.round(st.f)].jc || st.P.fr[Math.round(st.f)].ja;
  const g = n => [j[n + '_x'], j[n + '_y'], j[n + '_z']].map(v => isFinite(v) ? v.toFixed(1) : '—').join(' / ');
  const rows = [
    ['pelvis_angle (X/Y/Z)', g('pelvis_angle')], ['torso_angle (X/Y/Z)', g('torso_angle')],
    ['torso_pelvis_angle', g('torso_pelvis_angle')], ['rear_hip_angle（x/y 校正，z 原值）', g('rear_hip_angle')],
    ['lead_hip_angle（x/y 校正，z 原值）', g('lead_hip_angle')], ['shoulder_angle', g('shoulder_angle')],
    ['膝 x（校正；軸心 / 前導）', (j.rear_knee_angle_x || 0).toFixed(1) + '° / ' + (j.lead_knee_angle_x || 0).toFixed(1) + '°'],
    ['肘 x（投球 / 手套）', (j.elbow_angle_x || 0).toFixed(1) + '° / ' + (j.glove_elbow_angle_x || 0).toFixed(1) + '°']
  ];
  $('ja-box').innerHTML = rows.map(r => `<div style="display:flex;justify-content:space-between;gap:8px"><span>${r[0]}</span><b>${r[1]}</b></div>`).join('');
}

function syncSeg(gid, key, val) { $(gid).querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset[key] === String(val))); }

function bind() {
  $('sel-vis').value = st.vis; document.body.dataset.skin = st.skin; syncSeg('skin-group', 'skin', st.skin);
  $('skin-group').querySelectorAll('button').forEach(b => b.onclick = () => {
    st.skin = b.dataset.skin; document.body.dataset.skin = st.skin; syncSeg('skin-group', 'skin', st.skin);
    try { localStorage.setItem('skin2', st.skin); } catch (_) {}
    applySkinScene(); setTimeout(() => { resize(); applyCam(false); updateFrame(); }, 30);
  });
  $('btn-play').onclick = () => { st.playing = !st.playing; $('txt-play').textContent = st.playing ? '暫停' : '播放'; if (st.playing && st.f >= st.P.N - 1.5) st.f = 0; };
  $('btn-prev').onclick = () => { st.playing = false; $('txt-play').textContent = '播放'; setFrame(Math.round(st.f) - 1); };
  $('btn-next').onclick = () => { st.playing = false; $('txt-play').textContent = '播放'; setFrame(Math.round(st.f) + 1); };
  $('spd-group').querySelectorAll('button').forEach(b => b.onclick = () => { st.spd = +b.dataset.spd; syncSeg('spd-group', 'spd', b.dataset.spd); });
  $('event-btns').querySelectorAll('button').forEach(b => b.onclick = () => {
    const E = st.P.E, i = { pkh: E.pkh, fp100: E.fp100, mer: E.mer, br: E.br, mir: E.mir }[b.dataset.ev];
    st.playing = false; $('txt-play').textContent = '播放'; setFrame(i);
  });
  $('side-group').querySelectorAll('button').forEach(b => b.onclick = () => { st.side = b.dataset.side; syncSeg('side-group', 'side', st.side); remapSel(); syncPicker(); renderInfo(); updateFrame(); });
  $('cam-group').querySelectorAll('button').forEach(b => b.onclick = () => { st.cam = b.dataset.cam; syncSeg('cam-group', 'cam', st.cam); applyCam(false); });
  $('bone-op').value = st.boneOp; $('lbl-bone-op').textContent = Math.round(st.boneOp * 100) + '%';
  $('mus-sc').value = st.musOp; $('lbl-mus-sc').textContent = Math.round(st.musOp * 100) + '%';
  $('chk-lit').checked = st.lit;
  if ($('chk-tend')) { $('chk-tend').checked = st.showTend; $('chk-tend').onchange = e => { st.showTend = e.target.checked; updateFrame(); }; }
  $('bone-op').oninput = e => { st.boneOp = +e.target.value; $('lbl-bone-op').textContent = Math.round(st.boneOp * 100) + '%'; for (const k in boneMeshes) { boneMeshes[k].material.opacity = st.boneOp; boneMeshes[k].material.depthWrite = st.boneOp > 0.95; } };
  $('mus-sc').oninput = e => { st.musOp = +e.target.value; $('lbl-mus-sc').textContent = Math.round(st.musOp * 100) + '%'; updateFrame(); };
  if ($('chk-lit')) $('chk-lit').onchange = e => { st.lit = e.target.checked; updateFrame(); };
  $('chk-skin').onchange = e => { st.showBone = e.target.checked; updateFrame(); };
  const updLegend = () => {
    const A = st.abs && st.color !== 'strain';
    const L = A ? { vel: ['小 |dL/dt|', '大 |dL/dt|'], acc: ['小 |d²L/dt²|', '大 |d²L/dt²|'] }[st.color] : { vel: ['向心縮短 (−)', '離心拉長 (+)'], strain: ['比中立短 (−)', '比中立長 (+)'], acc: ['減速 / 負加速 (−)', '加速 / 正加速 (+)'] }[st.color];
    $('legend-neg').textContent = L[0]; $('legend-pos').textContent = L[1];
    document.querySelector('.legend .bar').style.background = A ? 'linear-gradient(90deg,#64748b,#f59e0b,#ef4444)' : '';
    $('abs-wrap').style.display = st.color === 'strain' ? 'none' : '';
  };
  $('sel-color').onchange = e => { st.color = e.target.value; updLegend(); updateFrame(); };
  $('chk-abs').onchange = e => { st.abs = e.target.checked; updLegend(); updateFrame(); };
  updLegend();
  $('tp-region').onchange = e => { st.group = e.target.value; st.func = 'all'; syncSel(); };
  $('tp-func').onchange = e => { st.func = e.target.value; syncSel(); };
  $('tp-q').oninput = e => { st.pq = e.target.value; fillPicker(); };
  $('tbl-func').onchange = e => { st.func = e.target.value; syncSel(); };
  $('tbl-q').oninput = e => { st.q = e.target.value.trim().toLowerCase(); updateTable(); };
  $('grf-comp').onchange = e => { st.grfComp = e.target.value; drawGrf(); };
  $('chk-grf').onchange = e => { st.grfOn = e.target.checked; updateFrame(); };
  $('sel-vis').onchange = e => { st.vis = e.target.value; updateFrame(); };
  $('sel-smooth').onchange = e => { st.smooth = +e.target.value; computeBio().then(updateFrame); };
  $('muscle-select').onchange = e => selectT(e.target.value);
  $('sort-group').querySelectorAll('button').forEach(b => b.onclick = () => { st.sort = b.dataset.sort; syncSeg('sort-group', 'sort', st.sort); updateTable(); });
  $('tabs').querySelectorAll('button').forEach(b => b.onclick = () => { st.tab = b.dataset.tab; syncSeg('tabs', 'tab', st.tab); ['charts', 'table', 'verify'].forEach(k => $('tab-' + k).classList.toggle('hidden', k !== st.tab)); updateFrame(); });
  $('btn-fs').onclick = () => {
    const on = !document.body.classList.contains('fs3d');
    document.body.classList.toggle('fs3d', on); $('txt-fs').textContent = on ? '退出全螢幕' : '全螢幕 3D';
    if (on) { try { $('left-stage').requestFullscreen && $('left-stage').requestFullscreen(); } catch (_) {} }
    else { try { document.fullscreenElement && document.exitFullscreen(); } catch (_) {} }
    setTimeout(() => { resize(); updateFrame(); }, 60);
  };
  document.addEventListener('fullscreenchange', () => { if (!document.fullscreenElement && document.body.classList.contains('fs3d')) { document.body.classList.remove('fs3d'); $('txt-fs').textContent = '全螢幕 3D'; setTimeout(() => { resize(); updateFrame(); }, 60); } });
  $('pitch-select').onchange = e => pickPitch(e.target.value);
  ['file-both', 'file-lm', 'file-ja'].forEach(id => $(id).addEventListener('change', e => { handleFiles(e.target.files); e.target.value = ''; }));
  ['dragenter', 'dragover'].forEach(ev => document.addEventListener(ev, e => { e.preventDefault(); document.body.classList.add('drag-over'); }));
  ['dragleave', 'drop'].forEach(ev => document.addEventListener(ev, e => { e.preventDefault(); document.body.classList.remove('drag-over'); if (ev === 'drop' && e.dataTransfer) handleFiles(e.dataTransfer.files); }));
  scrub($('scrubber-track'), el => el.getBoundingClientRect());
  ['cv-len', 'cv-vel', 'cv-acc', 'cv-grf'].forEach(id => scrub($(id), el => { const r = el.getBoundingClientRect(); return { left: r.left + 6, width: r.width - 12 }; }));
  window.addEventListener('resize', () => { resize(); updateFrame(); });
}

function scrub(el, rect) {
  let on = false;
  const mv = e => {
    const r = rect(el), u = Math.max(0, Math.min(1, (e.clientX - r.left) / r.width));
    st.playing = false; $('txt-play').textContent = '播放'; setFrame(u * (st.P.N - 1));
  };
  el.addEventListener('pointerdown', e => { on = true; try { el.setPointerCapture(e.pointerId); } catch (_) {} mv(e); });
  el.addEventListener('pointermove', e => { if (on) mv(e); });
  ['pointerup', 'pointercancel'].forEach(ev => el.addEventListener(ev, () => on = false));
}

let last = 0;
function animate(ts) {
  requestAnimationFrame(animate);
  const dt = Math.min(0.05, (ts - last) / 1000 || 0); last = ts;
  if (st.playing && st.P && !st.busy) {
    st.f += dt * 360 * st.spd;
    if (st.f >= st.P.N - 1) st.f = 0;
    updateFrame();
    st._dirty = true;
  }
  if (controls && !st.busy) {
    const moved = controls.update();
    if (moved || st._dirty) {
      st._dirty = false;
      renderer.render(scene, camera);
    }
  }
}

async function boot() {
  await ld(2, '並行下載 Z-Anatomy 3D 資產與動捕資料…');
  const [manRes, pdRes, binRes, mbinRes, tbinRes] = await Promise.all([
    fetch('assets/manifest.json').then(r => { if (!r.ok) throw new Error('manifest.json ' + r.status); return r.json(); }),
    fetch('assets/pitches.json').then(r => { if (!r.ok) throw new Error('pitches.json ' + r.status); return r.json(); }),
    fetch('assets/bones.bin').then(r => { if (!r.ok) throw new Error('bones.bin ' + r.status); return r.arrayBuffer(); }),
    fetch('assets/muscles.bin').then(r => { if (!r.ok) throw new Error('muscles.bin ' + r.status); return r.arrayBuffer(); }),
    fetch('assets/tendons.bin').then(r => { if (!r.ok) throw new Error('tendons.bin ' + r.status); return r.arrayBuffer(); })
  ]);
  MAN = manRes; PD = pdRes; BIN = binRes; MBIN = mbinRes; TBIN = tbinRes;
  await ld(25, '初始化解剖骨骼與 DQS 肌肉網格…');
  bindInit(MAN);
  const ms = MAN.muscles;
  for (let i = 0; i < ms.length; i++) {
    const mu = unpackSkinnedMesh(ms[i], MBIN);
    remapToesDQS(mu, MAN);
    MUS[ms[i].id] = mu;
    if (i % 48 === 47) await ld(25 + 15 * i / ms.length, '解碼肌肉網格 ' + (i + 1) + '/' + ms.length);
  }
  buildTissueList();
  bind();
  initThree();
  $('pitch-select').innerHTML = pitchOptions();
  $('pitch-select').classList.remove('hidden');
  await loadPitch(embeddedFrames(0), { from: 42, to: 99 });
  $('pitch-select').value = PD.pitches[0].id;
  syncSeg('cam-group', 'cam', 'side');
  requestAnimationFrame(animate);
  await ld(100, '完成');
  document.body.classList.remove('loading', 'boot');
  window.__ready = true;
}

window.addEventListener('DOMContentLoaded', () => {
  boot().catch(e => { $('ld-msg').textContent = '載入失敗：' + e; console.error(e); });
});
