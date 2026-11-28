/* ===== Three.js 3D 場景：骨骼、DQS 肌肉（含真實解剖肌腱分色）、獨立肌腱/腱膜/韌帶、投手丘與力板向量 ===== */

let renderer, scene, camera, controls, root, ground, grid, arrow;
let boneMeshes = {}, tubes = {}, tendMeshes = {}, pins = [];
let mound = null, grfArrows = [];

const C3 = p => new THREE.Vector3(p[0], p[2], -p[1]);

class Orbit {
  constructor(cam, dom) {
    this.cam = cam; this.dom = dom;
    this.target = new THREE.Vector3(0, 1, 0);
    this.sph = new THREE.Spherical(3, Math.PI * 0.45, Math.PI / 2);
    this.tS = this.sph.clone(); this.tC = this.target.clone();
    this.ptr = new Map(); this.pd = 0; this.pm = { x: 0, y: 0 };
    this.mode = 'orbit'; this.onI = null; this._b();
  }
  setView(pos, look, inst) {
    this.tC.copy(look);
    this.tS.setFromVector3(pos.clone().sub(look));
    if (inst) { this.target.copy(look); this.sph.copy(this.tS); this._a(); }
    if (typeof st !== 'undefined') st._dirty = true;
  }
  pan(dx, dy) {
    const sc = this.tS.radius * 0.0017, f = new THREE.Vector3();
    this.cam.getWorldDirection(f);
    const r = new THREE.Vector3().crossVectors(f, this.cam.up).normalize();
    const u = new THREE.Vector3().crossVectors(r, f).normalize();
    this.tC.addScaledVector(r, -dx * sc).addScaledVector(u, dy * sc);
  }
  _b() {
    const d = this.dom;
    d.addEventListener('contextmenu', e => e.preventDefault());
    d.addEventListener('pointerdown', e => {
      try { d.setPointerCapture(e.pointerId); } catch (_) {}
      this.ptr.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.ptr.size === 1) this.mode = (e.button === 2 || e.shiftKey) ? 'pan' : 'orbit';
      else if (this.ptr.size === 2) {
        const p = [...this.ptr.values()];
        this.pd = Math.hypot(p[1].x - p[0].x, p[1].y - p[0].y);
        this.pm = { x: (p[0].x + p[1].x) / 2, y: (p[0].y + p[1].y) / 2 };
      }
      this.onI && this.onI();
    });
    d.addEventListener('pointermove', e => {
      if (!this.ptr.has(e.pointerId)) return;
      const pr = this.ptr.get(e.pointerId), dx = e.clientX - pr.x, dy = e.clientY - pr.y;
      this.ptr.set(e.pointerId, { x: e.clientX, y: e.clientY });
      if (this.ptr.size === 1) {
        if (this.mode === 'orbit') {
          this.tS.theta -= dx * 0.0075;
          this.tS.phi = Math.max(0.08, Math.min(Math.PI * 0.985, this.tS.phi - dy * 0.0075));
        } else this.pan(dx, dy);
      } else if (this.ptr.size === 2) {
        const p = [...this.ptr.values()], ds = Math.hypot(p[1].x - p[0].x, p[1].y - p[0].y);
        const m = { x: (p[0].x + p[1].x) / 2, y: (p[0].y + p[1].y) / 2 };
        if (this.pd > 1) this.tS.radius = Math.max(0.35, Math.min(9, this.tS.radius * (this.pd / Math.max(1, ds))));
        this.pan(m.x - this.pm.x, m.y - this.pm.y);
        this.pd = ds; this.pm = m;
      }
    });
    const end = e => { this.ptr.delete(e.pointerId); this.pd = 0; };
    d.addEventListener('pointerup', end); d.addEventListener('pointercancel', end);
    d.addEventListener('wheel', e => {
      e.preventDefault();
      this.tS.radius = Math.max(0.35, Math.min(9, this.tS.radius * (e.deltaY > 0 ? 1.09 : 0.91)));
      this.onI && this.onI();
    }, { passive: false });
  }
  update() {
    let dt = this.tS.theta - this.sph.theta;
    while (dt > Math.PI) dt -= 2 * Math.PI;
    while (dt < -Math.PI) dt += 2 * Math.PI;
    const dp = this.tS.phi - this.sph.phi, dr = this.tS.radius - this.sph.radius;
    const dc = this.target.distanceToSquared(this.tC);
    if (Math.abs(dt) < 1e-4 && Math.abs(dp) < 1e-4 && Math.abs(dr) < 1e-4 && dc < 1e-8) return false;
    this.sph.theta += dt * 0.16;
    this.sph.phi += dp * 0.16;
    this.sph.radius += dr * 0.16;
    this.target.lerp(this.tC, 0.2);
    this._a();
    return true;
  }
  _a() {
    this.cam.position.copy(this.target).add(new THREE.Vector3().setFromSpherical(this.sph));
    this.cam.lookAt(this.target);
  }
}

function cssv(n) { return getComputedStyle(document.body).getPropertyValue(n).trim(); }

function initThree() {
  const mount = $('three-mount');
  try { renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false }); }
  catch (e) { $('webgl-err-msg').textContent = String(e); $('webgl-err').classList.remove('hidden'); return false; }
  renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
  mount.appendChild(renderer.domElement);
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(36, 1, 0.05, 60);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.88));
  const d1 = new THREE.DirectionalLight(0xffffff, 0.92); d1.position.set(2, 4, 3); scene.add(d1);
  const d2 = new THREE.DirectionalLight(0xffffff, 0.38); d2.position.set(-3, 2, -2); scene.add(d2);
  root = new THREE.Group(); scene.add(root);
  controls = new Orbit(camera, renderer.domElement);
  controls.onI = () => { if (st.cam !== 'free') { st.cam = 'free'; syncSeg('cam-group', 'cam', 'free'); } };
  window.addEventListener('resize', resize);
  resize();
  return true;
}

function resize() {
  const b = $('viewport-box');
  if (!renderer) return;
  const w = b.clientWidth, h = b.clientHeight;
  if (w < 2 || h < 2) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  if (st.skin === 'c' && innerWidth > 900 && !document.body.classList.contains('fs3d')) camera.setViewOffset(w, h, 200, 64, w, h);
  else camera.clearViewOffset();
  camera.updateProjectionMatrix();
}

function applySkinScene() {
  if (!scene) return;
  scene.background = new THREE.Color(cssv('--scene-bg'));
  if (ground) ground.material.color.set(cssv('--ground'));
  if (grid) grid.material.color.set(cssv('--grid'));
  if (mound) mound.material.color.set(cssv('--mound') || '#8b5e3c');
  const bc = new THREE.Color(cssv('--bone'));
  for (const k in boneMeshes) boneMeshes[k].material.color.copy(bc);
}

const MD = { H: 0.254, xb: -0.90, fl: 0.228, hw: 0.75, slope: 1 / 12, dz: -0.045, dy: 0 };
function moundZ(x, y) {
  const d = MD.dz; y -= MD.dy;
  if (Math.abs(y) > MD.hw || x < MD.xb) return d - MD.H;
  const L = MD.fl + MD.H / MD.slope;
  if (x >= L) return d - MD.H;
  return d + (x <= MD.fl ? 0 : -(x - MD.fl) * MD.slope);
}

function buildMound() {
  const H = MD.H, xb = MD.xb, fl = MD.fl, xe = fl + H / MD.slope, w = MD.hw, d = MD.dz;
  const P = [[xb, d - H, -w], [xb, d, -w], [fl, d, -w], [xe, d - H, -w], [xb, d - H, w], [xb, d, w], [fl, d, w], [xe, d - H, w]];
  const pos = []; P.forEach(p => pos.push(p[0], p[1], -p[2]));
  const quad = (a, b, c, d2) => [a, b, c, a, c, d2];
  const idx = [].concat(quad(1, 2, 6, 5), quad(2, 3, 7, 6), quad(0, 4, 7, 3), quad(0, 1, 5, 4), [0, 2, 1, 0, 3, 2], [4, 5, 6, 4, 6, 7]);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx); g.computeVertexNormals();
  mound = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: new THREE.Color(cssv('--mound') || '#8b5e3c'), roughness: 1, metalness: 0, side: THREE.DoubleSide, flatShading: true }));
  root.add(mound); mound.position.z = -MD.dy;
  const rub = new THREE.Mesh(new THREE.BoxGeometry(0.152, 0.04, 0.61), new THREE.MeshStandardMaterial({ color: 0xf2f2f2, roughness: 0.6, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 }));
  rub.position.set(0, MD.dz - 0.012, -MD.dy); root.add(rub);
}

const _fp = {};
function footParts(s) {
  if (_fp[s.n]) return _fp[s.n];
  const iv = new Int16Array(BIN, s.off, s.nv * 3);
  const pos = new Float32Array(s.nv * 3);
  for (let i = 0; i < pos.length; i++) pos[i] = iv[i] * s.q;
  const fi = new Uint16Array(BIN, s.off + s.nv * 6, s.nf * 3);
  const nfF = s.nfF != null ? s.nfF : s.nf;
  const idxF = fi.slice(0, nfF * 3);
  const idxT = fi.slice(nfF * 3);
  const toeV = new Uint8Array(s.nv);
  for (let i = 0; i < idxT.length; i++) toeV[idxT[i]] = 1;
  let L0 = 0;
  for (let i = 0; i < s.nv; i++) L0 = Math.max(L0, -pos[i * 3 + 1]);
  const n3 = [], tp = [];
  for (let i = 0; i < s.nv; i += 4) if (!toeV[i]) n3.push([pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]]);
  for (let i = 0; i < s.nv; i += 2) if (toeV[i]) tp.push([pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]]);
  return _fp[s.n] = { pos, idxF, idxT, n3, tp, L0 };
}

function buildSceneForPitch() {
  if (!renderer) return;
  const P = st.P;
  while (root.children.length) {
    const c = root.children[0];
    root.remove(c);
    c.geometry && c.geometry.dispose();
  }
  boneMeshes = {}; tubes = {}; tendMeshes = {}; pins = [];
  const f = P.ctx.fwd;
  root.rotation.y = -Math.atan2(f[1], f[0]);
  const bc = new THREE.Color(cssv('--bone'));

  // 1. 剛體骨骼（含獨立 R/L_patella 與足趾分離）
  for (const s of MAN.segs) {
    const g = new THREE.BufferGeometry();
    const isFoot = s.n === 'R_foot' || s.n === 'L_foot';
    let pos, idx;
    if (isFoot) {
      const fp = footParts(s);
      pos = fp.pos; idx = fp.idxF;
    } else {
      const iv = new Int16Array(BIN, s.off, s.nv * 3);
      pos = new Float32Array(s.nv * 3);
      for (let i = 0; i < pos.length; i++) pos[i] = iv[i] * s.q;
      idx = new Uint16Array(BIN, s.off + s.nv * 6, s.nf * 3).slice();
    }
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setIndex(new THREE.BufferAttribute(idx, 1));
    g.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({ color: bc, roughness: 0.76, metalness: 0, transparent: true, opacity: st.boneOp, side: THREE.DoubleSide, depthWrite: st.boneOp > 0.95 });
    const m = new THREE.Mesh(g, mat);
    m.matrixAutoUpdate = false; m.renderOrder = 1; root.add(m);
    boneMeshes[s.n] = m;
    if (isFoot) {
      const fp = footParts(s);
      if (fp.idxT.length) {
        const g2 = new THREE.BufferGeometry();
        g2.setAttribute('position', new THREE.BufferAttribute(pos, 3));
        g2.setIndex(new THREE.BufferAttribute(fp.idxT, 1));
        g2.computeVertexNormals();
        const m2 = new THREE.Mesh(g2, mat);
        m2.matrixAutoUpdate = false; m2.renderOrder = 1; root.add(m2);
        boneMeshes[s.n + '_toes'] = m2;
      }
    }
  }

  // 2. 蒙皮肋軟骨
  for (const sk of (MAN.skinBones || [])) {
    const nv = sk.nv, q16 = new Uint16Array(BIN, sk.off, nv * 3);
    const bp = new Float32Array(nv * 3), pos = new Float32Array(nv * 3);
    for (let i = 0; i < nv; i++) {
      bp[i * 3] = sk.lo[0] + q16[i * 3] * sk.q;
      bp[i * 3 + 1] = sk.lo[1] + q16[i * 3 + 1] * sk.q;
      bp[i * 3 + 2] = sk.lo[2] + q16[i * 3 + 2] * sk.q;
    }
    const fi = new Uint16Array(BIN, sk.off + nv * 6, sk.nf * 3).slice();
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setIndex(new THREE.BufferAttribute(fi, 1));
    const mat = new THREE.MeshStandardMaterial({ color: bc, roughness: 0.76, metalness: 0, transparent: true, opacity: st.boneOp, side: THREE.DoubleSide, depthWrite: st.boneOp > 0.95 });
    const m = new THREE.Mesh(g, mat);
    m.frustumCulled = false; m.renderOrder = 1; root.add(m);
    boneMeshes[sk.n] = m;
    m.userData.skin = { bp, idx: new Uint8Array(BIN, sk.offI, nv * 4).slice(), w: new Uint8Array(BIN, sk.offW, nv * 4).slice() };
  }

  // 3. 肌肉網格（頂點平滑羽化漸層：肌腹力學著色 ↔ Z-Anatomy 原生解剖肌腱象牙白）+ UCL
  const RAD = 8;
  for (const t of TIS) {
    if (t.kind === 'm') {
      const mu = t.mu, g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(mu.nv * 3), 3));
      g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(mu.nv * 3), 3));
      g.setIndex(new THREE.BufferAttribute(mu.faces, 1));
      let mBelly;
      if (mu.hasT) {
        g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(mu.nv * 3), 3));
        mBelly = new THREE.MeshStandardMaterial({ color: 0xffffff, vertexColors: true, roughness: 0.56, metalness: 0.02, side: THREE.FrontSide, transparent: true, opacity: st.musOp, depthWrite: st.musOp > 0.97 });
      } else {
        mBelly = new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.58, metalness: 0.02, side: THREE.FrontSide, transparent: true, opacity: st.musOp, depthWrite: st.musOp > 0.97 });
      }
      const mesh = new THREE.Mesh(g, mBelly);
      mesh.frustumCulled = false; mesh.renderOrder = 2; mesh.visible = false;
      root.add(mesh);
      tubes[t.id] = { t, mesh, mu, mBelly, _lastC: -1 };
      continue;
    }
    const n = 2, g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * RAD * 3), 3));
    g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(n * RAD * 3), 3));
    const idx = [];
    for (let i = 0; i < n - 1; i++) for (let k = 0; k < RAD; k++) {
      const a = i * RAD + k, b = i * RAD + (k + 1) % RAD, c = (i + 1) * RAD + k, d2 = (i + 1) * RAD + (k + 1) % RAD;
      idx.push(a, c, b, b, c, d2);
    }
    g.setIndex(idx);
    const mat = new THREE.MeshStandardMaterial({ color: 0x64748b, roughness: 0.55, metalness: 0, side: THREE.FrontSide, transparent: true, opacity: 0.95 });
    const mesh = new THREE.Mesh(g, mat);
    mesh.frustumCulled = false; mesh.renderOrder = 3; root.add(mesh);
    tubes[t.id] = { t, mesh, n, RAD, mBelly: mat };
  }

  // 4. 獨立真實解剖肌腱／腱膜／支持帶／韌帶（阿基里斯腱、髂脛束、胸腰筋膜、腹白線、支持帶、十字韌帶等）
  for (const td of TENDS) {
    const mu = td.mu, g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(mu.nv * 3), 3));
    g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(mu.nv * 3), 3));
    g.setIndex(new THREE.BufferAttribute(mu.faces, 1));
    const colHex = td.cat === 'ligament' ? 0xd9d2c5 : (td.cat === 'retinaculum' ? 0xded9ce : 0xeae5da);
    const mat = new THREE.MeshStandardMaterial({ color: colHex, roughness: 0.46, metalness: 0.05, side: THREE.FrontSide, transparent: true, opacity: st.musOp * 0.92, depthWrite: st.musOp > 0.95 });
    const mesh = new THREE.Mesh(g, mat);
    mesh.frustumCulled = false; mesh.renderOrder = 2; mesh.visible = false;
    root.add(mesh);
    tendMeshes[td.id] = { td, mesh, mu, mat };
  }

  // 5. 起止點標記球
  for (let i = 0; i < 4; i++) {
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.011, 14, 10), new THREE.MeshBasicMaterial({ color: i % 2 ? 0x2dd4bf : 0x38bdf8, depthTest: false }));
    m.renderOrder = 5; m.visible = false; root.add(m); pins.push(m);
  }

  // 6. 地面、投手丘、GRF 力向量
  const gy = MD.dz - MD.H;
  const GC = new THREE.Vector3(1.5, gy - 0.003, 0);
  ground = new THREE.Mesh(new THREE.CircleGeometry(5, 64), new THREE.MeshBasicMaterial({ color: cssv('--ground') }));
  ground.rotation.x = -Math.PI / 2; ground.position.copy(GC); root.add(ground);
  grid = new THREE.GridHelper(10, 40, 0x888888, 0x888888);
  grid.material.transparent = true; grid.material.opacity = 0.35; grid.material.color.set(cssv('--grid'));
  grid.position.set(1.5, gy, 0); root.add(grid);
  buildMound();
  grfArrows = [0, 1].map(i => {
    const a = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(), 0.5, i ? 0xf472b6 : 0x38bdf8, 0.08, 0.05);
    a.visible = false; a.line.material.depthTest = false; a.cone.material.depthTest = false;
    a.renderOrder = 6; a.line.renderOrder = 6; a.cone.renderOrder = 6; root.add(a); return a;
  });
  arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(3.6, gy + 0.012, 0), 1.6, 0xf59e0b, 0.18, 0.1);
  root.add(arrow);
  st.gy = gy;
  applySkinScene();
  applyCam(true);
}

function segMatrix(mesh, sg) {
  const F = sg.F, S = sg.S;
  const c0 = C3(F.x), c1 = C3(F.y), c2 = C3(F.z), o = C3(sg.O);
  mesh.matrix.set(
    c0.x * S[0], c1.x * S[1], c2.x * S[2], o.x,
    c0.y * S[0], c1.y * S[1], c2.y * S[2], o.y,
    c0.z * S[0], c1.z * S[1], c2.z * S[2], o.z,
    0, 0, 0, 1
  );
  mesh.matrixWorldNeedsUpdate = true;
}

const STOPS = [[-1, [6, 182, 212]], [-0.5, [59, 130, 246]], [0, [100, 116, 139]], [0.5, [245, 158, 11]], [1, [239, 68, 68]]];
function gradCol(u) {
  u = Math.max(-1, Math.min(1, u));
  for (let i = 0; i < STOPS.length - 1; i++) {
    const a = STOPS[i], b = STOPS[i + 1];
    if (u <= b[0]) {
      const t = (u - a[0]) / (b[0] - a[0]);
      return new THREE.Color((a[1][0] + (b[1][0] - a[1][0]) * t) / 255, (a[1][1] + (b[1][1] - a[1][1]) * t) / 255, (a[1][2] + (b[1][2] - a[1][2]) * t) / 255);
    }
  }
  return new THREE.Color(0.94, 0.27, 0.27);
}

function tubeUpdate(tb, pts, radii) {
  const { n, RAD, mesh } = tb;
  const pos = mesh.geometry.attributes.position, nor = mesh.geometry.attributes.normal;
  for (let i = 0; i < n; i++) {
    const a = pts[Math.max(0, i - 1)], b = pts[Math.min(n - 1, i + 1)];
    const t = b.clone().sub(a).normalize();
    const ref = Math.abs(t.y) > 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
    const n1 = new THREE.Vector3().crossVectors(t, ref).normalize(), n2 = new THREE.Vector3().crossVectors(t, n1);
    for (let k = 0; k < RAD; k++) {
      const ang = k / RAD * Math.PI * 2, c = Math.cos(ang), s = Math.sin(ang);
      const dx = n1.x * c + n2.x * s, dy = n1.y * c + n2.y * s, dz = n1.z * c + n2.z * s;
      const j = (i * RAD + k) * 3;
      pos.array[j] = pts[i].x + dx * radii[i]; pos.array[j + 1] = pts[i].y + dy * radii[i]; pos.array[j + 2] = pts[i].z + dz * radii[i];
      nor.array[j] = dx; nor.array[j + 1] = dy; nor.array[j + 2] = dz;
    }
  }
  pos.needsUpdate = true; nor.needsUpdate = true;
}

const _pts = [];
function updateScene() {
  if (!renderer) return;
  const P = st.P, i = Math.round(st.f), po = P.poses[i], mp = P.mats[i], A = mp.A;
  const wr = P.wraps && P.wraps[i];

  // 1. 骨骼
  for (const s of MAN.segs) {
    const sg = po[s.n];
    if (!sg || !boneMeshes[s.n]) continue;
    segMatrix(boneMeshes[s.n], sg);
    const tm = boneMeshes[s.n + '_toes'];
    if (tm) {
      segMatrix(tm, sg);
      const S = s.n.slice(0, 1), b = po[S + '_toeBend'] || 0;
      if (Math.abs(b) > 1e-4) {
        const mtp = getMTP(MAN.meta, S);
        const M = new THREE.Matrix4().makeTranslation(mtp.x, mtp.y, 0).multiply(new THREE.Matrix4().makeRotationZ(b)).multiply(new THREE.Matrix4().makeTranslation(-mtp.x, -mtp.y, 0));
        tm.matrix.multiply(M);
      }
    }
  }
  for (const k in boneMeshes) {
    const m = boneMeshes[k], sk = m.userData.skin;
    if (!sk) continue;
    const g = m.geometry, arr = g.attributes.position.array, { bp, idx, w } = sk, nv = bp.length / 3;
    for (let v = 0; v < nv; v++) {
      const x = bp[v * 3], y = bp[v * 3 + 1], z = bp[v * 3 + 2];
      let X = 0, Y = 0, Z = 0;
      for (let j = 0; j < 4; j++) {
        const ww = w[v * 4 + j]; if (!ww) continue;
        const o = idx[v * 4 + j] * 12, f = ww / 255;
        X += f * (A[o] * x + A[o + 1] * y + A[o + 2] * z + A[o + 9]);
        Y += f * (A[o + 3] * x + A[o + 4] * y + A[o + 5] * z + A[o + 10]);
        Z += f * (A[o + 6] * x + A[o + 7] * y + A[o + 8] * z + A[o + 11]);
      }
      arr[v * 3] = X; arr[v * 3 + 1] = Z; arr[v * 3 + 2] = -Y;
    }
    g.attributes.position.needsUpdate = true; g.computeVertexNormals(); g.computeBoundingSphere();
  }

  // 2. 肌肉 + UCL
  const SEL = selIds();
  pins.forEach(p => p.visible = false);
  for (const id in tubes) {
    const tb = tubes[id], t = tb.t, vis = visible(t);
    tb.mesh.visible = vis;
    if (!vis) continue;
    const smp = sample(id, st.f), col = gradCol(colorVal(t, smp));
    const si = SEL.indexOf(id), isSel = si >= 0;
    if (t.kind === 'm') {
      const o = (SEL.length && !isSel) ? Math.min(st.musOp, 0.16) : st.musOp;
      if (tb.mBelly.opacity !== o) { tb.mBelly.opacity = o; tb.mBelly.depthWrite = o > 0.97; }
      tb.mBelly.emissive = isSel ? col.clone().multiplyScalar(0.45) : new THREE.Color(0);
      const g = tb.mesh.geometry;
      if (tb.mu.hasT) {
        const keyC = ((col.r * 255) | 0) << 16 | ((col.g * 255) | 0) << 8 | ((col.b * 255) | 0);
        if (tb._lastC !== keyC) {
          tb._lastC = keyC;
          const ca = g.attributes.color.array, tf = tb.mu.tFac, nv = tb.mu.nv;
          const cr = col.r, cg = col.g, cb = col.b;
          const tr = 0.902, tg = 0.882, tb2 = 0.839; // #e6e1d6 象牙白肌腱色
          for (let v = 0; v < nv; v++) {
            const f = tf[v] / 255.0, fm = 1.0 - f;
            ca[v * 3]     = cr * fm + tr * f;
            ca[v * 3 + 1] = cg * fm + tg * f;
            ca[v * 3 + 2] = cb * fm + tb2 * f;
          }
          g.attributes.color.needsUpdate = true;
        }
      } else {
        tb.mBelly.color.copy(col);
      }
      const wp = skinMeshDQS(tb.mu, mp, g.attributes.position.array, g.attributes.normal.array, wr);
      g.attributes.position.needsUpdate = true;
      if (wp) g.computeVertexNormals();
      else g.attributes.normal.needsUpdate = true;
      if (isSel) {
        evalMtuPts(t.m.mtu, A, _pts);
        if (_pts.length >= 2) {
          pins[si * 2].visible = pins[si * 2 + 1].visible = true;
          pins[si * 2].position.copy(C3(_pts[0]));
          pins[si * 2 + 1].position.copy(C3(_pts[_pts.length - 1]));
        }
      }
      continue;
    }
    tb.mBelly.color.copy(col);
    tb.mBelly.emissive = isSel ? col.clone().multiplyScalar(0.45) : new THREE.Color(0);
    const S = t.side, hum = po[S + '_humerus'], fo = po[S + '_ulna'], u = MAN.ucl[S];
    const off = { ant: [0.004, 0, 0.002], post: [-0.004, 0, 0.002], pb: [-0.009, -0.002, 0.003] }[t.tab];
    const wp = [toWorld(hum, [u.o[0] + off[0], u.o[1] + off[1], u.o[2] + off[2]]), toWorld(fo, [u.i[0] + off[0] * 0.6, u.i[1], u.i[2] + off[2]])];
    tubeUpdate(tb, wp.map(C3), [0.0028, 0.0028]);
    if (isSel) {
      pins[si * 2].visible = pins[si * 2 + 1].visible = true;
      pins[si * 2].position.copy(C3(wp[0]));
      pins[si * 2 + 1].position.copy(C3(wp[1]));
    }
  }

  // 3. 獨立真實肌腱／腱膜／支持帶／韌帶
  const showT = st.showTend && st.vis !== 'solo';
  for (const id in tendMeshes) {
    const tm = tendMeshes[id], td = tm.td;
    const sOK = st.side === 'both' || role(td.side) === 'center' || role(td.side) === st.side;
    const vis = showT && sOK;
    tm.mesh.visible = vis;
    if (!vis) continue;
    const o = SEL.length ? Math.min(st.musOp * 0.92, 0.14) : st.musOp * 0.92;
    if (tm.mat.opacity !== o) { tm.mat.opacity = o; tm.mat.depthWrite = o > 0.90; }
    const g = tm.mesh.geometry;
    const wp = skinMeshDQS(tm.mu, mp, g.attributes.position.array, g.attributes.normal.array, wr);
    g.attributes.position.needsUpdate = true;
    if (wp) g.computeVertexNormals();
    else g.attributes.normal.needsUpdate = true;
  }

  SEL.forEach((id, k) => {
    const t = TIS.find(x => x.id === id);
    if (!t || !visible(t)) pins[k * 2].visible = pins[k * 2 + 1].visible = false;
  });

  const showB = st.showBone;
  for (const k in boneMeshes) boneMeshes[k].visible = showB;

  // 4. GRF 力向量
  if (grfArrows.length) {
    const on = st.grfOn && st.P.fp;
    if (!on) grfArrows.forEach(a => a.visible = false);
    else {
      const F = grfAt(st.f * 3), lm = st.P.fr[i].lm, SC = 0.55 / st.P.bw;
      [[F[0], lm.rear_ankle_jc], [F[1], lm.lead_ankle_jc]].forEach(([fv, ank], k) => {
        const a = grfArrows[k], mag = Math.hypot(fv[0], fv[1], fv[2]);
        if (mag < 0.03 * st.P.bw) { a.visible = false; return; }
        const org = C3([ank[0], ank[1], moundZ(ank[0], ank[1]) + 0.01]);
        const d = new THREE.Vector3(fv[0], fv[2], -fv[1]).normalize();
        a.visible = true; a.position.copy(org); a.setDirection(d);
        a.setLength(mag * SC, Math.min(0.12, mag * SC * 0.3), 0.05);
      });
    }
  }
  root.updateMatrixWorld(true);
  st._dirty = true;
}

const CAMREL = { side: [0, 0.05, 3.1], front: [3.1, 0.05, 0], back: [-3.1, 0.05, 0] };
function applyCam(inst) {
  if (!controls || !st.P) return;
  const T = st.P.ctx.T, rel = CAMREL[st.cam];
  if (!rel && st.cam !== 'pelvis') return;
  const sgn = T === 'R' ? 1 : -1, tgt = pelvisWorld(Math.round(st.f)); tgt.y += 0.1;
  const D = ((st.skin === 'c' && innerWidth > 900) ? 4.2 : 3.5) * (camera.aspect < 0.9 ? 1.45 : 1);
  const pos = new THREE.Vector3();
  if (st.cam === 'side') pos.set(tgt.x, tgt.y + 0.15, tgt.z + D * sgn);
  else if (st.cam === 'pelvis') pos.set(tgt.x - 1.6, tgt.y + 0.55, tgt.z + 1.6 * sgn);
  else if (st.cam === 'front') pos.set(tgt.x + D, tgt.y + 0.05, tgt.z);
  else if (st.cam === 'back') pos.set(tgt.x - D, tgt.y + 0.05, tgt.z);
  controls.setView(pos, tgt, inst);
}
function pelvisWorld(i) {
  const P = st.P, l = P.fr[i].lm;
  return C3(V.mul(V.add(l.rear_hip, l.lead_hip), 0.5)).applyAxisAngle(new THREE.Vector3(0, 1, 0), root.rotation.y);
}
function followCam() {
  if (!st.P || st.cam === 'free') return;
  const tgt = pelvisWorld(Math.round(st.f)); tgt.y += 0.1; controls.tC.copy(tgt);
}
