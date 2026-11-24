"""Build-time precomputation for the articulated-skeleton / shared-field muscle model.

Inputs : blob_man.txt, blob_bin.txt (bones), rest_export.json (original muscle rest meshes), app.js (ATT table)
Outputs: blob_mus2.txt (base64 binary), blob_sdf.txt (base64 binary), man2.json (headers), build_report.json
Nothing here moves a muscle vertex except the explicit, logged rest-overlap cleanup (belly vertices inside bone).
"""
import json, base64, re, math, sys, time
import numpy as np
from scipy import ndimage

T0 = time.time()
MAN = json.load(open('blob_man.txt'))
BIN = base64.b64decode(open('blob_bin.txt').read())
REST = json.load(open('rest_export.json'))['muscles']
APP = open('att_table.js', encoding='utf-8').read()

SEGS = [s['n'] for s in MAN['segs']]
NS = len(SEGS)
TOE = {'R': NS, 'L': NS + 1}          # virtual toe segments, same ids as runtime
NB = NS + 2
MTPX = 0.095
sid = {n: i for i, n in enumerate(SEGS)}

# ---------------- bone meshes (mesh coords) ----------------
def seg_mesh(s):
    iv = np.frombuffer(BIN, dtype=np.int16, count=s['nv'] * 3, offset=s['off']).reshape(-1, 3).astype(np.float64) * s['q']
    fi = np.frombuffer(BIN, dtype=np.uint16, count=s['nf'] * 3, offset=s['off'] + s['nv'] * 6).reshape(-1, 3).astype(np.int64)
    P = np.array(s['Ob']) + iv @ np.array(s['Fb'])
    return P, fi, iv

BONE = {}
for i, s in enumerate(MAN['segs']):
    P, F, iv = seg_mesh(s)
    if s['n'] in ('R_foot', 'L_foot'):
        # same split as runtime footParts(): connected components with local centroid x >= 0.10 are toes
        nv = len(P); par = np.arange(nv)
        def f(a):
            while par[a] != a:
                par[a] = par[par[a]]; a = par[a]
            return a
        for t in F:
            a = f(t[0]); b = f(t[1]); par[b] = a; par[f(t[2])] = a
        root = np.array([f(i2) for i2 in range(nv)])
        cx = {}
        for r in np.unique(root):
            cx[r] = iv[root == r, 0].mean()
        toe_v = np.array([cx[r] >= 0.10 for r in root])
        tri_toe = toe_v[F[:, 0]]
        BONE[i] = (P, F[~tri_toe])
        BONE[TOE[s['n'][0]]] = (P, F[tri_toe])
    else:
        BONE[i] = (P, F)

# ---------------- signed distance grids ----------------
def tri_samples(P, F, h):
    A = P[F[:, 0]]; B = P[F[:, 1]]; C = P[F[:, 2]]
    emax = np.max(np.stack([np.linalg.norm(B - A, axis=1), np.linalg.norm(C - B, axis=1), np.linalg.norm(A - C, axis=1)]), axis=0)
    k = np.maximum(1, np.ceil(emax / (0.4 * h)).astype(int))
    out = [P]
    for kk in np.unique(k):
        sel = k == kk
        ii, jj = np.meshgrid(np.arange(kk + 1), np.arange(kk + 1), indexing='ij')
        m = ii + jj <= kk
        u = (ii[m] / kk)[None, :, None]; v = (jj[m] / kk)[None, :, None]
        pts = A[sel][:, None] * (1 - u - v) + B[sel][:, None] * u + C[sel][:, None] * v
        out.append(pts.reshape(-1, 3))
    return np.concatenate(out)

SDF = {}
for b in range(NB):
    P, F = BONE[b]
    used = np.unique(F)
    Pu = P[used]
    ext = Pu.max(0) - Pu.min(0)
    h = max(0.003, ext.max() / 70.0)
    margin = 0.015
    org = Pu.min(0) - margin
    dims = np.ceil((ext + 2 * margin) / h).astype(int) + 1
    surf = np.zeros(dims, bool)
    S = tri_samples(P, F, h)
    ix = np.clip(np.floor((S - org) / h + 0.5).astype(int), 0, dims - 1)
    surf[ix[:, 0], ix[:, 1], ix[:, 2]] = True
    inside = ndimage.binary_fill_holes(surf)
    dout = ndimage.distance_transform_edt(~inside) * h
    din = ndimage.distance_transform_edt(inside) * h
    sdf = np.where(surf, 0.0, np.where(inside, -(din - 0.5 * h), dout - 0.5 * h))   # surface voxels = 0
    SDF[b] = dict(org=org, h=h, dims=dims, g=sdf, filled=int(inside.sum() - surf.sum()))

def sdf_at(b, X):
    """trilinear signed distance (m) of points X (n,3, mesh coords) to bone b; outside the grid: + distance to grid box"""
    G = SDF[b]; g = G['g']; d = G['dims']
    u = (X - G['org']) / G['h']
    uc = np.clip(u, 0, d - 1.000001)
    extra = np.linalg.norm((u - uc) * G['h'], axis=1)
    i0 = np.floor(uc).astype(int); f = uc - i0; i1 = np.minimum(i0 + 1, d - 1)
    x0, y0, z0 = i0.T; x1, y1, z1 = i1.T; fx, fy, fz = f.T
    c000 = g[x0, y0, z0]; c100 = g[x1, y0, z0]; c010 = g[x0, y1, z0]; c110 = g[x1, y1, z0]
    c001 = g[x0, y0, z1]; c101 = g[x1, y0, z1]; c011 = g[x0, y1, z1]; c111 = g[x1, y1, z1]
    c00 = c000 * (1 - fx) + c100 * fx; c10 = c010 * (1 - fx) + c110 * fx; c01 = c001 * (1 - fx) + c101 * fx; c11 = c011 * (1 - fx) + c111 * fx
    c0 = c00 * (1 - fy) + c10 * fy; c1 = c01 * (1 - fy) + c11 * fy
    return c0 * (1 - fz) + c1 * fz + extra

def sdf_grad(b, X, e=None):
    e = e or SDF[b]['h']
    g = np.stack([(sdf_at(b, X + np.eye(3)[k] * e) - sdf_at(b, X - np.eye(3)[k] * e)) / (2 * e) for k in range(3)], 1)
    return g / np.maximum(np.linalg.norm(g, axis=1, keepdims=True), 1e-9)

print('sdf grids', round(time.time() - T0, 1), 's; voxels', sum(int(np.prod(SDF[b]['dims'])) for b in SDF))

# ---------------- anatomy table (ported verbatim from app.js) ----------------
att_src = re.search(r'const ATT=\{(.*?)\};', APP, re.S).group(1)
att_src = re.sub(r'/\*.*?\*/', '', att_src, flags=re.S)
ATT = dict(re.findall(r"'([^']+)':'([^']*)'", att_src))
TOK = {'P': ['pelvis'], 'TH': ['thorax'], 'HD': ['head'], 'LB': ['lumbar0', 'lumbar1', 'lumbar2', 'lumbar3', 'lumbar4'], 'sc': ['S_scap'], 'cl': ['S_clav'],
       'hu': ['S_humerus'], 'ul': ['S_ulna'], 'ra': ['S_radius'], 'hd': ['S_hand'], 'fe': ['S_femur'], 'ti': ['S_tibia'], 'tf': ['S_foot'], 'tz': ['S_toes']}

def end_set(base, side, e):
    spec = ATT.get(base)
    if spec is None: return None
    ab = spec.split('>')[e].split(',')
    if ab[0] == '-': return []
    out = []
    S = 'R' if side == 'R' else 'L'
    for t in ab:
        for nm in TOK[t]:
            if nm == 'S_toes': i = TOE[S]
            elif nm.startswith('S_'): i = sid[S + nm[1:]]
            else: i = sid[nm]
            if i not in out: out.append(i)
    return out

# ---------------- skeleton tree ----------------
PAR = {}
for i, n in enumerate(SEGS):
    PAR[i] = None
PAR[sid['lumbar4']] = sid['pelvis']
for k in range(3, -1, -1): PAR[sid['lumbar%d' % k]] = sid['lumbar%d' % (k + 1)]
PAR[sid['thorax']] = sid['lumbar0']; PAR[sid['head']] = sid['thorax']
for S in 'RL':
    PAR[sid[S + '_clav']] = sid['thorax']; PAR[sid[S + '_scap']] = sid[S + '_clav']; PAR[sid[S + '_humerus']] = sid[S + '_scap']
    PAR[sid[S + '_ulna']] = sid[S + '_humerus']; PAR[sid[S + '_radius']] = sid[S + '_ulna']; PAR[sid[S + '_hand']] = sid[S + '_radius']
    PAR[sid[S + '_femur']] = sid['pelvis']; PAR[sid[S + '_tibia']] = sid[S + '_femur']; PAR[sid[S + '_foot']] = sid[S + '_tibia']
    PAR[TOE[S]] = sid[S + '_foot']

def anc(i):
    o = [i]
    while PAR[o[-1]] is not None: o.append(PAR[o[-1]])
    return o

def tree_path(a, b):
    A = anc(a); B = anc(b)
    common = next(x for x in A if x in B)
    return A[:A.index(common) + 1] + B[:B.index(common)][::-1]

# ---------------- 中間骨（大角度關節的半程旋轉；避免 DQS 在接近 180° 時混合失效） ----------------
# 每組 (A, B, 旋轉中心所在的骨)：執行時其旋轉 = A 與 B 的旋轉各半（隨時間連續追蹤），旋轉中心 = 該骨的關節原點
MIDS = []
for S in 'RL':
    hu, sc_, cl, ul, ra, ha, fe, ti = (sid[S + x] for x in ('_humerus', '_scap', '_clav', '_ulna', '_radius', '_hand', '_femur', '_tibia'))
    MIDS += [(sc_, hu, hu), (sid['thorax'], hu, hu), (cl, hu, hu), (hu, ra, ul), (hu, ul, ul), (ul, ra, ra), (ul, ha, ha), (sid['head'], cl, sid['head']), (fe, ti, ti), (sid['pelvis'], fe, fe)]
NBX = NB + len(MIDS)
def remap_mids(sidx, wq):
    """把同時落在 A、B 的權重拆成 A–中間骨–B 三段（質量守恆、連續）"""
    nv = len(sidx); out_i = np.zeros((nv, 4), int); out_w = np.zeros((nv, 4))
    for v in range(nv):
        d = {}
        for k in range(4):
            if wq[v, k] > 0: d[int(sidx[v, k])] = d.get(int(sidx[v, k]), 0) + wq[v, k] / 255.0
        for mi, (a, b, c) in enumerate(MIDS):
            if a in d and b in d:
                m = d[a] + d[b]; t = d[b] / m; M = NB + mi
                d[a] = m * max(0.0, 1 - 2 * t); d[b] = m * max(0.0, 2 * t - 1); d[M] = d.get(M, 0) + m * (1 - abs(1 - 2 * t))
                d = {k: x for k, x in d.items() if x > 1e-6}
        it = sorted(d.items(), key=lambda kv: -kv[1])[:4]
        tot = sum(x for _, x in it)
        for k, (bb, x) in enumerate(it): out_i[v, k] = bb; out_w[v, k] = x / tot
        for k in range(len(it), 4): out_i[v, k] = out_i[v, 0]
    q = np.floor(out_w * 255 + 0.5).astype(int); q[:, 0] += 255 - q.sum(1)
    return out_i, q

# ---------------- 肌腱止點的解剖標記（網格沒畫到骨頭的少數肌腱）----------------
# 跟腱（腓腸肌內外側頭、比目魚肌、蹠肌）→ 跟骨結節後上緣；闊筋膜張肌經髂脛束 → 脛骨外側髁 Gerdy 結節
LANDMARK = {}
for S in 'RL':
    Pf, Ff = BONE[sid[S + '_foot']]; Vf = Pf[np.unique(Ff)]
    post = Vf[Vf[:, 0] < Vf[:, 0].min() + 0.008]; LANDMARK[(S, 'calc')] = (sid[S + '_foot'], post[post[:, 1] >= np.median(post[:, 1])].mean(0))
    Pt, Ft = BONE[sid[S + '_tibia']]; Vt = Pt[np.unique(Ft)]
    prox = Vt[Vt[:, 1] > Vt[:, 1].max() - 0.05]; lat = 1 if S == 'R' else -1
    LANDMARK[(S, 'gerdy')] = (sid[S + '_tibia'], prox[np.argmax(lat * prox[:, 2] + 0.7 * prox[:, 0])])
for S in 'RL':
    Ph, Fh = BONE[sid[S + '_humerus']]; Vh = Ph[np.unique(Fh)]; gh = np.array(MAN['meta'][S]['sh'])
    band = Vh[(Vh[:, 1] < gh[1] - 0.035) & (Vh[:, 1] > gh[1] - 0.065)]
    LANDMARK[(S, 'groove')] = (sid[S + '_humerus'], band[band[:, 0] > band[:, 0].max() - 0.003].mean(0) - np.array([0.002, 0, 0]))
# 背闊肌止於結節間溝底（解剖學標準描述）；網格足印偏後，故直接用此標記
FORCE_LM = {'latissimus dorsi': ('hu', 'groove')}
TENDON_LM = {'medial head of gastrocnemius': ('tf', 'calc'), 'lateral head of gastrocnemius': ('tf', 'calc'), 'soleus': ('tf', 'calc'), 'plantaris': ('tf', 'calc'), 'tensor fasciae latae': ('ti', 'gerdy')}

# ---------------- obstacles (wrap surfaces) ----------------
OBST = []
def sphere_radius(bone, c):
    P, F = BONE[bone]; d = np.linalg.norm(P[np.unique(F)] - c, axis=1)
    near = d[d < 0.035]
    return float(np.percentile(near, 35)) if len(near) > 20 else 0.022
for S in 'RL':
    c = np.array(MAN['meta'][S]['sh']); hb = sid[S + '_humerus']
    OBST.append(dict(name=S + '_humeral_head', type='sphere', bone=hb, c=c.tolist(), r=[sphere_radius(hb, c)] * 3,
                     other=[sid[S + '_scap'], sid[S + '_clav'], sid['thorax']]))
    c = np.array(MAN['meta'][S]['hip']); fb = sid[S + '_femur']
    OBST.append(dict(name=S + '_femoral_head', type='sphere', bone=fb, c=c.tolist(), r=[sphere_radius(fb, c)] * 3,
                     other=[sid['pelvis']] + [sid['lumbar%d' % k] for k in range(5)]))
# 肱骨幹（近端）：背闊肌、大圓肌、胸大肌止於結節間溝，旋轉時肌腱繞過肱骨幹
for S in 'RL':
    hb = sid[S + '_humerus']; s_ = MAN['segs'][hb]; Fb = np.array(s_['Fb']); c = np.array(MAN['meta'][S]['sh']) - Fb[1] * 0.055
    Ph, Fh = BONE[hb]; Pu_ = Ph[np.unique(Fh)]; loc = (Pu_ - c) @ Fb.T; near = np.abs(loc[:, 1]) < 0.025
    c = c + Fb.T @ np.array([loc[near, 0].mean(), 0, loc[near, 2].mean()])   # 以該高度骨幹截面中心為軸
    loc = (Pu_ - c) @ Fb.T; near = np.abs(loc[:, 1]) < 0.025
    rr = float(np.median(np.hypot(loc[near, 0], loc[near, 2]))) if near.sum() > 10 else 0.012
    OBST.append(dict(name=S + '_humeral_shaft', type='ellipsoid_local', bone=hb, c=c.tolist(), r=[rr * 0.85, 0.035, rr * 0.85], ax=Fb.tolist(),
                     other=[sid[S + '_scap'], sid['thorax'], sid[S + '_clav']]))
Pth, Fth = BONE[sid['thorax']]; Pu = Pth[np.unique(Fth)]
# rib cage only: drop the top (cervical / first rib region) and spine outliers by percentiles
lo = np.percentile(Pu, 2, axis=0); hi = np.percentile(Pu, 98, axis=0)
OBST.append(dict(name='thorax', type='ellipsoid', bone=sid['thorax'], c=((lo + hi) / 2).tolist(), r=((hi - lo) / 2 * 0.9).tolist(),
                 other=[sid[S + x] for S in 'RL' for x in ('_scap', '_humerus', '_clav')]))

def obst_rho(o, X):
    c = np.array(o['c']); r = np.array(o['r'])
    if 'ax' in o: X = c + (X - c) @ np.array(o['ax']).T
    return np.sqrt((((X - c) / r) ** 2).sum(1))

# ---------------- per-muscle processing ----------------
def b64arr(a): return np.frombuffer(base64.b64decode(a), dtype=np.float32)
def smooth01(a, b, x):
    t = np.clip((x - a) / (b - a), 0, 1); return t * t * (3 - 2 * t)

def centerline(m, P, fp):
    K, M = m['K'], m['M']
    if m['mode'] == 1:
        C = fp.reshape(K, M, 3).mean(0)
    else:
        c = P.mean(0); U, s, Vt = np.linalg.svd(P - c, full_matrices=False); ax = Vt[0]
        t = (P - c) @ ax; lo_, hi_ = t.min(), t.max(); n = 9
        edges = np.linspace(lo_, hi_, n + 1); C = []
        for k in range(n):
            sel = (t >= edges[k]) & (t <= edges[k + 1])
            C.append(P[sel].mean(0) if sel.sum() else c + ax * (edges[k] + edges[k + 1]) / 2)
        C = np.array(C)
        C = np.vstack([P[np.argmin(t)] * 0.5 + C[0] * 0.5, C, P[np.argmax(t)] * 0.5 + C[-1] * 0.5])
    return C

def closest_on_poly(X, C):
    segs = C[1:] - C[:-1]; L = np.linalg.norm(segs, axis=1); cum = np.concatenate([[0], np.cumsum(L)]); tot = cum[-1] or 1e-9
    best_d = np.full(len(X), 1e9); best_p = np.zeros_like(X); best_u = np.zeros(len(X))
    for k in range(len(segs)):
        a = C[k]; ab = segs[k]; ll = (ab @ ab) or 1e-12
        t = np.clip(((X - a) @ ab) / ll, 0, 1); p = a + t[:, None] * ab; d = np.linalg.norm(X - p, axis=1)
        sel = d < best_d; best_d[sel] = d[sel]; best_p[sel] = p[sel]; best_u[sel] = (cum[k] + t[sel] * L[k]) / tot
    return best_p, best_u

def mesh_neighbors(nv, F):
    rows = np.concatenate([F[:, 0], F[:, 1], F[:, 2], F[:, 1], F[:, 2], F[:, 0]])
    cols = np.concatenate([F[:, 1], F[:, 2], F[:, 0], F[:, 0], F[:, 1], F[:, 2]])
    from scipy.sparse import coo_matrix
    A = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(nv, nv)).tocsr(); A.data[:] = 1
    deg = np.asarray(A.sum(1)).ravel(); deg[deg == 0] = 1
    return A, deg

import os
DELTA, KPOW, GH = float(os.environ.get('WDELTA', 0.012)), float(os.environ.get('WK', 3.0)), 0.012
# 全身共用的空間權重場：在 1.2 cm 格網上算出每個節點對 28 塊骨頭的權重（只看到骨面的距離），頂點以三線性內插取值，
# 再只保留該肌肉起止點骨頭鏈上的骨頭並正規化 → 空間上相鄰的點（例如腹壁的不同層）拿到幾乎相同的權重。
_allP = np.concatenate([b64arr(r['pos']).reshape(-1, 3) for r in REST]).astype(np.float64)
GORG = _allP.min(0) - 0.03; GDIM = np.ceil((_allP.max(0) + 0.03 - GORG) / GH).astype(int) + 1
_gx = [GORG[k] + np.arange(GDIM[k]) * GH for k in range(3)]
_GP = np.stack(np.meshgrid(*_gx, indexing='ij'), -1).reshape(-1, 3)
WGRID = np.stack([1.0 / (np.maximum(sdf_at(b, _GP), 0) + DELTA) ** KPOW for b in range(NB)], 1).reshape(*GDIM, NB).astype(np.float32)
print('weight grid', GDIM, round(time.time() - T0, 1), 's')
def field_W(X):
    u = np.clip((X - GORG) / GH, 0, GDIM - 1.000001); i0 = np.floor(u).astype(int); f = u - i0
    out = 0
    for dx in (0, 1):
        for dy in (0, 1):
            for dz in (0, 1):
                w = (f[:, 0] if dx else 1 - f[:, 0]) * (f[:, 1] if dy else 1 - f[:, 1]) * (f[:, 2] if dz else 1 - f[:, 2])
                out = out + w[:, None] * WGRID[i0[:, 0] + dx, i0[:, 1] + dy, i0[:, 2] + dz]
    return out
def raw_weights(X, chain):
    W = field_W(X)[:, chain]
    return W / W.sum(1, keepdims=True), None

HDR = []; BLOCKS = []; off = 0
report = dict(muscles=0, cleanup_moved=0, cleanup_max_mm=0, cleanup_skipped=0, chain_sizes={}, single=0, wrap_use={})
for m, r in zip(MAN['muscles'], REST):
    assert m['id'] == r['id']
    P = b64arr(r['pos']).reshape(-1, 3).astype(np.float64)
    F = np.frombuffer(base64.b64decode(r['faces']), dtype=np.uint16).reshape(-1, 3).astype(np.int64)
    fp = b64arr(r['fp']).astype(np.float64)
    nv = len(P)
    C = centerline(m, P, fp)
    E = [end_set(m['base'], m['side'], 0), end_set(m['base'], m['side'], 1)]
    if E[0] is None:
        if m['base'] != 'external anal sphincter': raise SystemExit('unmapped ' + m['base'])
        E = [[sid['pelvis']], [sid['pelvis']]]   # not shown in the UI; kept rigid on the pelvis
    allb = list(range(NB))
    def setcost(S, x):
        if not S: return 0.0
        return min(max(0.0, float(sdf_at(b, x[None])[0])) for b in S)
    c_same = setcost(E[0], C[0]) + setcost(E[1], C[-1]); c_swap = setcost(E[1], C[0]) + setcost(E[0], C[-1])
    if c_swap < c_same: E = [E[1], E[0]]
    for e in (0, 1):
        if not E[e]:   # soft-tissue end: nearest bone to that end (within 4 cm), else the other end's set
            x = C[0] if e == 0 else C[-1]
            d = [(max(0.0, float(sdf_at(b, x[None])[0])), b) for b in allb]
            d.sort()
            E[e] = [d[0][1]] if d[0][0] < 0.04 else list(E[1 - e])
    if not E[0]: E[0] = E[1]
    if not E[1]: E[1] = E[0]
    chain = []
    for a in E[0]:
        for b in E[1]:
            for x in tree_path(a, b):
                if x not in chain: chain.append(x)
    report['chain_sizes'][len(chain)] = report['chain_sizes'].get(len(chain), 0) + 1
    single = len(chain) == 1
    # ---- rest-overlap cleanup (belly vertices only; attachment regions untouched)
    _, U = closest_on_poly(P, C)
    belly = smooth01(0.12, 0.35, U) * smooth01(0.88, 0.65, U)
    moved = 0
    for b in chain:
        s = sdf_at(b, P)
        sel = (belly > 0.5) & (s < -max(0.001, 0.5 * SDF[b]['h']))
        if sel.any():
            g = sdf_grad(b, P[sel]); push = (0.0005 - s[sel])
            ok = push < 0.010
            idx = np.where(sel)[0]
            P[idx[ok]] += g[ok] * push[ok, None]
            moved += int(ok.sum()); report['cleanup_skipped'] += int((~ok).sum())
            if ok.any(): report['cleanup_max_mm'] = max(report['cleanup_max_mm'], float(push[ok].max() * 1000))
    report['cleanup_moved'] += moved
    report.setdefault('cleanup_by',{})[m['zh']+m['side']]=moved
    # ---- helpers: via-point bone assignment & wrap obstacles (same rule for centreline path and display fibres)
    def assign(Cx):
        pb = []
        for i, x in enumerate(Cx):
            if i == 0 or i == len(Cx) - 1:
                S = E[0] if i == 0 else E[1]
                bb = min(S, key=lambda q_: max(0.0, float(sdf_at(q_, x[None])[0])))
                # 網格沒有畫到該骨（例如闊筋膜張肌止於髂脛束→脛骨，但網格只到大腿上段）：此端實際是軟組織延伸，
                # 固定在端點所在位置的骨頭（同一鏈、空間場最大者），不把網格末端拉到遠處的骨頭上
                if max(0.0, float(sdf_at(bb, x[None])[0])) > 0.03 and not single:
                    w, _ = raw_weights(x[None], chain); bb = chain[int(np.argmax(w[0]))]; report['soft_end'] = report.get('soft_end', 0) + 1
            elif single:
                bb = chain[0]
            else:
                w, _ = raw_weights(x[None], chain); bb = chain[int(np.argmax(w[0]))]
            pb.append(int(bb))
        return pb
    def obstacles(Cx):
        obs = []
        if single: return obs
        sub = np.concatenate([Cx[k] + (Cx[k + 1] - Cx[k]) * np.linspace(0, 1, 7)[:, None] for k in range(len(Cx) - 1)])
        for oi, o in enumerate(OBST):
            if o['bone'] in chain and any(x in chain for x in o['other']):
                rho = obst_rho(o, sub).min(); rend = obst_rho(o, Cx[[0, -1]]).min()
                if rho >= 1.0 or rend >= 1.05: obs.append([oi, 1.0])     # 兩端都在包繞面外：靜止時就繞過（例如背闊肌繞肱骨幹）
                elif rho >= 0.6: obs.append([oi, round(float(rho) * 0.98, 4)])
        return obs
    def resample(Cx, n):
        cum = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(Cx, axis=0), axis=1))])
        tt = np.linspace(0, cum[-1], n)
        return np.stack([np.interp(tt, cum, Cx[:, k]) for k in range(3)], 1)
    # ---- centreline path (data)
    Cc = resample(C, 10) if len(C) != 10 else C
    pb = assign(Cc)
    # 數據路徑的起止點：以肌肉網格貼在起止骨上的「足印」（該端半段、離骨面 < 4 mm 的頂點）中心為準；
    # 網格沒畫到起止骨時（肌腱、髂脛束等未建模），沿肌肉末端走向延伸到骨頭內側面為止，補一段肌腱。
    Uc = closest_on_poly(P, C)[1]
    for e in (0, 1):
        i = 0 if e == 0 else len(Cc) - 1; x = Cc[i]; S = E[e]
        if single: continue
        fl = FORCE_LM.get(m['base'])
        if fl:
            tb_, tp_ = LANDMARK[(m['side'], fl[1])]
            if tb_ in S:
                Cc = Cc.copy(); Cc[i] = tp_; pb[i] = int(tb_); report.setdefault('force_lm', []).append(m['zh'] + m['side']); continue
        half = (Uc < 0.5) if e == 0 else (Uc >= 0.5)
        dS = np.stack([sdf_at(bq, P) for bq in S], 1); dmin = dS.min(1); fpm = half & (dmin < 0.004)
        if fpm.sum() >= 5:
            cen = P[fpm].mean(0); bq = S[int(np.argmin([float(sdf_at(b_, cen[None])[0]) for b_ in S]))]
            dd = float(sdf_at(bq, cen[None])[0])
            if abs(dd) < 0.01: cen = cen - sdf_grad(bq, cen[None])[0] * dd
            Cc = Cc.copy(); Cc[i] = cen; pb[i] = int(bq); report['footprint'] = report.get('footprint', 0) + 1
            continue
        bb = min(S, key=lambda q_: max(0.0, float(sdf_at(q_, x[None])[0])))
        tl = TENDON_LM.get(m['base'])
        if tl and float(sdf_at(bb, x[None])[0]) > 0.01:
            tb_, tp_ = LANDMARK[(m['side'], tl[1])]
            if tb_ in S:
                if e == 0: Cc = np.vstack([tp_[None], Cc]); pb = [int(tb_)] + pb
                else: Cc = np.vstack([Cc, tp_[None]]); pb = pb + [int(tb_)]
                report.setdefault('tendon_lm_list', []).append(m['zh'] + m['side']); continue
        if float(sdf_at(bb, x[None])[0]) > 0.03:
            tdir = (Cc[0] - Cc[1]) if e == 0 else (Cc[-1] - Cc[-2]); tdir = tdir / max(np.linalg.norm(tdir), 1e-9)
            ray = x[None] + tdir[None] * np.arange(0.005, 0.40, 0.0025)[:, None]
            hit = None
            for bq in S:
                k_ = np.where(sdf_at(bq, ray) < 0)[0]
                if len(k_) and (hit is None or k_[0] < hit[0]): hit = (k_[0], bq)
            if hit is None: report.setdefault('tendon_miss_list', []).append(m['zh'] + m['side'] + ('起' if e == 0 else '止')); continue
            y = ray[hit[0]]; bb = hit[1]
            if e == 0: Cc = np.vstack([y[None], Cc]); pb = [int(bb)] + pb
            else: Cc = np.vstack([Cc, y[None]]); pb = pb + [int(bb)]
            report.setdefault('tendon_ext_list', []).append(m['zh'] + m['side'] + ('起' if e == 0 else '止'))
    obs = obstacles(Cc)
    for o in obs: report['wrap_use'][OBST[o[0]]['name']] = report['wrap_use'].get(OBST[o[0]]['name'], 0) + 1
    # ---- display fibres: split the muscle across its width into K bands; each band's centroid line is a fibre
    Cp, U = closest_on_poly(P, C)
    Rr = P - Cp
    tang = np.gradient(C, axis=0); tang /= np.maximum(np.linalg.norm(tang, axis=1, keepdims=True), 1e-9)
    # width axis: principal direction of residuals with the local tangent removed
    ti = np.clip((U * (len(C) - 1)).round().astype(int), 0, len(C) - 1)
    Rp = Rr - (Rr * tang[ti]).sum(1, keepdims=True) * tang[ti]
    _, _, Vt = np.linalg.svd(Rp - Rp.mean(0), full_matrices=False); wax = Vt[0]
    tw = Rp @ wax
    width = float(np.percentile(tw, 95) - np.percentile(tw, 5))
    K = 1 if single else int(np.clip(np.ceil(width / 0.03), 1, 8))
    edges_ = np.percentile(tw, np.linspace(0, 100, K + 1)); edges_[0] -= 1; edges_[-1] += 1
    band = np.clip(np.searchsorted(edges_, tw, side='right') - 1, 0, K - 1)
    MF = 10; uj = np.linspace(0, 1, MF)
    fibres = []
    for k in range(K):
        sel = band == k; pts = []
        for j in range(MF):
            win = 0.06 if j in (0, MF - 1) else 0.08
            near = sel & (np.abs(U - uj[j]) <= win)
            if near.sum() >= 3: pts.append(P[near].mean(0))
            else:
                cj = C[min(len(C) - 1, int(round(uj[j] * (len(C) - 1))))]
                pts.append(cj + (Rr[sel].mean(0) if sel.any() else 0))
        Fk = np.array(pts)
        Fk[1:-1] = 0.25 * Fk[:-2] + 0.5 * Fk[1:-1] + 0.25 * Fk[2:]
        fibres.append(Fk)
    fpb = [assign(Fk) for Fk in fibres]; fobs = [obstacles(Fk) for Fk in fibres]
    # ---- vertex binding to the two nearest fibres
    D = np.zeros((nv, K)); S_ = np.zeros((nv, K))
    for k, Fk in enumerate(fibres):
        cp, u = closest_on_poly(P, Fk); D[:, k] = np.linalg.norm(P - cp, axis=1); S_[:, k] = u
    k0 = np.argmin(D, 1)
    if K > 1:
        dn = np.where(k0 > 0, D[np.arange(nv), np.maximum(k0 - 1, 0)], 1e9); up = np.where(k0 < K - 1, D[np.arange(nv), np.minimum(k0 + 1, K - 1)], 1e9)
        k1 = np.where(dn < up, k0 - 1, k0 + 1); k1 = np.clip(k1, 0, K - 1)
        d0 = D[np.arange(nv), k0]; d1 = D[np.arange(nv), k1]
        bw = np.where(k1 != k0, d0 / np.maximum(d0 + d1, 1e-9), 0.0)
    else:
        k1 = k0.copy(); bw = np.zeros(nv)
    sv = (1 - bw) * S_[np.arange(nv), k0] + bw * S_[np.arange(nv), k1]
    # ---- shared-field mixing weight α (0 at attachments, up to 0.5 in the belly) and belly thickening weight
    belly = smooth01(0.12, 0.35, U) * smooth01(0.88, 0.65, U)
    alpha = 0.5 * belly if not single else np.zeros(nv)
    # ---- bone push: two nearest chain bones and the rest-pose signed distance to each
    Dch = np.stack([sdf_at(bb, P) for bb in chain], 1) if chain else np.zeros((nv, 1))
    ordc = np.argsort(np.maximum(Dch, 0), 1)[:, :2]
    if ordc.shape[1] == 1: ordc = np.hstack([ordc, ordc])
    pbn = np.array(chain)[ordc]; rs = np.take_along_axis(Dch, ordc, 1)
    rsq = np.clip(np.round(rs / 0.0005), -127, 127).astype(np.int8)
    # region for the shared field (trunk / limbs) = region of the bone holding most fibre points
    REG = lambda bb: (3 if bb == TOE['R'] else 4) if bb >= NS else (1 if SEGS[bb].startswith('R_') and any(x in SEGS[bb] for x in ('humerus', 'ulna', 'radius', 'hand')) else
                                          2 if SEGS[bb].startswith('L_') and any(x in SEGS[bb] for x in ('humerus', 'ulna', 'radius', 'hand')) else
                                          3 if SEGS[bb].startswith('R_') and any(x in SEGS[bb] for x in ('femur', 'tibia', 'foot')) else
                                          4 if SEGS[bb].startswith('L_') and any(x in SEGS[bb] for x in ('femur', 'tibia', 'foot')) else 0)
    regs = [REG(bb) if bb < NB else (3 if bb == TOE['R'] else 4) for f_ in fpb for bb in f_]
    region = int(np.bincount(regs, minlength=5).argmax())
    # ---- pack
    lo_ = P.min(0); hi_ = P.max(0); q = float((hi_ - lo_).max() / 65535) or 1e-9
    pq = np.round((P - lo_) / q).astype(np.uint16)
    blk = pq.tobytes() + F.astype(np.uint16).tobytes() + np.round(sv * 65535).astype(np.uint16).tobytes() + \
        k0.astype(np.uint8).tobytes() + k1.astype(np.uint8).tobytes() + np.round(bw * 255).astype(np.uint8).tobytes() + \
        np.round(alpha * 255).astype(np.uint8).tobytes() + np.round(belly * 255).astype(np.uint8).tobytes() + \
        pbn.astype(np.uint8).tobytes() + rsq.tobytes()
    blk += b'\0' * ((-len(blk)) % 4)
    rnd = lambda v: [round(float(x), 6) for x in v]
    HDR.append(dict(id=m['id'], nv=nv, nf=len(F), off=off, lo=lo_.tolist(), q=q, chain=chain, ends=E, single=single, region=region,
                    path=[[pb[i]] + rnd(Cc[i]) for i in range(len(Cc))], obs=obs,
                    fib=[dict(p=[[fpb[k][i]] + rnd(fibres[k][i]) for i in range(MF)], obs=fobs[k]) for k in range(K)], wax=rnd(wax)))
    BLOCKS.append(blk); off += len(blk)
    report['fibres'] = report.get('fibres', 0) + K
    report['muscles'] += 1
report['single'] = sum(1 for h in HDR if h['single'])
print('muscles', round(time.time() - T0, 1), 's')

# ---------------- SDF blob ----------------
sdf_hdr = []; sdf_bytes = []; so = 0
for b in range(NB):
    G = SDF[b]; g = np.clip(np.round(G['g'] / 0.0004), -127, 127).astype(np.int8)
    raw = np.ascontiguousarray(g.transpose(2, 1, 0)).tobytes()    # x fastest
    sdf_hdr.append(dict(b=b, org=G['org'].tolist(), h=G['h'], dims=G['dims'].tolist(), off=so)); sdf_bytes.append(raw); so += len(raw)

man2 = dict(muscles=HDR, obst=[{k: (v if not isinstance(v, np.ndarray) else v.tolist()) for k, v in o.items()} for o in OBST], sdf=dict(unit=0.0004, bones=sdf_hdr), NB=NB)
json.dump(man2, open('man2.json', 'w'), separators=(',', ':'))
open('blob_mus2.txt', 'w').write(base64.b64encode(b''.join(BLOCKS)).decode())
open('blob_sdf.txt', 'w').write(base64.b64encode(b''.join(sdf_bytes)).decode())
report['sdf_bytes'] = so; report['mus_bytes'] = off; report['secs'] = round(time.time() - T0, 1)
report['filled']={SEGS[b] if b<NS else 'toe'+str(b):[SDF[b]['filled'],int(np.prod(SDF[b]['dims']))] for b in SDF}
report['obst'] = [(o['name'], [round(x, 4) for x in o['r']]) for o in OBST]
json.dump(report, open('build_report.json', 'w'), indent=1)
print(json.dumps(report, indent=1))
