"""Joint camera fit: base-plate screws (CAD positions), tape digits (10 mm, on the table at z = -32 mm),
and fingertip points (arm FK). Unknowns: camera pose, f, fingertip offset in the flange frame, tape lines."""
import sys, glob, json; sys.path.insert(0, __file__.rsplit('/', 1)[0]); import numpy as np, cv2
from scipy.optimize import least_squares
from screws import XYZ
import fit_tape as FT
CX, CY = 640.0, 480.0

def load_tapes(files):
    out = []
    for fn in files:
        d = FT.detect(cv2.imread(fn)); _, r = FT.fit1d(d['pts'], d['idx']); ok = np.abs(r) < 3
        out.append((d['pts'][ok], d['idx'][ok].astype(float)))
    return out

def fit(screw_px, tapes, tips, init, free_f=True, w_screw=3.0):
    keys = list(screw_px); O = np.array([XYZ[k] for k in keys]); I = np.array([screw_px[k] for k in keys], float)
    FT_T, FT_R, FI = tips
    def table_bp(px, rvv, tvv, ff):
        K = np.array([[ff, 0, CX], [0, ff, CY], [0, 0, 1.]]); R, _ = cv2.Rodrigues(rvv); C = -R.T @ tvv
        d = R.T @ np.linalg.solve(K, [px[0], px[1], 1.0]); return C + (-32 - C[2]) / d[2] * d
    lines = []
    for pts, k in tapes:
        A = table_bp(pts[0], init[:3], init[3:6], init[6]); B = table_bp(pts[-1], init[:3], init[3:6], init[6])
        th = np.arctan2(B[1] - A[1], B[0] - A[0]); lines += [A[0] - 10 * k[0] * np.cos(th), A[1] - 10 * k[0] * np.sin(th), th]
    def split(x):
        f = x[6] if free_f else init[6]; o = 7 if free_f else 6
        return x[:3], x[3:6], f, x[o:o + 2], x[o + 2:].reshape(-1, 3)
    def res(x, parts=False):
        rv, tv, f, dt, L = split(x); K = np.array([[f, 0, CX], [0, f, CY], [0, 0, 1.]])
        es = cv2.projectPoints(O, rv, tv, K, None)[0].reshape(-1, 2) - I
        ef = np.zeros((0, 2))
        if len(FI):
            P = FT_T + np.einsum('nij,j->ni', FT_R, np.array([dt[0], 8 + dt[1], 115.0]))
            ef = cv2.projectPoints(P, rv, tv, K, None)[0].reshape(-1, 2) - FI
        et = [cv2.projectPoints(np.c_[lx + 10 * k * np.cos(th), ly + 10 * k * np.sin(th), np.full_like(k, -32.0)], rv, tv, K, None)[0].reshape(-1, 2) - pts
              for (pts, k), (lx, ly, th) in zip(tapes, L)]
        et = np.concatenate(et)
        return (es, ef, et) if parts else np.concatenate([w_screw * es.ravel(), ef.ravel(), et.ravel()])
    x0 = np.r_[init[:6], [init[6]] if free_f else [], 6.7, 2.6, lines]
    r = least_squares(res, x0, loss='soft_l1', f_scale=3.0, x_scale='jac')
    rv, tv, f, dt, L = split(r.x)
    cov = np.linalg.pinv(r.jac.T @ r.jac) * np.mean(r.fun ** 2)
    return dict(rv=rv, tv=tv, f=f, dtip=dt, parts=res(r.x, True), f_sd=np.sqrt(cov[6, 6]) if free_f else 0.0)
