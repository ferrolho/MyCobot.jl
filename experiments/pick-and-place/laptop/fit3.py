import numpy as np, cv2, json, sys
from scipy.optimize import least_squares
def load(csv, det):
    rows = {int(float(l.split(',')[0])): [float(x) for x in l.split(',')[7:10]] for l in open(csv)}
    d = json.load(open(det))
    P, uv, ks = [], [], []
    for fn, (u, v) in d.items():
        k = int(fn.split('_')[-1].split('.')[0]); P.append(rows[k]); uv.append((u, v)); ks.append(k)
    return np.array(P), np.array(uv), ks
P, uv, ks = load(sys.argv[1], sys.argv[2])
def proj(p, P):
    f, cx, cy, k1 = p[6], p[7], p[8], p[9]
    R, _ = cv2.Rodrigues(p[:3]); Pc = P @ R.T + p[3:6]
    x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]; s = 1 + k1 * (x * x + y * y)
    return np.c_[f * x * s + cx, f * y * s + cy]
K0 = np.array([[1272, 0, 640], [0, 1272, 480], [0, 0, 1.0]])
ok, rv, t = cv2.solvePnP(P, uv, K0, None, flags=cv2.SOLVEPNP_SQPNP)
p0 = np.r_[rv.ravel(), t.ravel(), 1272, 640, 480, 0]
def run(free):
    idx = [i for i in range(10) if i < 6 or i in free]
    def res(x):
        p = p0.copy(); p[idx] = x; return (proj(p, P) - uv).ravel()
    r = least_squares(res, p0[idx]); p = p0.copy(); p[idx] = r.x
    e = np.sqrt(((proj(p, P) - uv) ** 2).sum(1)); return p, e
for name, free in (('f=1272 fixed', []), ('f free', [6]), ('f, centre free', [6, 7, 8]), ('f, centre, k1', [6, 7, 8, 9])):
    p, e = run(free)
    R, _ = cv2.Rodrigues(p[:3]); C = -R.T @ p[3:6]
    print(f'{name:16s} rms {np.sqrt((e**2).mean()):5.1f} px max {e.max():5.1f}  f {p[6]:6.0f} c ({p[7]:.0f},{p[8]:.0f}) k1 {p[9]:+.3f}  cam {np.round(C)}')
    np.save(sys.argv[3] + '_' + str(len(free)) + '.npy', p)
