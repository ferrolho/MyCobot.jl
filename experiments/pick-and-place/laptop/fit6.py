import numpy as np, cv2, json, sys
from scipy.optimize import least_squares
S = '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad'
rows = {}
for l in open(S + '/cal6/cal6_flange.csv'):
    v = [float(x) for x in l.split(',')]
    rows[int(v[0])] = (v[1], np.array(v[2:5]), np.array(v[5:14]).reshape(3, 3, order='F'))
det = json.load(open(S + '/cal6/det.json'))
drop = [int(a) for a in sys.argv[1:]]
K, YAW, T, R, UV = [], [], [], [], []
for fn, (u, v) in det.items():
    k = int(fn.split('_')[-1].split('.')[0])
    if k in drop: continue
    yaw, t, Rm = rows[k]; K.append(k); YAW.append(yaw); T.append(t); R.append(Rm); UV.append((u, v))
T, R, UV = np.array(T), np.array(R), np.array(UV)
def tips(d):   # d = (dx, dy, dz) in the flange frame, relative to (0, 8, 115)
    off = np.array([d[0], 8 + d[1], 115 + d[2]])
    return T + np.einsum('nij,j->ni', R, off)
def proj(p, P):
    Rc, _ = cv2.Rodrigues(p[:3]); Pc = P @ Rc.T + p[3:6]
    return np.c_[1272 * Pc[:, 0] / Pc[:, 2] + 640, 1272 * Pc[:, 1] / Pc[:, 2] + 480]
K = np.array([[1272, 0, 640], [0, 1272, 480], [0, 0, 1.0]])
P_init = tips(np.array([6.7, 2.6, 0.0]))
Q = P_init.copy(); zc = Q[:, 2].mean(); Q[:, 2] = 0
ok, rvs, ts, _ = cv2.solvePnPGeneric(Q, UV, K, None, flags=cv2.SOLVEPNP_IPPE)
best = None
for rv, t in zip(rvs, ts):
    Rm, _ = cv2.Rodrigues(rv); t2 = (t.ravel() + Rm @ np.array([0, 0, -zc]))
    e = np.sqrt(((proj(np.r_[rv.ravel(), t2], P_init) - UV) ** 2).sum(1)).mean()
    if best is None or e < best[0]: best = (e, np.r_[rv.ravel(), t2])
print('PnP init error', round(best[0], 1), 'px')
p0 = best[1]
for name, nd in (('camera only', 0), ('camera + tip dx,dy', 2)):
    pass
for name, nd in (('camera only, d fixed', 0), ('camera + tip dx,dy', 2)):
    def res(x):
        d = np.r_[x[6:6 + nd], np.zeros(3 - nd)] if nd else np.array([6.7, 2.6, 0.0])
        return (proj(x[:6], tips(d)) - UV).ravel()
    r = least_squares(res, np.r_[p0, np.array([6.7, 2.6])[:nd]], loss='soft_l1', f_scale=5)
    e = np.sqrt((res(r.x).reshape(-1, 2) ** 2).sum(1))
    Rc, _ = cv2.Rodrigues(r.x[:3]); C = -Rc.T @ r.x[3:6]
    print(f'{name:20s} rms {np.sqrt((e**2).mean()):5.1f} max {e.max():5.1f}  d {np.round(r.x[6:],1)}  cam {np.round(C)}')
    print('   per point', dict(zip(K, np.round(e, 1))), ' yaws', YAW)
    np.save(S + f'/cam8_{nd}.npy', r.x)
