import numpy as np, cv2, json
from scipy.optimize import least_squares
S = '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad'
rows = {}
for l in open(S + '/cal7/cal7_flange.csv'):
    v = [float(x) for x in l.split(',')]
    rows[int(v[0])] = (v[1], np.array(v[2:5]), np.array(v[5:14]).reshape(3, 3, order='F'))
import sys
det = {int(k): v for k, v in json.load(open(sys.argv[1])).items()}
use = sorted(k for k in det if k not in [int(x) for x in sys.argv[2:]])
T = np.array([rows[k][1] for k in use]); R = np.array([rows[k][2] for k in use]); UV = np.array([det[k] for k in use])
def tips(d):
    return T + np.einsum('nij,j->ni', R, np.array([d[0], 8 + d[1], 115 + d[2]]))
Kc = np.array([[1272, 0, 640], [0, 1272, 480], [0, 0, 1.0]])
def proj(p, P):
    Rc, _ = cv2.Rodrigues(p[:3]); Pc = P @ Rc.T + p[3:6]
    return np.c_[1272 * Pc[:, 0] / Pc[:, 2] + 640, 1272 * Pc[:, 1] / Pc[:, 2] + 480]
P0 = tips([6.7, 2.6, 0])
ok, rv, t = cv2.solvePnP(P0, UV, Kc, None, flags=cv2.SOLVEPNP_SQPNP)
p0 = np.r_[rv.ravel(), t.ravel()]
print('PnP init rms', round(float(np.sqrt(((proj(p0, P0) - UV) ** 2).sum(1).mean())), 1))
for name, free_d in (('offset fixed (6.7, 2.6)', False), ('offset free', True)):
    def res(x):
        d = x[6:8] if free_d else [6.7, 2.6]
        return (proj(x[:6], tips([d[0], d[1], 0])) - UV).ravel()
    r = least_squares(res, np.r_[p0, 6.7, 2.6] if free_d else p0, loss='soft_l1', f_scale=5)
    e = np.sqrt((res(r.x).reshape(-1, 2) ** 2).sum(1))
    Rc, _ = cv2.Rodrigues(r.x[:3]); C = -Rc.T @ r.x[3:6]
    print(f'{name:24s} rms {np.sqrt((e**2).mean()):4.1f} max {e.max():4.1f} cam {np.round(C)} d {np.round(r.x[6:],1)}  per point {dict(zip(use, np.round(e,1)))}')
    np.save(S + ('/cam10_free.npy' if free_d else '/cam10_fixed.npy'), np.r_[r.x[:6], 1272, 640, 480, 0])
    if free_d: np.save(S + '/cam10_d.npy', r.x[6:8])
