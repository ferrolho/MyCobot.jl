import numpy as np, cv2, json, sys
from scipy.optimize import least_squares
S = '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad'
rows = {}
for l in open(S + '/cal5/cal5_flange.csv'):
    v = [float(x) for x in l.split(',')]
    rows[int(v[0])] = (v[1], np.array(v[2:5]), np.array(v[5:14]).reshape(3, 3, order='F'))
det = json.load(open(S + '/cal5/det.json'))
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
p0 = np.load(S + '/cam5_0.npy')[:6]
for name, nd in (('camera only', 0), ('camera + tip dx,dy', 2)):
    def res(x):
        d = np.r_[x[6:6 + nd], np.zeros(3 - nd)]
        return (proj(x[:6], tips(d)) - UV).ravel()
    r = least_squares(res, np.r_[p0, np.zeros(nd)], loss='soft_l1', f_scale=5)
    e = np.sqrt((res(r.x).reshape(-1, 2) ** 2).sum(1))
    Rc, _ = cv2.Rodrigues(r.x[:3]); C = -Rc.T @ r.x[3:6]
    print(f'{name:20s} rms {np.sqrt((e**2).mean()):5.1f} max {e.max():5.1f}  d {np.round(r.x[6:],1)}  cam {np.round(C)}')
    print('   per point', dict(zip(K, np.round(e, 1))), ' yaws', YAW)
    np.save(S + f'/cam6_{nd}.npy', r.x)
