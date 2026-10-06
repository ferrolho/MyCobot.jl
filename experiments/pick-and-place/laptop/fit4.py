import numpy as np, cv2, json
from scipy.optimize import least_squares
S = '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad'
def load(csv, det, off=(0, 0), xyz_cols=slice(7, 10)):
    rows = {}
    for l in open(csv):
        v = [float(x) for x in l.split(',')]
        rows[int(v[0])] = v[xyz_cols] if xyz_cols.start == 7 else v[1:4]
    d = json.load(open(det))
    P, uv = [], []
    for fn, (u, v) in d.items():
        k = int(fn.split('_')[-1].split('.')[0]); P.append(rows[k]); uv.append((u + off[0], v + off[1]))
    return np.array(P), np.array(uv)
P3, uv3 = load(S + '/cal3/cal3.csv', S + '/cal3/det_ok.json', off=(7.5, 2.2))
P4, uv4 = load(S + '/cal4/cal4.csv', S + '/cal4/det_ok.json', xyz_cols=slice(1, 4))
P = np.vstack([P3, P4]); uv = np.vstack([uv3, uv4]); src = np.r_[np.zeros(len(P3)), np.ones(len(P4))]
print('points: old', len(P3), 'new', len(P4), ' z range', P[:, 2].min().round(), P[:, 2].max().round())
def proj(p, P):
    R, _ = cv2.Rodrigues(p[:3]); Pc = P @ R.T + p[3:6]
    x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]; s = 1 + p[9] * (x * x + y * y)
    return np.c_[p[6] * x * s + p[7], p[6] * y * s + p[8]]
old = np.load(S + '/cam3b_0.npy')
p0 = old.copy()
# bring the old pose into the shifted frame by a fit with the old model as start
def run(free, P, uv):
    idx = [i for i in range(10) if i < 6 or i in free]
    def res(x):
        p = p0.copy(); p[idx] = x; return (proj(p, P) - uv).ravel()
    r = least_squares(res, p0[idx], loss='soft_l1', f_scale=5); p = p0.copy(); p[idx] = r.x
    e = np.sqrt(((proj(p, P) - uv) ** 2).sum(1)); return p, e
e_old = np.sqrt(((proj(old, P) - uv) ** 2).sum(1))
print(f'old model on all: rms {np.sqrt((e_old**2).mean()):.1f}  new pts rms {np.sqrt((e_old[src==1]**2).mean()):.1f} px')
for name, free in (('f fixed 1272', []), ('f free', [6]), ('f + centre', [6, 7, 8]), ('f + k1', [6, 9])):
    p, e = run(free, P, uv)
    R, _ = cv2.Rodrigues(p[:3]); C = -R.T @ p[3:6]
    print(f'{name:13s} rms {np.sqrt((e**2).mean()):4.1f} max {e.max():4.1f} (old {np.sqrt((e[src==0]**2).mean()):.1f}, new {np.sqrt((e[src==1]**2).mean()):.1f})  f {p[6]:5.0f} c ({p[7]:.0f},{p[8]:.0f}) k1 {p[9]:+.3f} cam {np.round(C)}')
    np.save(S + f'/cam4_{len(free)}.npy', p)
