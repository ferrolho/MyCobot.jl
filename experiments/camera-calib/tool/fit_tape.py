"""Camera calibration from tape images: intrinsics (f, cx, cy, k1, k2) and the camera height and tilt over
the table, from cm digits 10 mm apart on straight tapes lying on the table. The camera's position along the
table and its heading are not observable from the tapes (the table is the same everywhere)."""
import sys, glob, json, numpy as np, cv2
from scipy.optimize import least_squares
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from ticks import detect
from line1d import fit1d

def rot(a, b):   # camera axes in the table frame: tilt a about x, then b about y
    ca, sa, cb, sb = np.cos(a), np.sin(a), np.cos(b), np.sin(b)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]]); Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    return Rx @ Ry

def project(P, cam):   # P: table points (n,3); cam: dict; returns pixels (n,2)
    R = rot(cam['a'], cam['b']); C = np.array([0, 0, cam['h']])
    Pc = (P - C) @ R            # camera coordinates (R maps camera axes to table)
    x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]; r2 = x * x + y * y
    dfac = 1 + cam['k1'] * r2 + cam['k2'] * r2 * r2
    return np.c_[cam['f'] * x * dfac + cam['cx'], cam['f'] * y * dfac + cam['cy']]

def unpack(x, n):
    cam = dict(f=x[0], cx=x[1], cy=x[2], k1=x[3], k2=x[4], a=x[5], b=x[6], h=x[7])
    return cam, x[8:].reshape(n, 3)

def calibrate(files, init, plot_dir=None):
    obs = []
    for f in files:
        d = detect(cv2.imread(f)); _, r = fit1d(d['pts'], d['idx'])
        ok = np.abs(r) < 3
        obs.append((d['pts'][ok], d['idx'][ok].astype(float)))
    n = len(obs)
    cam0 = init
    # per tape: line start (x0, y0) and heading th on the table, from the initial camera (back-projection)
    lines = []
    for pts, k in obs:
        R = rot(cam0['a'], cam0['b']); C = np.array([0, 0, cam0['h']])
        def bp(p):
            v = np.array([(p[0] - cam0['cx']) / cam0['f'], (p[1] - cam0['cy']) / cam0['f'], 1.0]); d = R @ v
            return C + (-C[2] / d[2]) * d
        A, B = bp(pts[0]), bp(pts[-1]); th = np.arctan2(B[1] - A[1], B[0] - A[0])
        lines.append([A[0] - 10 * k[0] * np.cos(th), A[1] - 10 * k[0] * np.sin(th), th])
    x0 = np.r_[cam0['f'], cam0['cx'], cam0['cy'], 0, 0, cam0['a'], cam0['b'], cam0['h'], np.ravel(lines)]
    def res(x):
        cam, L = unpack(x, n); out = []
        for (pts, k), (lx, ly, th) in zip(obs, L):
            P = np.c_[lx + 10 * k * np.cos(th), ly + 10 * k * np.sin(th), np.zeros_like(k)]
            out.append((project(P, cam) - pts).ravel())
        return np.concatenate(out)
    r = least_squares(res, x0, loss='soft_l1', f_scale=2.0, x_scale='jac')
    cam, L = unpack(r.x, n)
    e = res(r.x).reshape(-1, 2); per = np.sqrt((e ** 2).sum(1))
    return cam, L, per, obs

if __name__ == '__main__':
    init = json.loads(sys.argv[1]); files = sys.argv[2:]
    cam, L, per, obs = calibrate(files, init)
    print(json.dumps({k: round(float(v), 5) for k, v in cam.items()}))
    print('points', len(per), 'rms', round(float(np.sqrt((per ** 2).mean())), 2), 'px, max', round(float(per.max()), 1))
