import numpy as np, sys, json
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
def project(par, P):
    f, rv, t = par[0], par[1:4], par[4:7]
    cx, cy = (par[7], par[8]) if len(par) > 7 else (640, 480)
    k1 = par[9] if len(par) > 9 else 0.0
    Pc = Rot.from_rotvec(rv).apply(P) + t
    x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
    r2 = x * x + y * y
    d = 1 + k1 * r2
    return np.c_[f * x * d + cx, f * y * d + cy]
def fit(P, uv, par0, free_c=False, k1=False):
    p0 = list(par0[:7]) + ([640, 480] if free_c else []) + ([0.0] if k1 else [])
    if free_c and len(par0) > 7: p0[7:9] = par0[7:9]
    r = least_squares(lambda p: (project(p, P) - uv).ravel(), p0, loss='soft_l1', f_scale=10)
    return r.x, project(r.x, P) - uv
def init(P, uv):
    # brute-force initial pose: camera looking at the centroid from many directions
    best = None
    c = P.mean(0)
    for az in range(0, 360, 20):
        for el in (30, 45, 60, 75):
            for dist in (500, 800, 1200):
                d = np.array([np.cos(np.radians(el)) * np.cos(np.radians(az)), np.cos(np.radians(el)) * np.sin(np.radians(az)), np.sin(np.radians(el))])
                C = c + dist * d
                z = (c - C) / np.linalg.norm(c - C)
                x = np.cross(z, [0, 0, 1.0]); x /= np.linalg.norm(x); y = np.cross(z, x)
                for roll in range(0, 360, 45):
                    R0 = np.vstack([x, y, z])
                    Rr = Rot.from_euler('z', roll, degrees=True).as_matrix() @ R0
                    rv = Rot.from_matrix(Rr).as_rotvec(); t = -Rr @ C
                    for f in (900, 1300):
                        par, res = fit(P, uv, [f, *rv, *t])
                        e = np.sqrt((res ** 2).sum(1)).mean()
                        if np.all((Rot.from_rotvec(par[1:4]).apply(P) + par[4:7])[:, 2] > 0) and (best is None or e < best[1]):
                            best = (par, e)
    return best
if __name__ == '__main__':
    data = json.load(open(sys.argv[1]))
    P = np.array([d[0] for d in data], float); uv = np.array([d[1] for d in data], float)
    par, e = init(P, uv)
    print('mean err px', e)
    par, res = fit(P, uv, par)
    print('f', par[0], 'res', np.round(np.sqrt((res**2).sum(1)), 1))
    R = Rot.from_rotvec(par[1:4]).as_matrix(); C = -R.T @ par[4:7]
    print('camera centre (base mm)', np.round(C), 'view dir', np.round(R[2], 2))
    np.save(sys.argv[2], par)
