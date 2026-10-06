import numpy as np, json, sys, cv2
data = json.load(open(sys.argv[1]))
P = np.array([d[0] for d in data], float); uv = np.array([d[1] for d in data], float)
best = None
for f in range(700, 2001, 50):
    K = np.array([[f, 0, 640], [0, f, 480], [0, 0, 1.0]])
    flags = cv2.SOLVEPNP_SQPNP
    ok, rv, t = cv2.solvePnP(P, uv, K, None, flags=flags)
    if np.ptp(P[:, 2]) < 5:
        Q = P.copy(); Q[:, 2] = 0
        ok, rvs, ts, _ = cv2.solvePnPGeneric(Q, uv, K, None, flags=cv2.SOLVEPNP_IPPE)
        if not ok: continue
        Rz = np.eye(3); rv, t = rvs[0], ts[0] + (cv2.Rodrigues(rvs[0])[0] @ np.array([[0], [0], [-P[:, 2].mean()]]))
    if not ok: continue
    ok, rv, t = cv2.solvePnP(P, uv, K, None, rv, t, True, cv2.SOLVEPNP_ITERATIVE)
    pr, _ = cv2.projectPoints(P, rv, t, K, None)
    e = np.sqrt(((pr[:, 0] - uv) ** 2).sum(1))
    if best is None or e.mean() < best[0]: best = (e.mean(), f, rv, t, e)
e, f, rv, t, res = best
R, _ = cv2.Rodrigues(rv); C = (-R.T @ t).ravel()
print(f'f {f}  mean err {e:.1f} px  per point {np.round(res, 1)}')
print('camera centre (base mm)', np.round(C), ' view dir', np.round(R[2], 2))
np.savez(sys.argv[2], f=f, rv=rv, t=t)
