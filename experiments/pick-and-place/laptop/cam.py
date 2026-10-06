import numpy as np, cv2
S = '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad'
p = np.load(S + '/cam10_free.npy')   # 2026-10-06 22:50: base turned 180°, camera moved (12 pts, 9.2 px rms)
R, _ = cv2.Rodrigues(p[:3]); t = p[3:6]; f = p[6]; C = -R.T @ t
K = np.array([[f, 0, p[7]], [0, f, p[8]], [0, 0, 1.0]])
def project(P):
    Pc = np.atleast_2d(P) @ R.T + t; return np.c_[f * Pc[:, 0] / Pc[:, 2] + p[7], f * Pc[:, 1] / Pc[:, 2] + p[8]]
def ray(u, v):
    d = R.T @ np.linalg.solve(K, [u, v, 1.0]); return d / np.linalg.norm(d)
def on_plane(u, v, z):
    d = ray(u, v); return C + (z - C[2]) / d[2] * d
def vertical_edge(top, bot):
    """3D vertical segment whose ends project to top and bot: returns x, y, z_top, z_bot."""
    d1, d2 = ray(*top), ray(*bot)
    A = np.c_[d1[:2], -d2[:2]]; s, u = np.linalg.solve(A, np.zeros(2) + 0 - 0 + (C[:2] - C[:2]) if False else np.zeros(2)) if False else np.linalg.lstsq(A, np.zeros(2), rcond=None)[0]
    return None
