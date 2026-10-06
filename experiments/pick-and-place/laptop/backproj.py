import numpy as np, cv2, sys, json
c = np.load(sys.argv[1]); f = float(c['f'])
K = np.array([[f, 0, 640], [0, f, 480], [0, 0, 1.0]])
R, _ = cv2.Rodrigues(c['rv']); t = c['t'].ravel(); C = -R.T @ t
def ray(u, v):
    d = R.T @ np.linalg.solve(K, [u, v, 1.0]); return d / np.linalg.norm(d)
def on_plane(u, v, z):
    d = ray(u, v); s = (z - C[2]) / d[2]; return C + s * d
pts = json.loads(sys.argv[2])
for name, (u, v), z in pts:
    p = on_plane(u, v, z); print(f'{name:12s} ({u},{v}) z={z}: x {p[0]:.0f} y {p[1]:.0f}  r {np.hypot(p[0], p[1]):.0f} az {np.degrees(np.arctan2(p[1], p[0])) % 360:.0f}')
