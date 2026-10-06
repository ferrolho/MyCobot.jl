"""Fit how a gripped utensil hangs: it pivots about the finger axis, so it lies in the vertical plane through
the TCP square to the finger axis. Search the angle psi (0 = straight down, + toward w) of the lower ray
whose projection covers the most utensil-blue pixels; then walk along it to find the length.
Usage: hangfit.py IMG px py pz ax ay [out.jpg]  ->  prints psi, length, the lower end and the vector TCP->end."""
import sys
import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad')
from cam import project

img = Image.open(sys.argv[1]).convert('RGB')
a = np.asarray(img).astype(int)
R, G, B = a[..., 0], a[..., 1], a[..., 2]
mask = (B - R > 22) & (B > 110) & (G - R > 8)
p = np.array([float(v) for v in sys.argv[2:5]])
ax = np.array([float(sys.argv[5]), float(sys.argv[6]), 0.0]); ax /= np.linalg.norm(ax)
w = np.array([-ax[1], ax[0], 0.0])
Z = np.array([0, 0, 1.0])


def d_of(psi):
    return -np.cos(np.radians(psi)) * Z + np.sin(np.radians(psi)) * w


def hits(P):
    u, v = project(P).ravel()
    u, v = int(round(u)), int(round(v))
    if not (2 <= u < mask.shape[1] - 2 and 2 <= v < mask.shape[0] - 2):
        return 0.0
    return mask[v - 2:v + 3, u - 2:u + 3].mean()


best = None
for psi in np.arange(-88, 89, 2.0):
    d = d_of(psi)
    s = np.mean([hits(p + t * d) for t in np.arange(15, 75, 3.0)])
    if best is None or s > best[1]:
        best = (psi, s)
psi = best[0]
for dpsi in np.arange(-2, 2.1, 0.5):   # refine
    d = d_of(psi + dpsi)
    s = np.mean([hits(p + t * d) for t in np.arange(15, 75, 3.0)])
    if s > best[1]:
        best = (psi + dpsi, s)
psi, score = best
d = d_of(psi)
L, miss = 0.0, 0
for t in np.arange(10, 160, 2.0):
    if hits(p + t * d) > 0.3:
        L, miss = t, 0
    else:
        miss += 1
        if miss > 4:
            break
end = p + L * d
print(f"psi {psi:.1f} score {score:.2f} length {L:.0f} end {end.round(1).tolist()} vec {(L * d).round(1).tolist()}")
if len(sys.argv) > 7:
    dr = ImageDraw.Draw(img)
    for t, c in ((L, (255, 0, 0)),):
        u0, v0 = project(p).ravel(); u1, v1 = project(p + t * d).ravel()
        dr.line([(u0, v0), (u1, v1)], fill=c, width=3)
    m = Image.fromarray((mask * 255).astype(np.uint8))
    img.save(sys.argv[7]); m.save(sys.argv[7].replace('.jpg', '_mask.png'))
