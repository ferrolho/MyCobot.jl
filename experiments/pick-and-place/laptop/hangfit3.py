"""Hang fit from the image direction only (robust to a camera-arm offset of a few cm): the hanging utensil is the
blue blob that reaches farthest below the gripper; its principal axis in the image gives psi (the tilt in the
hang plane that projects to the same image direction at the TCP), and its pixel length / the projected scale at
the TCP gives the length below the top of the blob. Usage: hangfit3.py IMG px py pz ax ay [out.jpg]"""
import sys, numpy as np, cv2
sys.path.insert(0, '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad')
from cam import project
img = cv2.imread(sys.argv[1]); b, g, r = [img[..., i].astype(int) for i in range(3)]
mask = ((b - r > 22) & (b > 110) & (g - r > 8)).astype(np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
p = np.array([float(v) for v in sys.argv[2:5]]); a = np.array([float(sys.argv[5]), float(sys.argv[6]), 0.0]); a /= np.linalg.norm(a)
w = np.cross([0, 0, 1.0], a); Z = np.array([0, 0, 1.0])
u0, v0 = project(p).ravel()
n, lab, st, _ = cv2.connectedComponentsWithStats(mask)
best = None
for i in range(1, n):
    x, y, ww, hh, area = st[i]
    if area < 600 or abs(x + ww / 2 - u0) > 180 or y > v0 + 150 or y + hh < v0: continue
    if best is None or y + hh > best[0]: best = (y + hh, i)       # reaches farthest down
if best is None: sys.exit('no blob under the gripper')
ys, xs = np.nonzero(lab == best[1]); P = np.c_[xs, ys].astype(float); m = P.mean(0)
_, _, Vt = np.linalg.svd(P - m, full_matrices=False); d2 = Vt[0] if Vt[0][1] > 0 else -Vt[0]
s = (P - m) @ d2; top_px, bot_px = m + s.min() * d2, m + s.max() * d2
obs = (bot_px - top_px) / np.linalg.norm(bot_px - top_px)
def d_of(psi): return -np.cos(np.radians(psi)) * Z + np.sin(np.radians(psi)) * w
def imdir(psi):
    im = (project(p + 60 * d_of(psi)) - project(p)).ravel(); return im / np.linalg.norm(im)
psis = np.arange(-85, 85.01, 0.5); err = [np.degrees(np.arccos(np.clip(imdir(x) @ obs, -1, 1))) for x in psis]
psi = psis[int(np.argmin(err))]
scale = np.linalg.norm((project(p + d_of(psi)) - project(p)).ravel())    # px per mm along the utensil at the TCP
L = np.linalg.norm(bot_px - top_px) / scale
print(f"psi {psi:.1f} (image fit {min(err):.1f}°) length below blob top {L:.0f} mm, top px {top_px.round(0).tolist()} bottom px {bot_px.round(0).tolist()} TCP px {u0:.0f},{v0:.0f}")
if len(sys.argv) > 7:
    cv2.line(img, tuple(int(x) for x in top_px), tuple(int(x) for x in bot_px), (0, 0, 255), 3)
    cv2.circle(img, (int(u0), int(v0)), 6, (0, 255, 255), 2); cv2.imwrite(sys.argv[7], img)
