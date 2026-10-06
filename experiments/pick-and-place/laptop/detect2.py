"""Model-guided fingertip detection: search only a window around the predicted tip pixel."""
import cv2, numpy as np, json, sys
S = '/private/tmp/claude-501/-Users-henrique-myCobot/4710b7bc-7da5-4616-a4fc-077e72318965/scratchpad'
p = np.load(sys.argv[1]); d = [float(x) for x in sys.argv[2].split(',')]; out = sys.argv[3]
rows = {}
for l in open(S + '/cal7/cal7_flange.csv'):
    v = [float(x) for x in l.split(',')]
    rows[int(v[0])] = (np.array(v[2:5]), np.array(v[5:14]).reshape(3, 3, order='F'))
def proj(P):
    Rc, _ = cv2.Rodrigues(p[:3]); Pc = P @ Rc.T + p[3:6]
    return np.array([1272 * Pc[0] / Pc[2] + 640, 1272 * Pc[1] / Pc[2] + 480])
bg = cv2.imread(S + '/cal6/bg.jpg').astype(np.int16)
res = {}
for k, (t, R) in sorted(rows.items()):
    fn = S + (f'/cal6/cal6_{k}.jpg' if k < 100 else f'/cal7/cal7_{k - 100}.jpg')
    im = cv2.imread(fn)
    if im is None: continue
    u0, v0 = proj(t + R @ np.array([d[0], 8 + d[1], 115]))
    diff = np.abs(im.astype(np.int16) - bg).sum(2)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    m = ((diff > 60) & (hsv[..., 1] < 50) & (hsv[..., 2] > 160)).astype(np.uint8)
    win = np.zeros_like(m); W = 120
    x0, x1 = int(max(0, u0 - W)), int(min(1280, u0 + W)); y0, y1 = int(max(0, v0 - 2 * W)), int(min(960, v0 + W))
    win[y0:y1, x0:x1] = 1
    m = cv2.morphologyEx(m * win, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2: print(k, 'nothing near', (round(u0), round(v0))); continue
    kk = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
    ys, xs = np.nonzero(lab == kk); vmax = ys.max(); sel = ys >= vmax - 4
    u, v = float(np.median(xs[sel])), float(vmax)
    res[k] = (u, v)
    vis = im.copy(); vis[lab == kk] = (0.5 * vis[lab == kk] + [0, 0, 127]).astype(np.uint8)
    cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 0), 2)
    cv2.circle(vis, (int(u0), int(v0)), 8, (255, 255, 0), 2); cv2.circle(vis, (int(u), int(v)), 8, (0, 255, 0), 2)
    cv2.imwrite(S + f'/cal7/v2_{k}.jpg', cv2.resize(vis, (320, 240)))
    print(k, 'pred', (round(u0), round(v0)), 'found', (round(u), round(v)))
json.dump({str(k): v for k, v in res.items()}, open(out, 'w'))
