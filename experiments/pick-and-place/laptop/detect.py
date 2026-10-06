import cv2, numpy as np, sys, glob, json
bg = cv2.imread(sys.argv[1]).astype(np.int16)
out = {}
for fn in sorted(sys.argv[2:], key=lambda s: int(s.split('_')[-1].split('.')[0])):
    im = cv2.imread(fn)
    d = np.abs(im.astype(np.int16) - bg).sum(2)
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    white = (hsv[..., 1] < 45) & (hsv[..., 2] > 170)
    m = ((d > 60) & white).astype(np.uint8)
    m[:, :300] = 0                       # the white wall
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2: print(fn, 'nothing'); continue
    k = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
    ys, xs = np.nonzero(lab == k)
    vmax = ys.max(); sel = ys >= vmax - 4
    u, v = float(np.median(xs[sel])), float(vmax)
    out[fn] = (u, v)
    vis = im.copy(); vis[lab == k] = (0.5 * vis[lab == k] + [0, 0, 127]).astype(np.uint8)
    cv2.circle(vis, (int(u), int(v)), 8, (0, 255, 0), 2)
    cv2.imwrite(fn.replace('.jpg', '_det.jpg'), vis)
    print(fn, round(u), round(v), 'area', st[k, cv2.CC_STAT_AREA])
json.dump(out, open('det.json', 'w'))
