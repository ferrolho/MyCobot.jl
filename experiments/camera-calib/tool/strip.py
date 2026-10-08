"""Find the tape (yellow) in an image and resample it as a straight strip (length along the tape x width)."""
import numpy as np, cv2

def tape_strip(img, width_pad=6, step=0.5):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (18, 70, 110), (42, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    k = 1 + np.argmax(st[1:, 4]); ys, xs = np.nonzero(lab == k)
    P = np.c_[xs, ys].astype(float); c = P.mean(0)
    _, _, Vt = np.linalg.svd(P - c, full_matrices=False); u = Vt[0]
    if u[0] < 0: u = -u
    nrm = np.array([-u[1], u[0]])
    s = (P - c) @ u; w = (P - c) @ nrm
    # centre line w(s) as a quadratic (lens distortion bends straight lines a little); edges from the mask
    bins = np.arange(s.min(), s.max(), 8.0)
    sc, wc, hw = [], [], []
    for a in bins:
        sel = (s >= a) & (s < a + 8)
        if sel.sum() > 20:
            sc.append(a + 4); wc.append((np.percentile(w[sel], 2) + np.percentile(w[sel], 98)) / 2)
            hw.append((np.percentile(w[sel], 98) - np.percentile(w[sel], 2)) / 2)
    sc, wc, hw = map(np.array, (sc, wc, hw))
    pc = np.polyfit(sc, wc, 2); half = np.median(hw)
    ss = np.arange(sc.min() + 4, sc.max() - 4, step)
    ws = np.arange(-half - width_pad, half + width_pad + 0.01, 0.5)
    # local frame: centre point and normal from the polynomial
    cen = c[None, :] + ss[:, None] * u[None, :] + np.polyval(pc, ss)[:, None] * nrm[None, :]
    dw = np.polyval(np.polyder(pc), ss)
    tang = u[None, :] + dw[:, None] * nrm[None, :]; tang /= np.linalg.norm(tang, axis=1)[:, None]
    nn = np.c_[-tang[:, 1], tang[:, 0]]
    mapx = (cen[:, 0][None, :] + ws[:, None] * nn[:, 0][None, :]).astype(np.float32)
    mapy = (cen[:, 1][None, :] + ws[:, None] * nn[:, 1][None, :]).astype(np.float32)
    strip = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
    return dict(strip=strip, ss=ss, ws=ws, cen=cen, nn=nn, half=half, mapx=mapx, mapy=mapy)
