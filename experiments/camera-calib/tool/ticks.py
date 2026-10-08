"""Find the cm digits along a tape: one dark blob per cm (the digits of the cm scale), so the spacing of the
detections is 10 mm. Returns the image points (on the tape centre line) and their cm index (relative)."""
import numpy as np, cv2
from scipy.signal import find_peaks
from strip import tape_strip

def local_period(p, lo=6, hi=70):
    p = p - p.mean(); ac = np.correlate(p, p, 'full')[len(p) - 1:]; ac /= ac[0] + 1e-9
    k = lo + np.argmax(ac[lo:hi]); return k, ac[k]

def detect(img, step=0.5):
    r = tape_strip(img, step=step)
    S = cv2.cvtColor(r['strip'], cv2.COLOR_BGR2GRAY).astype(float)
    ws, half = r['ws'], r['half']
    inside = np.abs(ws) < half * 0.92
    dark = 255 - S
    # pick the band (one half of the tape) whose profile is most periodic: the cm digits
    best = None
    for side in (-1, 1):
        rows = inside & (side * ws > half * 0.05) & (side * ws < half * 0.75)
        prof = dark[rows].mean(0)
        prof = prof - cv2.GaussianBlur(prof[None, :], (0, 0), 25)[0]          # remove slow shading
        n = len(prof); score = 0; per = []
        for a in range(0, n - 160, 80):
            k, v = local_period(prof[a:a + 160], lo=12, hi=min(150, 159)); score += v; per.append((a + 80, k))
        if best is None or score > best[0]: best = (score, side, prof, per)
    _, side, prof, per = best
    prof = cv2.GaussianBlur(prof[None, :], (0, 0), 1.5)[0]
    # expected period along the strip (smooth fit of the local estimates), then peaks at least 0.6 period apart
    pa = np.array(per, float)
    good = pa[:, 1] > 8
    cf = np.polyfit(pa[good, 0], pa[good, 1], 1) if good.sum() >= 2 else [0, np.median(pa[:, 1])]
    period = np.polyval(cf, np.arange(len(prof)))
    pk, _ = find_peaks(prof, distance=max(4, int(0.6 * period.min())), prominence=np.std(prof) * 0.4)
    pk = [p for p in pk if 0.25 * period[p] < p < len(prof) - 0.25 * period[p]]
    # consecutive indices: a gap of ~2 periods means one missed digit; drop peaks closer than 0.6 period
    idx, keep = [0], [pk[0]]
    for p in pk[1:]:
        g = (p - keep[-1]) / period[p]
        if g < 0.6: continue
        idx.append(idx[-1] + max(1, int(round(g)))); keep.append(p)
    keep = np.array(keep)
    pts = r['cen'][keep]                        # on the centre line of the tape
    return dict(pts=pts, idx=np.array(idx), period=period[keep] / (1 / step), side=side, strip=r)
