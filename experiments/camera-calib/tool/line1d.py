"""Check the digit detections of one tape: equally spaced points on a straight line project to positions
s(k) = (a k + b) / (c k + 1) along the image line (1D homography). Fit robustly, report residuals."""
import numpy as np
from scipy.optimize import least_squares

def fit1d(pts, idx):
    c0 = pts.mean(0); _, _, Vt = np.linalg.svd(pts - c0); u = Vt[0]
    s = (pts - c0) @ u
    k = idx.astype(float)
    def res(p): return (p[0] * k + p[1]) / (p[2] * k + 1) - s
    A = np.polyfit(k, s, 1)
    r = least_squares(res, [A[0], A[1], 0.0], loss='soft_l1', f_scale=2.0)
    return r.x, res(r.x)
