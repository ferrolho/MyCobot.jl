#!/usr/bin/env python3
"""Track the camera pose continuously from the six brass screws on the base plate, and serve it to the
Control page's camera overlay: the tracker rewrites ~/myCobot/lab-camera.json (the lab service serves it as
/lab/camera.json), with the screws it found as `marks`.

    python3 track_screws.py [--url http://127.0.0.1:8280] [--rate 4] [--out ~/myCobot/lab-camera.json]
    python3 track_screws.py --replay IMG [IMG ...]   # offline: run the filter over images, print the poses

It works only while someone watches the camera (the lab service's /camera.json: running, clients > 0), so
it never starts the camera itself.

Model: camera pose = M · T_accurate. T_accurate is the calibrated pose of the reference image (screws, tape
and fingertips, 2026-10-08); M = exp(δ) is the camera's motion since then, seen through the screws (as in
rereg_screws.relative). A Kalman filter estimates δ (rotation vector, rad; translation, mm):
- prediction: δ stays, its uncertainty grows (random walk: a loose mount creeps, a bump moves it);
- update: each screw found near its predicted place is one measurement (2 px). A screw that is far from
  where the filter expects it (chi-square gate) is left out: hidden by the arm, in its shadow, or matched
  to the hole next to it. With few screws the filter keeps what the others do not see;
- a real move (3+ screws far off, two frames in a row): the uncertainty grows so the filter follows;
- lost (fewer than 2 screws for 3 frames): the full-image search of rereg_screws finds the plate again.
"""
import argparse
import datetime as dt
import json
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from screws import XYZ                                         # noqa: E402
from rereg_screws import REF, K_of, T_of, p_of, subpix, register  # noqa: E402

TPL = np.load(os.path.join(REF, "screw_templates.npz"))
POSE = np.load(os.path.join(REF, "pose_ref.npz"))
PLATE = [str(k) for k in TPL["keys"]]                          # the six brass screws on the base plate (CAD holes)
# The six screws on top of the round base housing (joint1, fixed to the plate): 60° apart at 30° + k·60°, radius
# 34.5 mm, heads at 73 mm, first at 31° (fitted with the plate screws over 34 snapshots, 1.2 px rms, 2026-10-09; the user confirmed six. The URDF mesh has eight
# holes 45° apart, which this robot does not have). Out of the plate's plane, they fix the camera's tilt and shift.
TOP = [f"T{k + 1}" for k in range(6)]
TOP_XYZ = [[34.5 * np.cos(np.radians(31 + 60 * k)), 34.5 * np.sin(np.radians(31 + 60 * k)), 73.0] for k in range(6)]
KEYS = PLATE + TOP
X = np.array([XYZ[k] for k in PLATE] + TOP_XYZ, float)          # mm, base frame
DEG = np.pi / 180
P0 = np.diag([(1.0 * DEG) ** 2] * 3 + [2.0 ** 2] * 3)          # after a full-image search: 1°, 2 mm
P_KNOWN = np.diag([(0.1 * DEG) ** 2] * 3 + [0.5 ** 2] * 3)     # from a known camera model: 0.1°, 0.5 mm
Q = np.diag([(0.05 * DEG) ** 2] * 3 + [0.3 ** 2] * 3)          # random walk per second: 0.05°, 0.3 mm
JUMP = np.diag([(3.0 * DEG) ** 2] * 3 + [20.0 ** 2] * 3)       # a real move: 3°, 20 mm
R_PX = 1.5 ** 2                                                # screw measurement noise (px²)
GATE = 13.8                                                    # chi-square, 2 dof, 99.9 %
MIN_SCORE = 0.6


def motion(d):
    """M = exp(δ): rotation vector d[:3] (rad), translation d[3:] (mm)."""
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(d[:3], float).reshape(3, 1))[0]
    T[:3, 3] = d[3:]
    return T


def delta_of(M):
    return np.r_[cv2.Rodrigues(M[:3, :3])[0].ravel(), M[:3, 3]]


def project(d, idx):
    p = p_of(motion(d) @ T_of(POSE["accurate"]), POSE["accurate"])   # the camera now (absolute: the screws are not all in one plane)
    return cv2.projectPoints(X[idx], p[:3], p[3:6], K_of(p), None)[0].reshape(-1, 2)


def jacobian(d, k):
    eps = np.r_[[1e-5] * 3, [1e-3] * 3]
    z0 = project(d, [k])[0]
    J = np.zeros((2, 6))
    for i in range(6):
        e = np.zeros(6)
        e[i] = eps[i]
        J[:, i] = (project(d + e, [k])[0] - z0) / eps[i]
    return z0, J


def find_dot(g, u, v, win):
    """A top screw: a small light dot on the black housing (difference of Gaussians), not on the white arm."""
    x0, y0 = max(0, int(u) - win - 8), max(0, int(v) - win - 8)
    x1, y1 = min(g.shape[1], int(u) + win + 9), min(g.shape[0], int(v) + win + 9)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None
    w = g[y0:y1, x0:x1].astype(np.float32)
    a, b = cv2.GaussianBlur(w, (0, 0), 1.2), cv2.GaussianBlur(w, (0, 0), 3.5)
    dog = a - b
    dog[(b > 140) | (a > 170)] = 0                     # bright surroundings: the white arm, not the black ring
    inner = np.zeros_like(dog)
    inner[8:-8, 8:-8] = dog[8:-8, 8:-8]
    _, peak, _, (bx, by) = cv2.minMaxLoc(inner)
    if peak < 8:
        return None
    sub = np.clip(dog[by - 2:by + 3, bx - 2:bx + 3], 0, None)
    yy, xx = np.mgrid[-2:3, -2:3]
    cx, cy = bx + (sub * xx).sum() / sub.sum(), by + (sub * yy).sum() / sub.sum()
    return np.array([x0 + cx, y0 + cy]), float(min(1.0, peak / 25))


def find_screw(g, k, u, v, win):
    if k >= len(PLATE):
        return find_dot(g, u, v, win)
    T = TPL[f"t_{KEYS[k]}"]
    h = T.shape[0] // 2
    x0, y0 = max(0, int(u) - h - win), max(0, int(v) - h - win)
    x1, y1 = min(g.shape[1], int(u) + h + 1 + win), min(g.shape[0], int(v) + h + 1 + win)
    if x1 - x0 < T.shape[1] + 2 or y1 - y0 < T.shape[0] + 2:
        return None
    r = cv2.matchTemplate(g[y0:y1, x0:x1], T, cv2.TM_CCOEFF_NORMED)
    _, s, _, (bx, by) = cv2.minMaxLoc(r)
    if s < MIN_SCORE:
        return None
    sx, sy = subpix(r, bx, by)
    return np.array([x0 + sx + h, y0 + sy + h]), s


class Tracker:
    def __init__(self, d=None):
        self.d = np.zeros(6) if d is None else np.asarray(d, float)
        self.P = P0.copy() if d is None else P_KNOWN.copy()
        self.t = None
        self.lost = 0
        self.last = dict(found={}, rejected=[], reacquired=False)

    def step(self, img, t):
        if self.t is not None:
            self.P += Q * max(0.0, min(t - self.t, 5.0))
        self.t = t
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        found, rejected = self.update(g, need=2)
        if not found:                        # the camera moved, or the screws are hidden: search wider, now;
            saved = (self.d.copy(), self.P.copy())   # keep it only if 3+ screws agree (not 2 matches by chance)
            self.P = self.P + JUMP
            found, rejected = self.update(g, need=3)
            if not found:
                self.d, self.P = saved
        self.lost = self.lost + 1 if len(found) < 2 else 0
        reacq = False
        if self.lost >= 2:      # find the plate in the whole image (rereg_screws), then track again
            try:
                s_new, f2, e, sc, shift = register(img, POSE["screws"])
                self.d = delta_of(T_of(s_new) @ np.linalg.inv(T_of(POSE["screws"])))
                self.P = P0.copy()
                self.lost = 0
                reacq = True
            except SystemExit:
                pass
        self.last = dict(found=found, rejected=rejected, reacquired=reacq)
        return self.camera()

    def update(self, g, need):
        """Match the screws, update with them if `need` or more pass the gate and agree afterwards (each within
        3 px of where the new pose puts it). Otherwise leave the filter as it was. Returns (found, rejected)."""
        cand, rejected = self.match(g)
        if len(cand) < need:
            return {}, rejected
        saved = (self.d.copy(), self.P.copy())
        while len(cand) >= need:
            self.d, self.P = saved[0].copy(), saved[1].copy()
            for k, (z, s) in cand:
                z0, H = jacobian(self.d, k)
                S = H @ self.P @ H.T + R_PX * np.eye(2)
                K = self.P @ H.T @ np.linalg.inv(S)
                self.d = self.d + K @ (z - z0)
                self.P = (np.eye(6) - K @ H) @ self.P
            res = np.linalg.norm(project(self.d, [k for k, _ in cand]) - np.array([z for _, (z, s) in cand]), axis=1)
            if res.max() <= 3.0:
                return {KEYS[k]: m for k, m in cand}, rejected
            worst = int(np.argmax(res))      # a wrong match (a highlight, a hole next to it): drop it, update again
            rejected.append(KEYS[cand[worst][0]])
            cand.pop(worst)
        self.d, self.P = saved
        return {}, rejected + [KEYS[k] for k, _ in cand]

    def match(self, g):
        """Each screw near its predicted place (window from the uncertainty); the ones that pass the gate."""
        cand, rejected = [], []
        for k in range(len(KEYS)):
            z0, H = jacobian(self.d, k)
            S = H @ self.P @ H.T + R_PX * np.eye(2)
            win = int(np.clip(3 * np.sqrt(np.max(np.linalg.eigvalsh(S))) + 4, 8, 120))
            m = find_screw(g, k, z0[0], z0[1], win)
            if m is None:
                continue
            y = m[0] - z0
            if y @ np.linalg.solve(S, y) > GATE:
                rejected.append(KEYS[k])
            else:
                cand.append((k, m))
        return cand, rejected

    def camera(self):
        """The camera parameters (as current_cam.npy: rvec, tvec mm, f, cx, cy, k1)."""
        return p_of(motion(self.d) @ T_of(POSE["accurate"]), POSE["accurate"])

    def sigma(self):
        s = np.sqrt(np.diag(self.P))
        return float(np.degrees(np.linalg.norm(s[:3]))), float(np.linalg.norm(s[3:]))


def model_json(p, tr, w=1280, h=960):
    sd, sm = tr.sigma()
    return {
        "updated": dt.datetime.now().isoformat(timespec="seconds"),
        "source": "track_screws.py: the calibrated camera (CAD base screws, tape and fingertips, 2026-10-08), moved as "
                  "the base plate screws show, Kalman filter. Base frame with the plate centred on J1.",
        "width": w, "height": h,
        "rvec": [round(float(x), 9) for x in p[:3]], "tvec": [round(float(x), 6) for x in p[3:6]],
        "f": round(float(p[6]), 5), "cx": float(p[7]), "cy": float(p[8]), "k1": float(p[9]),
        "marks": [{"name": k, "u": round(float(z[0]), 1), "v": round(float(z[1]), 1)} for k, (z, s) in sorted(tr.last["found"].items())],
        "tracking": {"screws": len(tr.last["found"]), "rejected": tr.last["rejected"], "reacquired": tr.last["reacquired"],
                     "sigma_deg": round(sd, 3), "sigma_mm": round(sm, 2)},
    }


def start_delta(path):
    """δ for the camera model in a lab-camera.json (the last pose): the filter starts there, not at the reference."""
    try:
        m = json.load(open(path))
        p = POSE["accurate"].copy()
        p[:3], p[3:6] = m["rvec"], m["tvec"]
        return delta_of(T_of(p) @ np.linalg.inv(T_of(POSE["accurate"])))
    except (OSError, KeyError, ValueError):
        return None


def write_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1)
    os.replace(tmp, path)


def get(url, timeout=2.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8280")
    ap.add_argument("--rate", type=float, default=4.0)
    ap.add_argument("--out", default=os.path.expanduser("~/myCobot/lab-camera.json"))
    ap.add_argument("--replay", nargs="+")
    ap.add_argument("--start", help="camera model to start from (default: --out, if it exists)")
    args = ap.parse_args()
    tr = Tracker(start_delta(args.start or args.out))
    if args.replay:
        for k, path in enumerate(args.replay):
            t0 = time.perf_counter()
            p = tr.step(cv2.imread(path), float(k) / args.rate)
            ms = 1000 * (time.perf_counter() - t0)
            sd, sm = tr.sigma()
            C = -cv2.Rodrigues(p[:3])[0].T @ p[3:6]
            w = cv2.projectPoints(np.array([[200.0, 100.0, 150.0]]), p[:3], p[3:6], K_of(p), None)[0].ravel()   # a point where the gripper works
            print(f"{os.path.basename(path)}: screws {sorted(tr.last['found'])} rejected {tr.last['rejected']}"
                  f"{' REACQUIRED' if tr.last['reacquired'] else ''} | centre {np.round(C, 1)} | work point px {np.round(w, 1)}"
                  f" | ±{sd:.2f}° ±{sm:.1f} mm | {ms:.0f} ms")
        return
    period = 1.0 / args.rate
    while True:
        t0 = time.time()
        try:
            st = json.loads(get(args.url + "/camera.json"))
            if st.get("running") and st.get("clients", 0) > 0:
                img = cv2.imdecode(np.frombuffer(get(args.url + "/snapshot.jpg"), np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    p = tr.step(img, time.time())
                    write_atomic(args.out, model_json(p, tr, img.shape[1], img.shape[0]))
        except Exception as e:
            print("tracker:", e, file=sys.stderr)
        time.sleep(max(0.05, period - (time.time() - t0)))


if __name__ == "__main__":
    main()
