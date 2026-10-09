"""Kinematic calibration from point features: the holes in the gripper's palm (gripper_base, the flat face z = 0 in
the mesh: 9 holes in a 3 x 3 grid of 8 mm pitch, turned 45 degrees, and 3 screw holes, from the CAD). In each snapshot where the
palm faces the camera, find each hole as a dark dot near where the model puts it; then fit the joint zero offsets so
that the model's holes land on the dots (least squares, robust), the camera fixed (the base screws give it).

    uv run ... python kc_holes.py DATA_DIR TAG [TAG ...] --camera CAMERA_JSON [--offsets a,b,c,d,e,f] [--fit]
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

import kc_model as K

# Hole centres (x, y) in the gripper_base mesh (mm), on the palm face (z = 0): the grid (r 2.35 mm), then the screws (r 2.7 mm)
HOLES = np.array([[0.0, -19.0], [-5.66, -13.35], [5.66, -13.35], [-11.31, -7.69], [0.0, -7.69], [11.31, -7.69],
                  [-5.66, -2.03], [5.66, -2.03], [0.0, 3.62],
                  [-21.25, -20.02], [21.25, -20.02], [-0.5, 12.31]])
NAMES = [f"g{k}" for k in range(9)] + ["s1", "s2", "s3"]


def holes_link(robot):
    """Hole centres and the palm normal in the gripper_base link frame (metres), as the robot model places the mesh."""
    fn, Tv, sc = robot.visuals["gripper_base"]
    P = np.c_[HOLES, np.zeros(len(HOLES))] * sc        # mesh mm -> m (URDF scale)
    P = P @ Tv[:3, :3].T + Tv[:3, 3]
    n = Tv[:3, :3] @ np.array([0, 0, -1.0])            # the palm faces -z in the mesh
    return P, n


def find_blobs(g, box, r):
    """Dark discs of radius about r (px) on the light palm inside box (x0, y0, x1, y1): local maxima of an inverted
    difference of Gaussians tuned to r, sub-pixel by the weighted centroid."""
    x0, y0, x1, y1 = [int(v) for v in box]
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(g.shape[1], x1), min(g.shape[0], y1)
    if x1 - x0 < 20 or y1 - y0 < 20:
        return np.zeros((0, 2))
    w = g[y0:y1, x0:x1].astype(np.float32)
    s1 = max(1.0, r / np.sqrt(2))
    dog = cv2.GaussianBlur(w, (0, 0), 1.6 * s1) - cv2.GaussianBlur(w, (0, 0), s1)   # dark disc on light surroundings
    dog[cv2.GaussianBlur(w, (0, 0), 3 * s1) < 110] = 0                              # only on the light palm
    h = max(2, int(round(r)))
    peak = (dog == cv2.dilate(dog, np.ones((2 * h + 1, 2 * h + 1), np.uint8))) & (dog > 5)
    out = []
    yy, xx = np.mgrid[-h:h + 1, -h:h + 1]
    for by, bx in zip(*np.nonzero(peak)):
        if h <= by < dog.shape[0] - h and h <= bx < dog.shape[1] - h:
            sub = np.clip(dog[by - h:by + h + 1, bx - h:bx + h + 1], 0, None)
            out.append([x0 + bx + (sub * xx).sum() / sub.sum(), y0 + by + (sub * yy).sum() / sub.sum()])
    return np.array(out).reshape(-1, 2)


def match_pattern(pred, blobs, tol=2.5):
    """Match the predicted hole pattern to the blobs. The holes form a lattice, so a shift of one pitch also matches
    some holes: try the shift of every (hole, blob) pair, keep the one that matches the most holes, then refine with
    an affine map. Returns {hole index: blob}, or {} when the best shift is not clearly better than the next."""
    if len(blobs) < 4:
        return {}
    best = []
    for k in range(len(pred)):
        for z in blobs:
            d = np.linalg.norm(pred[:, None] + (z - pred[k]) - blobs[None], axis=2)
            best.append(((d.min(1) < tol).sum(), tuple(np.round(z - pred[k], 1))))
    best.sort(reverse=True)
    n1, t = best[0][0], np.array(best[0][1])
    n2 = max((n for n, tt in best if np.linalg.norm(np.array(tt) - t) > 4), default=0)
    if n1 < 6 or n1 - n2 < 2:
        return {}
    P = pred + t
    for _ in range(3):
        d = np.linalg.norm(P[:, None] - blobs[None], axis=2)
        m = {k: int(d[k].argmin()) for k in range(len(P)) if d[k].min() < tol}
        if len(m) < 4:
            return {}
        A = np.c_[pred[list(m)], np.ones(len(m))]
        M, *_ = np.linalg.lstsq(A, blobs[list(m.values())], rcond=None)
        P = np.c_[pred, np.ones(len(pred))] @ M
    d = np.linalg.norm(P[:, None] - blobs[None], axis=2)
    return {k: blobs[d[k].argmin()] for k in range(len(P)) if d[k].min() < 1.5}


def detect(robot, cam, poses, off):
    """Per pose: the holes found (index -> pixel), where the palm faces the camera."""
    Ph, nh = holes_link(robot)
    for p in poses:
        Ts = robot.link_transforms(p["q"], off)
        T = Ts["gripper_base"]
        W = Ph @ T[:3, :3].T + T[:3, 3]
        n = T[:3, :3] @ nh
        view = cam.C - W.mean(0)
        p["facing"] = float(n @ view / np.linalg.norm(view))
        p["holes"] = {}
        if p["facing"] < 0.35:
            continue
        g = cv2.cvtColor(cv2.imread(p["path"]), cv2.COLOR_BGR2GRAY)
        uv = cam.project(W)
        lo, hi = uv.min(0) - 30, uv.max(0) + 30
        depth = np.linalg.norm(W.mean(0) - cam.C)
        p["blobs"] = find_blobs(g, (*lo, *hi), cam.f * 0.00235 / depth)
        p["holes"] = match_pattern(uv, p["blobs"])


def residuals(x, robot, cam, poses):
    Ph, _ = holes_link(robot)
    out = []
    for p in poses:
        if len(p["holes"]) < 4:
            continue
        T = robot.link_transforms(p["q"], x[:6])["gripper_base"]
        uv = cam.project(Ph @ T[:3, :3].T + T[:3, 3])
        for k, z in p["holes"].items():
            out.append(uv[k] - z)
    return np.concatenate(out) if out else np.zeros(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("tags", nargs="+")
    ap.add_argument("--camera", required=True)
    ap.add_argument("--offsets", default="0,0,0,0,0,0", help="start offsets (deg)")
    ap.add_argument("--fit", action="store_true")
    args = ap.parse_args()
    robot = K.Robot(links=["gripper_base"])
    cam = K.Camera(model=json.load(open(args.camera)))
    poses = []
    for t in args.tags:
        for r in csv.DictReader(open(os.path.join(args.data, f"{t}.csv"))):
            path = os.path.join(args.data, r["name"] + ".jpg")
            if os.path.exists(path):
                poses.append(dict(name=r["name"], path=path, q=np.array([float(r[f"q{j}"]) for j in range(1, 7)])))
    x0 = np.array([float(v) for v in args.offsets.split(",")])
    detect(robot, cam, poses, x0)
    used = [p for p in poses if len(p["holes"]) >= 4]
    print(f"{len(poses)} poses, palm facing the camera in {sum(p['facing'] >= 0.35 for p in poses)}, 4+ holes in {len(used)}:",
          [(p["name"], len(p["holes"])) for p in used])
    r0 = residuals(x0, robot, cam, used).reshape(-1, 2)
    print(f"start offsets {x0}: hole error rms {np.sqrt((r0 ** 2).sum(1).mean()):.1f} px, median {np.median(np.linalg.norm(r0, axis=1)):.1f} px")
    if args.fit and len(used) >= 3:
        x = x0
        for _ in range(3):   # detect again near the improved prediction
            r = least_squares(residuals, x, args=(robot, cam, used), loss="soft_l1", f_scale=3.0, diff_step=0.01)
            x = r.x
            detect(robot, cam, used, x)
            used = [p for p in used if len(p["holes"]) >= 4]
        r1 = residuals(x, robot, cam, used).reshape(-1, 2)
        print(f"fitted offsets {np.round(x, 2)}: hole error rms {np.sqrt((r1 ** 2).sum(1).mean()):.1f} px, median {np.median(np.linalg.norm(r1, axis=1)):.1f} px")
        for p in used:      # per pose: mean error (shift) and the rest (shape)
            e = residuals(x, robot, cam, [p]).reshape(-1, 2)
            m = e.mean(0)
            print(f"  {p['name']}: {len(e):2d} holes, shift ({m[0]:5.1f},{m[1]:5.1f}) px, rest rms {np.sqrt(((e - m) ** 2).sum(1).mean()):.1f} px, facing {p['facing']:.2f}")


if __name__ == "__main__":
    main()
