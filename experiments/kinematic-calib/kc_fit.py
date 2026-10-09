"""Kinematic calibration with the camera, step 2: fit joint offsets (and the gripper's finger mapping) so that
the model's visible outline lies on the edges of the arm in each snapshot.

    uv run --with trimesh --with embreex --with fast_simplification --with rtree --with scipy \
        --with opencv-python --with pycollada --with networkx --with lxml python kc_fit.py DATA_DIR TAG

DATA_DIR holds TAG.csv (collect.jl) and the snapshots. Method (ICP-like): at the current estimate, take the
outline of each mesh as the camera sees it, keep the points that no other part hides (ray test), and fix
them in their link's frame; then fit the parameters so that these points land on image edges (Canny on a
contrast-enhanced image, distance map, clipped, robust loss). Repeat a few times.
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np
import trimesh
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

import kc_model as K

HERE = os.path.dirname(os.path.abspath(__file__))
CAMERA_URL = "http://100.69.15.110:8280/lab/camera.json"   # the lab camera model (the Pi's lab service)


def load_camera(src):
    """The camera model (lab-camera.json format) from a URL or a file."""
    if src.startswith("http"):
        import urllib.request
        with urllib.request.urlopen(src, timeout=5) as r:
            return json.load(r)
    return json.load(open(src))
ARM_LINKS = ["joint2", "joint3", "joint4", "joint5", "joint6", "gripper_base"]
FINGERS = ["gripper_left1", "gripper_left2", "gripper_left3", "gripper_right1", "gripper_right2", "gripper_right3"]
STATIC = ["g_base", "joint1"]
CLIP = 15.0
J7_MIN, J7_MAX = -51.5039, 0.0
G_LO, G_HI = -0.7, 0.15


def gripper_angle(j7, a=1.0, b=0.0):
    """The URDF finger joint for J7 (deg), as the 3D view maps it, times a and plus b (fitted)."""
    f = np.clip((j7 - J7_MIN) / (J7_MAX - J7_MIN), 0, 1)
    return a * (G_LO + f * (G_HI - G_LO)) + b


def edge_map(img, bg):
    """Edges of what differs from the background (the arm): Canny on the colour difference, and its distance map."""
    d = np.abs(img.astype(np.int16) - bg).max(axis=2).astype(np.float32)
    d = cv2.GaussianBlur(np.clip(d * 2, 0, 255).astype(np.uint8), (5, 5), 1.2)
    edges = cv2.Canny(d, 30, 80)
    return edges, cv2.distanceTransform(255 - edges, cv2.DIST_L2, 5)


def load(data, tags, bg_tags=None):
    """Poses of `tags`; the background is the median of the snapshots of `bg_tags` (default: every CSV in data)."""
    def rows(tag):
        return [r for r in csv.DictReader(open(os.path.join(data, f"{tag}.csv"))) if os.path.exists(os.path.join(data, r["name"] + ".jpg"))]
    bg_tags = bg_tags or sorted(f[:-4] for f in os.listdir(data) if f.endswith(".csv"))
    bg = np.median(np.stack([cv2.imread(os.path.join(data, r["name"] + ".jpg")) for t in bg_tags for r in rows(t)]), axis=0).astype(np.int16)
    poses = []
    for t in tags:
        for r in rows(t):
            img = cv2.imread(os.path.join(data, r["name"] + ".jpg"))
            edges, dt = edge_map(img, bg)
            poses.append(dict(name=r["name"], path=os.path.join(data, r["name"] + ".jpg"),
                              q=np.array([float(r[f"q{j}"]) for j in range(1, 7)]), j7=float(r["j7"]), edges=edges, dt=dt,
                              acc=np.array([float(r["ax"]), float(r["ay"]), float(r["az"])])))
    return poses


SAG = False   # set by --sag: x[6:8] are the J2, J3 sag coefficients (deg per 100 mm of reach)


def params(x):
    off = x[:6]
    a, b = (x[6], x[7]) if len(x) > 6 and not SAG else (1.0, 0.0)
    return off, a, b


def transforms(robot, p, x):
    off, a, b = params(x)
    return robot.link_transforms(p["q"], off, gripper_angle(p["j7"], a, b), sag=x[6:8] if SAG else None)


def visible_points(robot, cam, Ts, links, all_links, step_mm=1.0):
    """Outline points (3D, base frame) of `links` that no part of `all_links` hides, with their link names."""
    meshes = []
    for l in all_links:
        m = robot.tm[l].copy()
        m.apply_transform(Ts[l])
        meshes.append(m)
    scene = trimesh.util.concatenate(meshes)
    ray = trimesh.ray.ray_pyembree.RayMeshIntersector(scene) if trimesh.ray.has_embree else scene.ray
    out_p, out_l = [], []
    for l in links:
        T = Ts[l]
        c = np.linalg.inv(T) @ np.r_[cam.C, 1]
        a, b = robot.meshes[l].outline(c[:3])
        if not len(a):
            continue
        A = a @ T[:3, :3].T + T[:3, 3]
        B = b @ T[:3, :3].T + T[:3, 3]
        L = np.linalg.norm(B - A, axis=1) * 1000
        k = np.maximum(1, np.ceil(L / step_mm)).astype(int)
        idx = np.repeat(np.arange(len(k)), k)
        s = (np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k) + 0.5) / np.repeat(k, k)
        P = A[idx] + s[:, None] * (B[idx] - A[idx])
        d = P - cam.C
        dist = np.linalg.norm(d, axis=1)
        locs, ri, _ = ray.intersects_location(np.repeat(cam.C[None], len(P), 0), d / dist[:, None], multiple_hits=False)
        first = np.full(len(P), np.inf)
        first[ri] = np.linalg.norm(locs - cam.C, axis=1)
        vis = first >= dist - 0.0015
        out_p.append(np.c_[P[vis], np.ones(vis.sum())] @ np.linalg.inv(T).T)  # in the link frame
        out_l += [l] * int(vis.sum())
    if not out_p:
        return np.zeros((0, 4)), []
    return np.concatenate(out_p), out_l


def build(robot, cam, poses, x, links):
    """Per pose: the visible outline points in their link frames (fixed for the next fit)."""
    for p in poses:
        Ts = transforms(robot, p, x)
        P, L = visible_points(robot, cam, Ts, links, STATIC + ARM_LINKS + FINGERS)
        p["pts"], p["lnk"] = P, np.array(L)


IMU_W = 0.0   # set by --imu: weight of the IMU residual (px per rad of tilt); x[-3:] is the IMU mount (rotation vector)
IMU_LINK = "joint6"   # the ATOM (and its IMU) is on this link (fit 2026-10-09: 1.6° rms; other links: 30°+)


def imu_residuals(x, robot, poses):
    from scipy.spatial.transform import Rotation
    Rm = Rotation.from_rotvec(x[-3:]).as_matrix()
    out = []
    for p in poses:
        T = transforms(robot, p, x)[IMU_LINK]
        up = Rm.T @ (T[:3, :3].T @ np.array([0, 0, 1.0]))      # an accelerometer at rest reads +up
        out.append((up - p["acc"] / np.linalg.norm(p["acc"])) * IMU_W / np.sqrt(3 * len(poses)))
    return np.concatenate(out)


def residuals(x, robot, cam, poses):
    if IMU_W:
        return np.r_[camera_residuals(x[:-3], robot, cam, poses), imu_residuals(x, robot, poses)]
    return camera_residuals(x, robot, cam, poses)


def camera_residuals(x, robot, cam, poses):
    out = []
    for p in poses:
        Ts = transforms(robot, p, x)
        W = np.empty((len(p["pts"]), 3))
        for l in np.unique(p["lnk"]):
            m = p["lnk"] == l
            W[m] = (p["pts"][m] @ Ts[l].T)[:, :3]
        uv = cam.project(W)
        d = map_coordinates(p["dt"], [uv[:, 1], uv[:, 0]], order=1, mode="constant", cval=CLIP)
        out.append(np.minimum(d, CLIP) / np.sqrt(max(1, len(d))))
    return np.concatenate(out)


def score(robot, cam, poses, x, links):
    """Per pose: mean clipped distance (px) and the fraction of visible outline points within 3 px of an edge."""
    build(robot, cam, poses, x, links)
    res = []
    for p in poses:
        r = camera_residuals(x[:-3] if IMU_W else x, robot, cam, [p]) * np.sqrt(max(1, len(p["pts"])))
        res.append((float(np.mean(r)), float(np.mean(r < 3))))
    return res


def draw(robot, cam, p, x, links, path):
    img = cv2.imread(p["path"])
    img[p["edges"] > 0] = (img[p["edges"] > 0] * 0.4 + np.array([255, 255, 0]) * 0.6).astype(np.uint8)
    Ts = transforms(robot, p, x)
    P, L = visible_points(robot, cam, Ts, links, STATIC + ARM_LINKS + FINGERS, step_mm=0.5)
    for u, v in cam.project(P[:, :3] if len(P) else np.zeros((0, 3))).astype(int):
        pass
    # points are in link frames: transform back
    W = np.empty((len(P), 3))
    for l in set(L):
        m = np.array(L) == l
        W[m] = (P[m] @ Ts[l].T)[:, :3]
    for u, v in cam.project(W).astype(int):
        if 0 <= u < 1280 and 0 <= v < 960:
            img[v, u] = (0, 90, 255)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data")
    ap.add_argument("tags", nargs="+")
    ap.add_argument("--out", default=os.path.join(HERE, "fit.json"))
    ap.add_argument("--fingers", action="store_true", help="also fit the finger mapping (a, b)")
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--camera", default=CAMERA_URL, help="camera model: URL or lab-camera.json file (default: the Pi)")
    ap.add_argument("--eval", help="only score these parameters (comma-separated) against zero")
    ap.add_argument("--sag", action="store_true", help="also fit a gravity sag of J2 and J3")
    ap.add_argument("--imu", type=float, default=0.0, help="also fit the IMU (gravity direction), with this weight (px per rad)")
    args = ap.parse_args()
    global SAG, IMU_W
    SAG, IMU_W = args.sag, args.imu
    robot = K.Robot(links=STATIC + ARM_LINKS + FINGERS, simplify=0.3)
    cam = K.Camera(model=load_camera(args.camera))
    poses = load(args.data, args.tags)
    links = ARM_LINKS + FINGERS if args.fingers else ARM_LINKS
    x = np.r_[np.zeros(6), 1.0, 0.0] if args.fingers else np.zeros(8) if args.sag else np.zeros(6)
    if IMU_W:
        x = np.r_[x, np.radians([-89.2, -0.9, 3.3])]   # IMU mount: start from the IMU-only fit
    s0 = score(robot, cam, poses, x, links)
    print(f"{len(poses)} poses; before: mean {np.mean([s[0] for s in s0]):.2f} px, within 3 px {np.mean([s[1] for s in s0]):.2f}")
    if args.eval:
        xe = np.array([float(v) for v in args.eval.split(",")])
        se = score(robot, cam, poses, xe, links)
        print(f"with {np.round(xe, 2)}: mean {np.mean([v[0] for v in se]):.2f} px, within 3 px {np.mean([v[1] for v in se]):.2f}")
        print("per pose (zero -> these):", [(p["name"], round(a[1], 2), round(b[1], 2)) for p, a, b in zip(poses, s0, se)])
        return
    for k in range(args.rounds):
        build(robot, cam, poses, x, links)
        r = least_squares(residuals, x, args=(robot, cam, poses), diff_step=0.02, loss="soft_l1", f_scale=3.0, max_nfev=300)
        x = r.x
        s = score(robot, cam, poses, x, links)
        print(f"round {k + 1}: offsets {np.round(x[:6], 2)}" + (f" extra {np.round(x[6:], 3)}" if len(x) > 6 else "") +
              f"; mean {np.mean([v[0] for v in s]):.2f} px, within 3 px {np.mean([v[1] for v in s]):.2f}")
    json.dump(dict(offsets_deg=x[:6].tolist(), fingers=x[6:].tolist(), tags=args.tags, names=[p["name"] for p in poses],
                   before=s0, after=s), open(args.out, "w"), indent=1)
    os.makedirs(os.path.join(args.data, "fit"), exist_ok=True)
    x0 = np.zeros(len(x)) if args.sag else np.r_[np.zeros(6), 1.0, 0.0][: len(x)]
    if IMU_W:
        x0 = np.r_[x0[:-3] if len(x0) == len(x) else x0, x[-3:]]
    for p in poses:
        draw(robot, cam, p, x0, links, os.path.join(args.data, "fit", p["name"] + "_before.jpg"))
        draw(robot, cam, p, x, links, os.path.join(args.data, "fit", p["name"] + "_after.jpg"))
    print("per pose (before -> after, within 3 px):", [(n, round(a[1], 2), round(b[1], 2)) for n, a, b in zip([p["name"] for p in poses], s0, s)])


if __name__ == "__main__":
    main()
