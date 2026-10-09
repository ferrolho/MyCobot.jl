"""Kinematic calibration with the camera: the robot model (URDF + source meshes), its forward kinematics with
joint offsets, and the outline of each mesh as the lab camera sees it.

The joint angles are the Julia package's (atom_state: degrees, with the encoder correction). The camera
model is lab-camera.json (base frame, mm, OpenCV pinhole).
"""
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
URDF = os.path.join(REPO, "mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino_gripper.urdf")
ARM_JOINTS = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4",
              "joint6_to_joint5", "joint6output_to_joint6"]


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def tf(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def origin_tf(el):
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return tf(rpy_matrix(*rpy), xyz)


class Robot:
    def __init__(self, urdf=URDF, links=None, simplify=None):
        root = ET.parse(urdf).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            ax = j.find("axis")
            mim = j.find("mimic")
            self.joints[j.get("name")] = dict(
                type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                T=origin_tf(j), axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
                mimic=(mim.get("joint"), float(mim.get("multiplier", 1)), float(mim.get("offset", 0))) if mim is not None else None)
        self.parent_joint = {j["child"]: n for n, j in self.joints.items()}
        self.visuals = {}
        for l in root.findall("link"):
            v = l.find("visual")
            if v is None:
                continue
            m = v.find("geometry/mesh")
            if m is None:
                continue
            fn = m.get("filename").replace("package://mycobot_description", os.path.join(REPO, "mycobot_description"))
            sc = [float(s) for s in m.get("scale", "1 1 1").split()]
            self.visuals[l.get("name")] = (fn, origin_tf(v), np.array(sc))
        self.root = next(l for l in self.visuals if l not in self.parent_joint)
        self.meshes, self.tm = {}, {}
        for name, (fn, Tv, sc) in self.visuals.items():
            if links is not None and name not in links:
                continue
            m = trimesh.load(fn, force="scene").dump(concatenate=True)
            m.apply_scale(sc)            # metres
            m.apply_transform(Tv)        # in the link frame
            m.merge_vertices()
            if simplify and len(m.faces) > 2000:   # keep this fraction of the faces (fast_simplification)
                m = m.simplify_quadric_decimation(percent=1 - simplify)
                m.merge_vertices()
            self.tm[name] = m
            self.meshes[name] = Mesh(m)

    def sag_deg(self, q_deg, sag):
        """Gravity sag (deg) of J2 and J3: sag[k] (deg per 100 mm) times the horizontal reach from the joint
        to the wrist (joint6 origin), along the arm. A model with no masses: the moment grows with the reach."""
        T = self.link_transforms(q_deg, sag=None)
        w = T["joint6"][:3, 3]
        d = np.array([np.cos(np.radians(q_deg[0])), np.sin(np.radians(q_deg[0]))])
        out = np.zeros(6)
        for k, link in ((1, "joint3"), (2, "joint4")):     # J2 turns joint3, J3 turns joint4
            r = float(np.dot((w - T[link][:3, 3])[:2], d))
            out[k] = sag[k - 1] * r / 0.1
        return out

    def link_transforms(self, q_deg, offsets_deg=None, gripper=0.0, sag=None):
        """Base-frame (root link) transform of each link, metres. q: J1..J6 (deg); offsets added to q; with sag
        (deg per 100 mm for J2, J3), the gravity sag too."""
        qd = np.asarray(q_deg, float) + (0 if offsets_deg is None else np.asarray(offsets_deg, float))
        if sag is not None:
            qd = qd + self.sag_deg(qd, sag)
        q = np.radians(qd)
        vals = {n: q[k] for k, n in enumerate(ARM_JOINTS)}
        vals["gripper_controller"] = gripper
        out = {self.root: np.eye(4)}

        def T(link):
            if link in out:
                return out[link]
            jn = self.parent_joint[link]
            j = self.joints[jn]
            a = 0.0
            if j["type"] == "revolute":
                if j["mimic"]:
                    src, mul, off = j["mimic"]
                    a = vals.get(src, 0.0) * mul + off
                else:
                    a = vals.get(jn, 0.0)
            M = T(j["parent"]) @ j["T"] @ tf(axis_angle(j["axis"], a), [0, 0, 0])
            out[link] = M
            return M

        for l in self.visuals:
            T(l)
        return out


class Mesh:
    """Welded triangles, face planes and the edges with two faces (for the outline)."""

    def __init__(self, m):
        self.v = np.asarray(m.vertices, float)
        f = np.asarray(m.faces)
        n = np.cross(self.v[f[:, 1]] - self.v[f[:, 0]], self.v[f[:, 2]] - self.v[f[:, 0]])
        ok = np.linalg.norm(n, axis=1) > 1e-12
        f, n = f[ok], n[ok]
        self.n = n
        self.d = np.einsum("ij,ij->i", n, self.v[f[:, 0]])
        e = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
        fid = np.tile(np.arange(len(f)), 3)
        key = e[:, 0].astype(np.int64) * len(self.v) + e[:, 1]
        order = np.argsort(key, kind="stable")
        key, e, fid = key[order], e[order], fid[order]
        same = key[1:] == key[:-1]
        # keep edges seen exactly twice
        first = np.where(same)[0]
        cnt = np.diff(np.r_[0, np.where(np.r_[True, ~same, True][1:] if False else np.r_[~same, True])[0] + 1])
        starts = np.r_[0, np.where(~same)[0] + 1]
        counts = np.diff(np.r_[starts, len(key)])
        two = starts[counts == 2]
        self.e = e[two]
        self.f1, self.f2 = fid[two], fid[two + 1]

    def outline(self, cam_link):
        """Outline edges (end points, link frame) for a camera at cam_link (link frame, metres)."""
        front = self.n @ cam_link > self.d
        s = front[self.f1] != front[self.f2]
        return self.v[self.e[s, 0]], self.v[self.e[s, 1]]


class Camera:
    def __init__(self, path=None, model=None):
        m = model or json.load(open(path))
        import cv2
        self.R, _ = cv2.Rodrigues(np.array(m["rvec"], float))
        self.t = np.array(m["tvec"], float) / 1000      # metres
        self.f, self.cx, self.cy = m["f"], m["cx"], m["cy"]
        self.C = -self.R.T @ self.t                      # centre, base frame (m)

    def project(self, P):
        c = P @ self.R.T + self.t
        return np.c_[self.f * c[:, 0] / c[:, 2] + self.cx, self.f * c[:, 1] / c[:, 2] + self.cy]


def outline_points(robot, cam, Ts, links, step_px=2.0):
    """Pixel points sampled along the outline of the given links (no hidden-line removal)."""
    pts = []
    for l in links:
        T = Ts[l]
        c = np.linalg.inv(T) @ np.r_[cam.C, 1]
        a, b = robot.meshes[l].outline(c[:3])
        if not len(a):
            continue
        A = a @ T[:3, :3].T + T[:3, 3]
        B = b @ T[:3, :3].T + T[:3, 3]
        pa, pb = cam.project(A), cam.project(B)
        L = np.linalg.norm(pb - pa, axis=1)
        k = np.maximum(1, np.ceil(L / step_px)).astype(int)
        idx = np.repeat(np.arange(len(k)), k)
        s = (np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k) + 0.5) / np.repeat(k, k)
        pts.append(pa[idx] + s[:, None] * (pb[idx] - pa[idx]))
    return np.concatenate(pts) if pts else np.zeros((0, 2))
