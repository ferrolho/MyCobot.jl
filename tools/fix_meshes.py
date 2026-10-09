# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "trimesh", "manifold3d", "pillow", "scipy", "networkx", "lxml", "pycollada"]
# ///
"""
Our changes to Elephant's meshes in mycobot_description/urdf/mycobot_280_arduino. The tool reads Elephant's files
from Git LFS at commit 7cea001 (the same geometry as mycobot_ros, mycobot_280_m5 and mycobot_280_arduino) and
writes ours, so that every change is here and can be made again.

    uv run tools/fix_meshes.py               # write joint1.dae and joint6.dae
    uv run tools/web_meshes.py joint1 joint6 # then the Control page's .glb files

joint6.dae, the J6 housing (the link with the ATOM). Measured on this arm on 2026-10-09 (caliper and a side
photo; Elephant's newer STEP model "myCobot3.3" agrees within about 1 mm):
  - The ATOM end is 6.5 mm further from the J5 axis (21.6 -> 28.1 mm). The flange-side end is 3.7 mm nearer
    (35.1 -> 31.4 mm). The vertices beyond each end's cut move along the J6 axis (the link's y axis).
  - The faces between a moved and a fixed vertex get longer. Where such a face carries a decal (the USB-C
    port, the connector, the pin labels), the moved corners get new texture coordinates from the face's own
    mapping, so the decal keeps its size and shape.
joint1.dae, the base. The top cap has 6 screw holes 60° apart, at 31° + k·60° in the base frame, as on this arm.
  Elephant's has 8, 45° apart. The tool rebuilds the cap: its cross-section between two holes (the same at every
  angle), revolved, minus 6 holes of Elephant's shape (r 1.15 mm, a 45° countersink to r 2.02 mm, a counterbore)
  and Elephant's small hole in the inner ring.
"""
import io
import os
import re
import subprocess
import sys

import numpy as np
import trimesh
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR = "mycobot_description/urdf/mycobot_280_arduino"
ELEPHANT = "7cea001"   # the commit that added Elephant's meshes (Git LFS)

# joint6.dae: metres, the J6 axis is y, the J5 axis crosses it at y = 0
J6_CUTS = [(lambda y: y < -0.020, -0.0065),   # the ATOM end: 6.5 mm further from J5
           (lambda y: y > 0.025, -0.0037)]    # the flange-side end: 3.7 mm nearer J5

# joint1.dae: millimetres, the mesh is turned 180° about z in the URDF (visual rpy 0 0 -π)
CAP = "cc87c4f1-d741-4c69-a9bb-517982206b4b"   # the top cap of the base
SCREW_R = 34.55                                  # mm, the radius of the hole circle
SCREW_DEG_BASE = [31 + 60 * k for k in range(6)]  # this arm, base frame (camera, experiments/camera-calib)
HOLE = [(0, 69.0), (1.15, 69.0), (1.15, 73.7), (2.02, 74.57), (2.02, 75.6), (0, 75.6)]   # (r, z) mm, Elephant's
PIN = (20.41, [(0, 67.5), (1.19, 67.5), (1.19, 70.12), (0.81, 70.12), (0.81, 78.6), (0, 78.6)])   # Elephant's small
#   hole in the inner ring, on +x in the base frame: (distance from the axis, (r, z) profile) mm


def elephant(name):
    """Elephant's original file (bytes), from Git LFS."""
    ptr = subprocess.run(["git", "show", f"{ELEPHANT}:{DIR}/{name}"], cwd=ROOT, capture_output=True, check=True).stdout
    return subprocess.run(["git", "lfs", "smudge"], cwd=ROOT, input=ptr, capture_output=True, check=True).stdout


def get_array(s, aid):
    m = re.search(rf'<float_array id="{re.escape(aid)}"[^>]*>([^<]*)</float_array>', s)
    return np.array(m.group(1).split(), float)


def set_array(s, aid, values, stride):
    """Replace a float array and the count of its accessor."""
    text = " ".join(repr(round(float(x), 7)) for x in values.ravel())
    s = re.sub(rf'(<float_array id="{re.escape(aid)}"[^>]*count=")\d+("[^>]*>)[^<]*(</float_array>)',
               lambda m: f"{m.group(1)}{values.size}{m.group(2)}{text}{m.group(3)}", s)
    return re.sub(rf'(<accessor count=")\d+("[^>]*source="#{re.escape(aid)}")', rf"\g<1>{values.size // stride}\g<2>", s)


def fix_joint6(s, texture):
    pos_id, uv_id = "shape0-lib-positions-array", "shape0-lib-map-array"
    P = get_array(s, pos_id).reshape(-1, 3)
    T = get_array(s, uv_id).reshape(-1, 2)
    m = re.search(r"(<triangles count=\"\d+\"[^>]*>.*?<p>)([^<]*)(</p>)", s, re.S)
    idx = np.array(m.group(2).split(), int).reshape(-1, 3, 3)   # face, corner, (position, normal, texcoord)
    dy = np.zeros(len(P))
    for sel, d in J6_CUTS:
        dy[sel(P[:, 1])] = d
    Q = P.copy()
    Q[:, 1] += dy
    # the decals: dark texels; a face carries one if its texture triangle covers any
    img = np.asarray(texture.convert("L").resize((512, 512)))
    new_uv, fixed = [], 0
    for f in range(len(idx)):
        a, t = idx[f, :, 0].copy(), idx[f, :, 2].copy()
        if len(set(dy[a])) == 1:
            continue                                            # moved as a whole (or not at all)
        mask = Image.new("L", (512, 512))
        ImageDraw.Draw(mask).polygon([(u * 512, (1 - v) * 512) for u, v in T[t]], fill=1)
        if not (img[np.asarray(mask, bool)] < 200).any():
            continue                                            # plain: a longer face does not show
        E = np.c_[P[a[1]] - P[a[0]], P[a[2]] - P[a[0]]]         # the face's original mapping: x = P0 + E·(b1, b2)
        for k in range(3):
            if dy[a[k]] != 0:
                b = np.linalg.lstsq(E, Q[a[k]] - P[a[0]], rcond=None)[0]
                new_uv.append(T[t[0]] + b[0] * (T[t[1]] - T[t[0]]) + b[1] * (T[t[2]] - T[t[0]]))
                idx[f, k, 2] = len(T) + len(new_uv) - 1
        fixed += 1
    T = np.vstack([T, new_uv]) if new_uv else T
    s = s[:m.start(2)] + " ".join(map(str, idx.ravel())) + s[m.end(2):]   # before the arrays change length
    s = set_array(s, pos_id, Q, 3)
    s = set_array(s, uv_id, T, 2)
    print(f"joint6: moved {int((dy < 0).sum())} vertices; {fixed} decal faces kept their texture scale")
    return s


def revolve(profile, n):
    import manifold3d as mf
    return mf.Manifold.revolve(mf.CrossSection([np.asarray(profile, float)]), circular_segments=n)


def cap_profile(mesh, deg=22.5):
    """The cap's cross-section (r, z) at an angle between two holes, as one closed polygon."""
    d = np.array([np.cos(np.radians(deg)), np.sin(np.radians(deg)), 0])
    seg = trimesh.intersections.mesh_plane(mesh, np.array([-d[1], d[0], 0]), [0, 0, 0])
    seg = seg[(seg @ d > 0).all(1)]
    rz = np.round(np.stack([seg @ d, seg[:, :, 2]], -1), 4)
    nxt = {}
    for p, q in rz:
        nxt.setdefault(tuple(p), []).append(tuple(q))
        nxt.setdefault(tuple(q), []).append(tuple(p))
    start = next(iter(nxt))
    loop, prev, cur = [start], None, start
    while True:
        cand = [c for c in nxt[cur] if c != prev]
        cur, prev = cand[0], cur
        if cur == start:
            break
        loop.append(cur)
    if len(loop) != len(nxt):
        sys.exit(f"cap profile: {len(loop)} of {len(nxt)} points in one loop")
    loop = np.array(loop)
    area = 0.5 * np.sum(loop[:, 0] * np.roll(loop[:, 1], -1) - np.roll(loop[:, 0], -1) * loop[:, 1])
    return loop if area > 0 else loop[::-1]


def fix_joint1(s):
    scene = trimesh.load(io.BytesIO(s.encode()), file_type="dae", force="scene")
    cap = scene.geometry[next(g for g in scene.geometry if CAP in g)].copy()
    cap.merge_vertices()
    solid = revolve(cap_profile(cap), 96)
    hole = revolve(HOLE, 32)
    for deg in SCREW_DEG_BASE:
        a = np.radians(deg + 180)                               # the mesh is turned 180° in the URDF
        solid = solid - hole.translate([SCREW_R * np.cos(a), SCREW_R * np.sin(a), 0])
    solid = solid - revolve(PIN[1], 24).translate([-PIN[0], 0, 0])   # +x in the base frame is -x in the mesh
    out = solid.to_mesh()
    V = np.asarray(out.vert_properties)[:, :3]
    F = np.asarray(out.tri_verts)
    N = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    N /= np.linalg.norm(N, axis=1)[:, None]
    s = set_array(s, f"lib-position-array-{CAP}", V, 3)
    s = set_array(s, f"lib-normal-array-{CAP}", N, 3)
    s = set_array(s, f"lib-map1-array-{CAP}", np.zeros((1, 2)), 2)   # the cap has no texture
    corners = np.stack([F, np.repeat(np.arange(len(F))[:, None], 3, 1), np.zeros_like(F)], -1)
    g0 = s.index(f'<geometry id="mesh-{CAP}"')
    g1 = s.index("</geometry>", g0)
    g = s[g0:g1]
    g = re.sub(r'(<triangles count=")\d+(")', rf"\g<1>{len(F)}\g<2>", g)
    g = re.sub(r"<p>[^<]*</p>", "<p>" + " ".join(map(str, corners.ravel())) + "</p>", g)
    print(f"joint1: top cap rebuilt with {len(SCREW_DEG_BASE)} screw holes ({len(F)} faces; Elephant's: {len(cap.faces)})")
    return s[:g0] + g + s[g1:]


def main():
    path = os.path.join(ROOT, DIR)
    j6 = fix_joint6(elephant("joint6.dae").decode(), Image.open(io.BytesIO(elephant("joint6.png"))))
    open(os.path.join(path, "joint6.dae"), "w").write(j6)
    j1 = fix_joint1(elephant("joint1.dae").decode())
    open(os.path.join(path, "joint1.dae"), "w").write(j1)


if __name__ == "__main__":
    sys.exit(main())
