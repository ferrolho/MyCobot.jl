"""
Make the 3D model for the Control page: convert the URDF's COLLADA meshes to small GLB files.
Output: website/public/robot/. The URDF copy that points to them comes from tools/gen_robot.py.

    python3 tools/web_meshes.py              # needs: pip install trimesh pycollada pillow; and Node (npx)
    python3 tools/web_meshes.py gripper_base # only the named meshes

Step 1 (trimesh) converts each .dae to .glb with its texture, and gives it smooth normals: Elephant's meshes have
one normal per face, so curved surfaces showed their facets (flat shading). Corners whose faces meet at less than
CREASE_DEG share an averaged normal; sharper edges stay sharp. Step 2 (gltf-transform) welds, quantizes
and compresses (meshopt) each file. It does not simplify: with welded vertices, simplification removed most
triangles and broke the shading at the joints (2026-10-09). The browser decodes meshopt with three.js.
"""
import os
import re
import subprocess
import sys

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "mycobot_description", "urdf", "mycobot_280_arduino")
OUT = os.path.join(ROOT, "website", "public", "robot")
URDFS = ["mycobot_280_arduino.urdf", "mycobot_280_arduino_gripper.urdf"]
CREASE_DEG = 30.0          # faces that meet at a smaller angle are shaded smoothly


def smooth(mesh):
    """The mesh with one vertex per face corner and crease-angle normals (the area-weighted mean of the normals of
    the faces at the same point that are within CREASE_DEG of the corner's face). gltf-transform welds the corners
    again where position, normal and texture coordinates agree."""
    f = mesh.faces
    corners = mesh.vertices[f].reshape(-1, 3)
    fn, area = mesh.face_normals, mesh.area_faces
    _, point = np.unique(np.round(corners, 7), axis=0, return_inverse=True)
    point = point.ravel()
    face = np.repeat(np.arange(len(f)), 3)
    order = np.argsort(point, kind="stable")
    starts = np.r_[0, np.flatnonzero(np.diff(point[order])) + 1, len(order)]
    normals = np.empty((len(corners), 3))
    cos = np.cos(np.radians(CREASE_DEG))
    for a, b in zip(starts[:-1], starts[1:]):
        c = order[a:b]                                  # the corners at one point
        n = fn[face[c]]
        w = (n @ n.T > cos) * area[face[c]][None, :]    # each corner: the faces within the crease angle
        normals[c] = w @ n
    normals /= np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12
    visual = mesh.visual
    if isinstance(visual, trimesh.visual.TextureVisuals) and visual.uv is not None:
        visual = trimesh.visual.TextureVisuals(uv=visual.uv[f].reshape(-1, 2), material=visual.material)
    elif isinstance(visual, trimesh.visual.TextureVisuals):
        visual = trimesh.visual.TextureVisuals(material=visual.material)
    else:
        visual = trimesh.visual.ColorVisuals(face_colors=visual.face_colors)
    return trimesh.Trimesh(corners, np.arange(len(corners)).reshape(-1, 3), vertex_normals=normals, visual=visual, process=False)


def main():
    os.makedirs(OUT, exist_ok=True)
    urdf = "".join(open(os.path.join(SRC, u)).read() for u in URDFS)
    meshes = sorted(set(re.findall(r'filename="package://mycobot_description/urdf/mycobot_280_arduino/([^"]+)\.dae"', urdf)))
    only = sys.argv[1:]   # optional: convert only these meshes, for example gripper_base
    if set(only) - set(meshes):
        sys.exit(f"not in the URDFs: {sorted(set(only) - set(meshes))}")
    for name in only or meshes:
        raw = os.path.join(OUT, f"{name}.raw.glb")
        out = os.path.join(OUT, f"{name}.glb")
        scene = trimesh.load(os.path.join(SRC, f"{name}.dae"), force="scene")
        for g in list(scene.geometry):
            scene.geometry[g] = smooth(scene.geometry[g])
        scene.export(raw, include_normals=True)
        subprocess.run(["npx", "--yes", "@gltf-transform/cli", "optimize", raw, out,
                        "--compress", "meshopt", "--simplify", "false",
                        "--texture-compress", "webp", "--texture-size", "512"], check=True, stdout=subprocess.DEVNULL)
        os.remove(raw)
        print(f"{name}: {os.path.getsize(os.path.join(SRC, name + '.dae')) / 1e6:.1f} MB -> {os.path.getsize(out) / 1e3:.0f} kB")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    sys.exit(main())
