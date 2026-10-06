"""
Make the 3D model for the Control page: convert the URDF's COLLADA meshes to small GLB files.
Output: website/public/robot/. The URDF copy that points to them comes from tools/gen_robot.py.

    python3 tools/web_meshes.py              # needs: pip install trimesh pycollada pillow; and Node (npx)
    python3 tools/web_meshes.py gripper_base # only the named meshes

Step 1 (trimesh) converts each .dae to .glb with its texture. Step 2 (gltf-transform) welds,
simplifies, quantizes and compresses (meshopt) each file. The browser decodes meshopt with three.js.
"""
import os
import re
import subprocess
import sys

import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "mycobot_description", "urdf", "mycobot_280_arduino")
OUT = os.path.join(ROOT, "website", "public", "robot")
URDFS = ["mycobot_280_arduino.urdf", "mycobot_280_arduino_gripper.urdf"]
SIMPLIFY_RATIO = "0.3"   # keep about 30 % of the triangles
SIMPLIFY_ERROR = "0.0005"  # relative to the mesh size


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
        scene.export(raw)
        subprocess.run(["npx", "--yes", "@gltf-transform/cli", "optimize", raw, out,
                        "--compress", "meshopt", "--simplify-ratio", SIMPLIFY_RATIO, "--simplify-error", SIMPLIFY_ERROR,
                        "--texture-compress", "webp", "--texture-size", "512"], check=True, stdout=subprocess.DEVNULL)
        os.remove(raw)
        print(f"{name}: {os.path.getsize(os.path.join(SRC, name + '.dae')) / 1e6:.1f} MB -> {os.path.getsize(out) / 1e3:.0f} kB")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    sys.exit(main())
