"""
Make the 3D model for the Control page: convert the URDF's COLLADA meshes to small GLB files and
write a copy of the URDF that points to them. Output: website/public/robot/.

    python3 tools/web_meshes.py            # needs: pip install trimesh pycollada pillow; and Node (npx)

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
URDF = "mycobot_280_arduino.urdf"
SIMPLIFY_RATIO = "0.3"   # keep about 30 % of the triangles
SIMPLIFY_ERROR = "0.0005"  # relative to the mesh size


def main():
    os.makedirs(OUT, exist_ok=True)
    urdf = open(os.path.join(SRC, URDF)).read()
    meshes = sorted(set(re.findall(r'filename="package://mycobot_description/urdf/mycobot_280_arduino/([^"]+)\.dae"', urdf)))
    for name in meshes:
        raw = os.path.join(OUT, f"{name}.raw.glb")
        out = os.path.join(OUT, f"{name}.glb")
        scene = trimesh.load(os.path.join(SRC, f"{name}.dae"), force="scene")
        scene.export(raw)
        subprocess.run(["npx", "--yes", "@gltf-transform/cli", "optimize", raw, out,
                        "--compress", "meshopt", "--simplify-ratio", SIMPLIFY_RATIO, "--simplify-error", SIMPLIFY_ERROR,
                        "--texture-compress", "webp", "--texture-size", "512"], check=True, stdout=subprocess.DEVNULL)
        os.remove(raw)
        print(f"{name}: {os.path.getsize(os.path.join(SRC, name + '.dae')) / 1e6:.1f} MB -> {os.path.getsize(out) / 1e3:.0f} kB")
    # Same URDF, with relative paths to the GLB files.
    web = re.sub(r'package://mycobot_description/urdf/mycobot_280_arduino/([^"]+)\.dae', r"\1.glb", urdf)
    open(os.path.join(OUT, URDF), "w").write(web)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    sys.exit(main())
