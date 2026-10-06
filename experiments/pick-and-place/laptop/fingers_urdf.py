"""Finger geometry from the gripper URDF: mesh vertices of every gripper link in the joint6_flange frame (mm),
for a gripper opening (0..1000 -> gripper_controller -0.7..0.15 rad, mimic joints follow).
Usage: fingers_urdf.py  -> prints the lowest finger points (largest flange z) per opening and writes a table."""
import re, sys, json
import numpy as np
import xml.etree.ElementTree as ET

D = '/Users/henrique/myCobot/mycobot-280-lab/mycobot_description/urdf/mycobot_280_arduino/'
root = ET.parse(D + 'mycobot_280_arduino_gripper.urdf').getroot()


def rpy_xyz(o):
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = Rz @ Ry @ Rx; T[:3, 3] = xyz
    return T


def rot_axis(axis, a):
    k = np.array(axis, float); k /= np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4); T[:3, :3] = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    return T


joints = {}
for j in root.findall('joint'):
    joints[j.find('child').get('link')] = j


def link_T(link, theta):
    """joint6_flange -> link."""
    T = np.eye(4)
    chain = []
    while link in joints and link != 'joint6_flange':
        j = joints[link]; chain.append(j); link = j.find('parent').get('link')
        if link == 'joint6_flange':
            break
    for j in reversed(chain):
        T = T @ rpy_xyz(j.find('origin'))
        if j.get('type') == 'revolute':
            m = j.find('mimic')
            a = theta if j.get('name') == 'gripper_controller' else (theta * float(m.get('multiplier', 1)) + float(m.get('offset', 0)) if m is not None else 0.0)
            T = T @ rot_axis([float(v) for v in j.find('axis').get('xyz').split()], a)
    return T


def mesh_vertices(fn):
    s = open(D + fn).read()
    unit = re.search(r'<unit[^>]*meter="([0-9.eE-]+)"', s)
    arrs = re.findall(r'<float_array[^>]*id="([^"]*positions[^"]*)"[^>]*>([^<]*)</float_array>', s, re.I)
    if not arrs:
        arrs = re.findall(r'<float_array[^>]*id="([^"]*)"[^>]*>([^<]*)</float_array>', s)[:1]
    V = np.concatenate([np.array(a[1].split(), float).reshape(-1, 3) for a in arrs])
    return V, (float(unit.group(1)) if unit else 1.0)


links = {}
for l in root.findall('link'):
    vis = l.find('visual')
    if vis is None or not l.get('name').startswith('gripper'):
        continue
    mesh = vis.find('geometry/mesh')
    sc = float(mesh.get('scale', '1 1 1').split()[0])
    V, unit = mesh_vertices(mesh.get('filename').split('/')[-1])
    links[l.get('name')] = (rpy_xyz(vis.find('origin')), V * sc)


def finger_points(opening):
    theta = -0.7 + 0.85 * opening / 1000
    out = {}
    for name, (To, V) in links.items():
        T = link_T(name, theta) @ To
        P = (T[:3, :3] @ V.T).T + T[:3, 3]
        out[name] = P * 1000   # mm
    return out


if __name__ == '__main__':
    tab = {}
    for o in (0, 100, 200, 300, 400, 500, 700, 1000):
        F = finger_points(o)
        allp = np.concatenate(list(F.values()))
        k = np.argmax(allp[:, 2])
        tips = {n: P[np.argmax(P[:, 2])].round(1).tolist() for n, P in F.items() if n.endswith('1')}
        L, R = F['gripper_left1'], F['gripper_right1']
        def low(P, d=3.0):   # points within d mm of the finger's lowest point: the tip face
            return P[P[:, 2] > P[:, 2].max() - d]
        tl, tr = low(L), low(R)
        print(f"o {o:4d}: lowest z {allp[k, 2]:6.1f} at {allp[k].round(1)} | left tip x {tl[:,0].min():5.1f}..{tl[:,0].max():5.1f} y {tl[:,1].min():5.1f}..{tl[:,1].max():5.1f} | right tip x {tr[:,0].min():5.1f}..{tr[:,0].max():5.1f} y {tr[:,1].min():5.1f}..{tr[:,1].max():5.1f}")
        # tip-face corners (bounding box of the lowest 3 mm of each finger) for the Julia side
        corners = []
        for P in (tl, tr):
            for x in (P[:, 0].min(), P[:, 0].max()):
                for y in (P[:, 1].min(), P[:, 1].max()):
                    corners.append([round(float(x), 2), round(float(y), 2), round(float(P[:, 2].max()), 2)])
        tab[o] = corners
    json.dump(tab, open(sys.argv[1] if len(sys.argv) > 1 else '/dev/null', 'w'))
