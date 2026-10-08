#!/usr/bin/env python3
"""Re-register the camera from the six brass screws on the robot's base plate (no tape, no arm motion).

  python3 rereg_screws.py [NEW.jpg]            # no file: take a snapshot from the Pi
        [--save]                               # write the new pose to ../current_cam.npy (keeps a backup)

The screws sit in the plate holes of the CAD model (G_base.dae). The lens values (f, centre) are fixed
(2026-10-08 fit: screws + tape + fingertips). Steps: find the plate as a whole (coarse shift), find each
screw with a template near its predicted position, solve the camera pose with PnP. Needs 4+ screws."""
import os, sys, time, subprocess, shutil, numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from screws import XYZ
REF = os.path.join(ROOT, 'reference')

def K_of(p): return np.array([[p[6], 0, p[7]], [0, p[6], p[8]], [0, 0, 1.0]])

def proj(p, X): return cv2.projectPoints(np.asarray(X, float).reshape(-1, 3), p[:3], p[3:6], K_of(p), None)[0].reshape(-1, 2)

def make_reference(img_path, px):
    """Templates of the screws (31x31) and of the plate (to find big camera moves)."""
    g = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2GRAY)
    T = {k: g[int(round(v)) - 15:int(round(v)) + 16, int(round(u)) - 15:int(round(u)) + 16] for k, (u, v) in px.items()}
    us = [u for u, v in px.values()]; vs = [v for u, v in px.values()]
    box = (int(min(us)) - 40, int(min(vs)) - 40, int(max(us)) + 30, int(max(vs)) + 40)
    np.savez(os.path.join(REF, 'screw_templates.npz'), box=box, plate=g[box[1]:box[3], box[0]:box[2]],
             px=np.array([px[k] for k in sorted(px)]), keys=np.array(sorted(px)), **{f't_{k}': T[k] for k in T})

def subpix(r, x, y):
    def para(a, b, c): d = a - 2 * b + c; return 0.0 if d == 0 else 0.5 * (a - c) / d
    dx = para(r[y, x - 1], r[y, x], r[y, x + 1]) if 0 < x < r.shape[1] - 1 else 0
    dy = para(r[y - 1, x], r[y, x], r[y + 1, x]) if 0 < y < r.shape[0] - 1 else 0
    return x + dx, y + dy

def register(img, cam):
    R = np.load(os.path.join(REF, 'screw_templates.npz'))
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 1. coarse: where is the plate now? (search the whole image)
    pad = 200   # the plate may be partly out of the image after a camera move
    gp = cv2.copyMakeBorder(g, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    m = cv2.matchTemplate(gp, R['plate'], cv2.TM_CCOEFF_NORMED); _, sc, _, loc = cv2.minMaxLoc(m)
    shift = np.array(loc) - pad - np.array(R['box'][:2])
    # 2. each screw: template match in a window around its reference position + shift
    found = {}
    for k, (u0, v0) in zip(R['keys'], R['px']):
        T = R[f't_{k}']; u, v = u0 + shift[0], v0 + shift[1]; w = 25
        x0, y0 = max(0, int(u) - 15 - w), max(0, int(v) - 15 - w)
        x1, y1 = min(g.shape[1], int(u) + 16 + w), min(g.shape[0], int(v) + 16 + w)
        if x1 - x0 < 40 or y1 - y0 < 40: continue
        r = cv2.matchTemplate(g[y0:y1, x0:x1], T, cv2.TM_CCOEFF_NORMED)
        _, s, _, (bx, by) = cv2.minMaxLoc(r)
        if s < 0.6: continue
        sx, sy = subpix(r, bx, by); found[str(k)] = (x0 + sx + 15, y0 + sy + 15, s)
    good = {k: v for k, v in found.items() if v[2] >= 0.8}
    if len(good) >= 4: found = good          # weak matches (partly hidden screws) only if needed
    if len(found) < 4: raise SystemExit(f'only {len(found)} screws found (plate match {sc:.2f}); is the base in view?')
    keys = sorted(found); O = np.array([XYZ[k] for k in keys]); I = np.array([found[k][:2] for k in keys])
    R0, _ = cv2.Rodrigues(cam[:3]); C0 = -R0.T @ cam[3:6]
    def solve(O, I):
        if len(O) >= 6:   # full pose
            ok, rv, tv = cv2.solvePnP(O, I, K_of(cam), None, cam[:3].copy(), cam[3:6].copy(), True, cv2.SOLVEPNP_ITERATIVE)
            new = cam.copy(); new[:3], new[3:6] = rv.ravel(), tv.ravel()
        else:             # fewer screws: rotation only (a bump turns the camera about its mount; the centre stays)
            from scipy.optimize import least_squares
            def res(r):
                Rm, _ = cv2.Rodrigues(r); t = -Rm @ C0
                return (cv2.projectPoints(O, r, t, K_of(cam), None)[0].reshape(-1, 2) - I).ravel()
            r = least_squares(res, cam[:3].copy()).x; Rm, _ = cv2.Rodrigues(r)
            new = cam.copy(); new[:3], new[3:6] = r, -Rm @ C0
        return new, np.linalg.norm(proj(new, O) - I, axis=1)
    new, e = solve(O, I)
    # a screw hidden by the arm or matched to the hole next to it: drop the worst while the fit is poor
    while np.sqrt((e ** 2).mean()) > 2.0 and len(keys) > 4:
        w = int(np.argmax(e)); found.pop(keys[w]); keys.pop(w); O = np.delete(O, w, 0); I = np.delete(I, w, 0)
        new, e = solve(O, I)
    return new, found, e, sc, shift

def T_of(p):
    R, _ = cv2.Rodrigues(p[:3]); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p[3:6]; return T

def p_of(T, like):
    out = like.copy(); out[:3] = cv2.Rodrigues(T[:3, :3])[0].ravel(); out[3:6] = T[:3, 3]; return out

def relative(img, cur):
    """Camera motion M from the screws (screw-only pose now vs in the reference image), applied to the
    accurate reference pose: T_new = M T_ref. The reference pair is stored next to the templates."""
    ref = np.load(os.path.join(REF, 'pose_ref.npz'))
    s_new, found, e, sc, shift = register(img, ref['screws'])
    M = T_of(s_new) @ np.linalg.inv(T_of(ref['screws']))
    return p_of(M @ T_of(ref['accurate']), ref['accurate']), found, e, sc, shift

def camera_moved(a, b):
    Ra, _ = cv2.Rodrigues(a[:3]); Rb, _ = cv2.Rodrigues(b[:3])
    Ca, Cb = -Ra.T @ a[3:6], -Rb.T @ b[3:6]
    return np.linalg.norm(Cb - Ca), np.degrees(np.linalg.norm(cv2.Rodrigues(Rb @ Ra.T)[0]))

if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if args: path = args[0]
    else:
        path = os.path.join(ROOT, 'snapshots', time.strftime('%Y%m%d-%H%M%S') + '.jpg'); os.makedirs(os.path.dirname(path), exist_ok=True)
        subprocess.run(['ssh', 'raspberrypi5', 'curl -s -m 10 http://127.0.0.1:8280/snapshot.jpg'], stdout=open(path, 'wb'), check=True)
    cur_path = os.path.join(ROOT, 'current_cam.npy'); cur = np.load(cur_path)
    new, found, e, sc, shift = relative(cv2.imread(path), cur)
    d, a = camera_moved(cur, new)
    print(f"{len(found)} screws ({', '.join(f'{k} {v[2]:.2f}' for k, v in found.items())}), plate shift {shift.tolist()} px; "
          f"reprojection {np.sqrt((e ** 2).mean()):.2f} px rms, max {e.max():.2f}; camera moved {d:.1f} mm, turned {a:.2f}°")
    if '--save' in sys.argv:
        if np.sqrt((e ** 2).mean()) > 2.5: raise SystemExit('not saved: reprojection too large')
        shutil.copy(cur_path, cur_path.replace('.npy', time.strftime('_before_%Y%m%d-%H%M%S.npy'))); np.save(cur_path, new)
        print('saved', cur_path)
