"""The six visible brass screws on the top face of the base plate, at the plate holes of the CAD model
(G_base.dae; base frame = mesh + (0, 0, -32) mm: the plate is centred on J1, top face z = 0)."""
import numpy as np, cv2
HOLES = {'A': (65, -46), 'B': (47, -40), 'C': (47, 40), 'D': (65, 46), 'E': (-47, 40), 'F': (-65, 46)}
XYZ = {k: np.array([x, y, 0.0]) for k, (x, y) in HOLES.items()}

def refine(img, guess, win=6):
    """Centroid of the brass head near guess (pixels): yellowish hue, some saturation, not dark."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(float)
    u0, v0 = int(round(guess[0])), int(round(guess[1]))
    H, S, V = (hsv[v0 - win:v0 + win + 1, u0 - win:u0 + win + 1, i] for i in range(3))
    w = ((H > 8) & (H < 35) & (V > 70)) * np.clip(S - 40, 0, None) ** 2
    if w.sum() == 0: return None
    ys, xs = np.mgrid[-win:win + 1, -win:win + 1]
    return np.array([u0 + (w * xs).sum() / w.sum(), v0 + (w * ys).sum() / w.sum()])
