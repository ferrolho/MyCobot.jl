import sys, numpy as np
from PIL import Image
a, b = sys.argv[1], sys.argv[2]
x0, y0, x1, y1 = map(int, sys.argv[3:7])
A = np.asarray(Image.open(a).convert("L").crop((x0, y0, x1, y1)), float)
B = np.asarray(Image.open(b).convert("L").crop((x0, y0, x1, y1)), float)
A -= A.mean(); B -= B.mean()          # ignore exposure changes
print(round(float(np.abs(A - B).mean()), 2))
