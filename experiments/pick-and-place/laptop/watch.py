"""Contact sheet of recorder frames between two Unix times (about one per second), to review a motion."""
import sys, os, glob
from PIL import Image, ImageDraw
d, t0, t1, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
step = float(sys.argv[5]) if len(sys.argv) > 5 else 1.0
fs = sorted(glob.glob(os.path.join(d, '*.jpg')))
ts = [int(os.path.basename(f)[:-4]) / 1000 for f in fs]
pick, nxt = [], t0
for f, t in zip(fs, ts):
    if t0 <= t <= t1 and t >= nxt: pick.append((f, t)); nxt = t + step
pick = pick[:40]
W, H = 320, 240; cols = 5
sheet = Image.new('RGB', (cols * W, H * ((len(pick) + cols - 1) // cols)))
for i, (f, t) in enumerate(pick):
    try: im = Image.open(f).resize((W, H))
    except Exception: continue
    ImageDraw.Draw(im).text((4, 4), f'{t - t0:5.1f}s', fill=(255, 255, 0))
    sheet.paste(im, ((i % cols) * W, (i // cols) * H))
sheet.save(out); print(len(pick), 'frames')
