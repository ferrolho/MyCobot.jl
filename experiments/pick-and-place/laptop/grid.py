import sys
from PIL import Image, ImageDraw
src, dst = sys.argv[1], sys.argv[2]
x0, y0, x1, y1 = map(int, sys.argv[3:7]) if len(sys.argv) > 6 else (0, 0, 1280, 960)
step = int(sys.argv[7]) if len(sys.argv) > 7 else 50
im = Image.open(src).crop((x0, y0, x1, y1))
scale = max(1, 900 // max(x1 - x0, y1 - y0))
im = im.resize(((x1 - x0) * scale, (y1 - y0) * scale))
d = ImageDraw.Draw(im)
for x in range((x0 // step + 1) * step, x1, step):
    d.line([((x - x0) * scale, 0), ((x - x0) * scale, im.height)], fill=(255, 0, 0) if x % (2*step) == 0 else (0, 160, 255), width=1)
    d.text(((x - x0) * scale + 2, 2), str(x), fill=(255, 0, 0))
for y in range((y0 // step + 1) * step, y1, step):
    d.line([(0, (y - y0) * scale), (im.width, (y - y0) * scale)], fill=(255, 0, 0) if y % (2*step) == 0 else (0, 160, 255), width=1)
    d.text((2, (y - y0) * scale + 2), str(y), fill=(255, 0, 0))
im.save(dst)
