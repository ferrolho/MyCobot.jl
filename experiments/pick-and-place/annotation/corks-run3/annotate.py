"""Annotated timelapse: labels on the white wall (lower left), header (upper left)."""
import bisect, datetime as dt, os, subprocess, sys
from PIL import Image, ImageDraw, ImageFont

D = os.path.dirname(os.path.abspath(__file__))
FPS = 60
CAPTURE_FPS = 4.7

def at(hms):
    h, m, s = map(int, hms.split(':'))
    return dt.datetime(2026, 10, 6, h, m, s).timestamp()

# (start, title, lines) — ASD-STE100 style: short sentences, present tense, active voice.
STEPS = [
    ('18:55:00', 'RUN 3: THE CORKS AGAIN', ['The user puts the 4 corks back', 'and walks the dog.', 'Claude works alone.']),
    ('19:05:40', '1. CORK 3: 5 TRIES', ['The fingers are along the', 'camera view: the weak direction.', 'A miss. Then a grip too high.']),
    ('19:09:00', '1. CORK 3', ['Lower is worse:', 'the fingertips touch the table.']),
    ('19:12:50', '1. CORK 3', ['Higher again: a miss.', 'The tries move the corks.', 'Stop. Take a new image.']),
    ('19:14:20', '2. CORK 1', ['The end centres are on the axis', '(z = −18 mm), not on the top.', 'Better estimate. Still a miss.']),
    ('19:17:30', '3. WHY THE MISSES?', ['The fingers are 18 mm from the model.', 'All calibration points had one', 'gripper angle. Now: 4 angles.', 'Found: the fingertip is 6.7 mm', 'off the flange axis. Error: 1.3 px.']),
    ('19:35:50', "5. SMOOTH GRASP (USER'S IDEA)", ['One close. The arm moves up', 'with the measured opening.', 'Cork 1: first try. In the box.']),
    ('19:37:40', '5. CORK 4', ['A finger lands on the cork.', 'Lift. Move 6 mm. Try again.']),
    ('19:39:50', '5. CORK 4', ['It holds. At the reach limit,', 'carry at 76 mm. In the box.']),
    ('19:42:40', '6. CORK 3', ['A finger lands on the cork.', 'Move 6 mm to the camera.', 'It holds. In the box.']),
    ('19:46:20', '7. CORK 2', ['First try.', 'A new path to the box.', 'In the box.']),
    ('19:49:00', 'ALL 4 CORKS ARE IN THE BOX', ['Smooth grasp: 6 tries, 4 corks.', 'No learning. No demonstrations.']),
]
STARTS = [at(s[0]) for s in STEPS]

def font(size, bold=False):
    return ImageFont.truetype('/System/Library/Fonts/HelveticaNeue.ttc', size, index=1 if bold else 0)
F_TITLE, F_TEXT, F_HEAD, F_HEAD_B, F_SPEED = font(25, True), font(23), font(19), font(19, True), font(30, True)

def draw(im, step, t, speed):
    d = ImageDraw.Draw(im, 'RGBA')
    title, lines = STEPS[step][1], STEPS[step][2]
    # label: lower left, on the wall
    x, y0 = 22, 600
    h = 40 + 31 * len(lines) + 16
    y = 940 - h
    w = max([d.textlength(title, font=F_TITLE)] + [d.textlength(l, font=F_TEXT) for l in lines])
    d.rounded_rectangle([x - 12, y - 12, x + max(372, w + 14), 940], 10, fill=(255, 255, 255, 200))
    d.text((x, y), title, font=F_TITLE, fill=(20, 60, 140))
    for i, l in enumerate(lines):
        d.text((x, y + 40 + 31 * i), l, font=F_TEXT, fill=(25, 25, 25))
    # header: upper left
    clock = dt.datetime.fromtimestamp(t).strftime('%H:%M')
    d.rounded_rectangle([10, 10, 300, 104], 8, fill=(255, 255, 255, 200))
    d.text((20, 16), 'myCobot 280: corks, run 3', font=F_HEAD_B, fill=(25, 25, 25))
    d.text((20, 42), clock, font=F_HEAD, fill=(70, 70, 70))
    d.text((20, 64), f'speed ×{speed:.0f}' if speed >= 2 else f'speed ×{speed:.1f}', font=F_SPEED, fill=(20, 60, 140))

def main(out, test=False):
    files = [l.strip() for l in open(os.path.join(D, 'kept_files.txt'))]
    times = [int(f[:-4]) / 1000 for f in files]
    groups = {}
    for f, t in zip(files, times):
        k = max(0, bisect.bisect_right(STARTS, t) - 1)
        groups.setdefault(k, []).append((f, t))
    plan = []   # (file, t, step, speed)
    for k in sorted(groups):
        g = groups[k]
        chars = len(STEPS[k][1]) + sum(len(l) for l in STEPS[k][2])
        need = int(FPS * (1.5 + chars / 14))
        rep = max(1, -(-need // len(g)))   # ceil
        speed = CAPTURE_FPS * 0 + (FPS / rep) / CAPTURE_FPS * 1.0
        speed = (FPS / rep) / CAPTURE_FPS
        for f, t in g:
            plan += [(f, t, k, speed)] * rep
    last = plan[-1]
    plan += [last] * (FPS * 4)   # hold the end
    print(len(plan), 'output frames,', round(len(plan) / FPS, 1), 's', file=sys.stderr)
    if test:
        for k in test:
            i = next(i for i, p in enumerate(plan) if p[2] == k)
            f, t, s, sp = plan[i]
            im = Image.open(os.path.join(D, 'frames', f)).convert('RGB'); draw(im, s, t, sp)
            im.save(os.path.join(D, f'test_{k}.jpg'))
        return
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '1280x960',
                           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-preset', 'medium', '-crf', '23', '-pix_fmt', 'yuv420p',
                           '-movflags', '+faststart', out], stdin=subprocess.PIPE)
    cache = (None, None)
    last_ok = None
    for f, t, s, sp in plan:
        if cache[0] != (f, s):
            try:
                im = Image.open(os.path.join(D, 'frames', f)).convert('RGB'); last_ok = f
            except Exception:   # a frame cut off when the Pi went down: repeat the last good one
                im = Image.open(os.path.join(D, 'frames', last_ok)).convert('RGB')
            draw(im, s, t, sp)
            cache = ((f, s), im.tobytes())
        ff.stdin.write(cache[1])
    ff.stdin.close(); ff.wait()

if __name__ == '__main__':
    if sys.argv[1] == 'test':
        main(None, test=[int(a) for a in sys.argv[2:]])
    else:
        main(sys.argv[1])
