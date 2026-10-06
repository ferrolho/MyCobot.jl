"""Annotated timelapse: labels on the white wall (lower left), header (upper left)."""
import bisect, datetime as dt, os, subprocess, sys
from PIL import Image, ImageDraw, ImageFont

D = os.path.dirname(os.path.abspath(__file__))
FPS = 30
CAPTURE_FPS = 4.7

def at(hms):
    h, m, s = map(int, hms.split(':'))
    return dt.datetime(2026, 10, 6, h, m, s).timestamp()

# (start, title, lines) — ASD-STE100 style: short sentences, present tense, active voice.
STEPS = [
    ('17:47:00', 'THE TASK', ['Put 4 wine corks in the tissue box.', 'Corks: 24 mm wide, 38–49 mm long.', 'Cork A is next to the box wall.', 'The check stops it: do it last.']),
    ('17:52:00', '1. CORK C', ['Fingers across the cork.', 'Close in steps. Lift 80 mm.', 'Carry. Open.', 'First try: in the box.']),
    ('17:56:20', '2. CORK D', ['At the edge of the reach', '(281 mm from the base).', 'Carry low, at 50 mm.']),
    ('17:59:20', '2. CORK D', ['The wall check stops the carry.', 'Go through a waypoint.', 'First try: in the box.']),
    ('18:02:10', '3. CORK B', ['Next to a white board.', 'Attempt 1: the fingers land', 'on the end of the cork.']),
    ('18:05:00', '3. CORK B', ['Attempt 2: 12 mm along the cork.', 'Grip too high: it slips out.']),
    ('18:07:00', '3. CORK B', ['Attempt 3: 3 mm lower.', 'The camera stops (USB).', 'Continue without it.', 'In the box.']),
    ('18:10:30', '4. RESTART', ['Router and Pi restart.', 'The robot stays off WiFi.', 'Power cycle. Firmware fix:', 'join WiFi again (4.6.1).']),
    ('18:24:30', '5. CORK A', ['Move away from the base.', 'Attempts 1–2: the fingers', 'land on top of the cork.']),
    ('18:32:00', '5. CORK A: A TEST', ['Hover at two finger angles.', 'Model and image do not agree:', 'error 50 px (about 20 mm).']),
    ('18:34:10', '6. CALIBRATE AGAIN', ['The camera moved (new lamp).', '12 points near the table (9 used).', 'Error: 2.0 px.', 'Focal length: 1276 px (data: 1272).']),
    ('18:41:00', '6. CALIBRATE AGAIN', ['Check: the fingertip', 'is over the cork centre.']),
    ('18:42:30', '7. CORK A', ['Attempt 3: the sag correction', 'pushes the cork away.']),
    ('18:44:30', '7. CORK A', ['Attempts 4–5: correct the sag', 'above the cork.', 'A bug in the fix. Fixed.']),
    ('18:50:00', '7. CORK A', ['Attempt 6: fingers across,', '6 mm nearer the camera.', 'Grip holds. Lift. Carry.']),
    ('18:51:40', 'ALL 4 CORKS ARE IN THE BOX', ['C, D: first try. B: 3 tries.', 'A: next to a wall,', 'and the camera moved.', 'No learning. No demonstrations.']),
]
STARTS = [at(s[0]) for s in STEPS]

def font(size, bold=False):
    return ImageFont.truetype('/System/Library/Fonts/HelveticaNeue.ttc', size, index=1 if bold else 0)
F_TITLE, F_TEXT, F_HEAD, F_HEAD_B = font(25, True), font(23), font(19), font(19, True)

def draw(im, step, t, speed):
    d = ImageDraw.Draw(im, 'RGBA')
    title, lines = STEPS[step][1], STEPS[step][2]
    # label: lower left, on the wall
    x, y0 = 22, 600
    h = 40 + 31 * len(lines) + 16
    y = 940 - h
    d.rounded_rectangle([x - 12, y - 12, x + 372, 940], 10, fill=(255, 255, 255, 200))
    d.text((x, y), title, font=F_TITLE, fill=(20, 60, 140))
    for i, l in enumerate(lines):
        d.text((x, y + 40 + 31 * i), l, font=F_TEXT, fill=(25, 25, 25))
    # header: upper left
    clock = dt.datetime.fromtimestamp(t).strftime('%H:%M')
    d.rounded_rectangle([10, 10, 300, 72], 8, fill=(255, 255, 255, 200))
    d.text((20, 16), 'myCobot 280: corks into box', font=F_HEAD_B, fill=(25, 25, 25))
    d.text((20, 42), f'{clock}  ·  speed ×{speed:.1f}', font=F_HEAD, fill=(70, 70, 70))

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
