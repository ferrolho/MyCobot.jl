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
    ('15:33:00', 'THE TASK', ['Put the toy in the tissue box.', 'Data: camera, joint angles,', 'IMU and gripper.', 'No learning. No demonstrations.']),
    ('15:34:15', '1. MOVE THE ARM UP', ['Now the camera sees', 'the toy and the box.']),
    ('15:42:00', '2. CALIBRATE THE CAMERA', ['The gripper goes to', '9 known points.', 'Near the base, the tip', 'is hard to see.']),
    ('15:52:30', '2. CALIBRATE THE CAMERA', ['Low points over the table:', 'the tip is clear on the wood.']),
    ('15:55:00', '2. CALIBRATE THE CAMERA', ['16 points. A script finds', 'the tip in each image.', 'Error: 3.7 px (about 2 mm).']),
    ('16:02:00', '3. FIND THE TABLE', ['The tip goes down', 'in small steps.', 'A joint shift looks like', 'a contact. It is not.']),
    ('16:05:30', '3. FIND THE TABLE', ['Record the IMU at 500 Hz', 'during each step.']),
    ('16:07:00', '3. FIND THE TABLE', ['Turn the gripper 180°.', 'This frees the cable.']),
    ('16:08:20', '3. FIND THE TABLE', ['Go down again.', 'Watch the IMU.']),
    ('16:09:30', '3. FIND THE TABLE', ['Impact: 0.7 g (normal 0.15 g).', 'Table height: z = −30 mm.']),
    ('16:10:00', '4. FIND THE TOY', ['Park the arm.', 'Find the toy in the image.', 'A leg is 20 mm wide:', 'it fits in the gripper.']),
    ('16:16:00', '5. ATTEMPT 1: A LEG', ['Open the gripper', 'above the leg.']),
    ('16:18:40', '5. ATTEMPT 1: A LEG', ['Turn the gripper.', 'Check the finger direction.']),
    ('16:25:00', '5. ATTEMPT 1: A LEG', ['Close. Miss.', 'The leg is 20–25 mm', 'nearer the camera.', 'One camera: depth is hard.']),
    ('16:29:30', '5. ATTEMPT 2', ['Move 12 mm nearer', 'the camera.']),
    ('16:32:00', '5. ATTEMPT 3: OTHER LEG', ['Fingers along the camera view.', 'Grip! But the leg slips out', 'during the lift.']),
    ('16:35:40', '5. ATTEMPT 4: GRIP LOWER', ['The fingers move down', 'when they close.', 'They push the leg out.']),
    ('16:39:40', '6. ATTEMPTS 5–6: THE HEAD', ['(The user\'s idea.)', 'The fingers touch the table.', 'J5 gets hot (66 °C). Lift.']),
    ('16:47:20', '6. ATTEMPT 7: THE HEAD', ['14 mm higher.', 'Close in steps. At each step,', 'lift the arm a little.', 'Grip holds. Slips at 35 mm.']),
    ('16:52:00', '6. ATTEMPT 8: MORE FORCE', ['Gripper force: 30 % → 60 %.', 'Slips at 20 mm.']),
    ('16:59:05', '6. ATTEMPT 9', ['Miss.', 'Found: each small step adds', 'the sag of the arm (13 mm).', 'Fixed in the code.']),
    ('17:03:30', '7. POKE THE TOY', ['(The user\'s idea.)', 'Push from behind', 'in 4 mm steps.', 'Step 11: the camera sees it move.', 'Now the depth is known.']),
    ('17:06:30', '8. ATTEMPT 11', ['Aim with the new depth.', 'Grip holds. Slips at 30 mm.']),
    ('17:08:50', '8. ATTEMPT 12: FULL FORCE', ['Gripper force: 100 %.', 'Holds to 70 mm.', 'The toy is almost in the air.']),
    ('17:11:50', '8. ATTEMPT 13', ['The toy has turned.', 'Better view of the head.', 'Grip. Lift 120 mm.']),
    ('17:15:30', '9. CARRY AND OPEN', ['Carry it to the box.', 'Open the gripper.']),
    ('17:15:46', 'THE COW IS IN THE BOX', ['13 attempts in 1 h 44 min.', 'No learning. No demonstrations.', 'Tools: camera, kinematics,', 'IMU and gripper.', 'Method: try, measure, change.']),
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
    d.rounded_rectangle([10, 10, 262, 72], 8, fill=(255, 255, 255, 200))
    d.text((20, 16), 'myCobot 280: toy into box', font=F_HEAD_B, fill=(25, 25, 25))
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
    for f, t, s, sp in plan:
        if cache[0] != (f, s):
            im = Image.open(os.path.join(D, 'frames', f)).convert('RGB'); draw(im, s, t, sp)
            cache = ((f, s), im.tobytes())
        ff.stdin.write(cache[1])
    ff.stdin.close(); ff.wait()

if __name__ == '__main__':
    if sys.argv[1] == 'test':
        main(None, test=[int(a) for a in sys.argv[2:]])
    else:
        main(sys.argv[1])
