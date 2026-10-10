// Gamepad teleoperation of the TCP (an Xbox controller over Bluetooth, or any gamepad with the browser's
// "standard" mapping). The sticks give a twist of the TCP. The page sends it to the ATOM every 20 ms
// (end-effector JOG, firmware 5.1), and the ATOM turns it into joint goals at 500 Hz (twist.h), inside its
// own limits, with its 200 ms deadman. Docs: src/content/docs/software/gamepad.md.
import { parseChain, tcpPose, mulv, norm, type Chain, type Pose, type Vec3 } from './kinematics';
import * as P from './protocol';
import { GRIPPER_TCP_MM } from './robot_params';
import type { AtomLink } from './connection';

/** What the page gives to the gamepad. */
export interface TeleopHost {
  link: AtomLink;
  urdfUrl: string;
  /** Why the gamepad cannot start now ('' if it can). */
  cannotStart(): string;
  /** The gamepad starts or stops: the page stops Live mode, locks the faders and shows the tab. */
  onActive(active: boolean): void;
  /** Show the goals that the robot follows (faders, see-through arm). */
  showGoals(goalDeg: number[]): void;
  /** The Speed setting (°/s): the joint speed cap. */
  speed(): number;
  toast(text: string, kind: 'info' | 'warning'): void;
}

type Twist6 = [number, number, number, number, number, number];

// --- Inputs ---------------------------------------------------------------------------------------

/** The standard mapping (W3C Gamepad): an Xbox controller in Chrome, also over Bluetooth. */
const BTN = { A: 0, B: 1, X: 2, Y: 3, LB: 4, RB: 5, LT: 6, RT: 7, VIEW: 8, MENU: 9, LS: 10, RS: 11, UP: 12, DOWN: 13, LEFT: 14, RIGHT: 15 };

/** Stick and trigger values after the dead zone and the curve: sticks −1…1 (up and right +), triggers and bumpers 0…1. */
export interface Sticks {
  lx: number; ly: number; rx: number; ry: number;
  lt: number; rt: number; lb: number; rb: number;
}

const DEAD = 0.12; // stick dead zone (radial)
const TRIGGER_DEAD = 0.05;
// Fine control near the centre: 30 % linear, 70 % cubic.
const curve = (x: number) => 0.3 * x + 0.7 * x * x * x;

function stick(x: number, y: number): [number, number] {
  const r = Math.hypot(x, y);
  if (r < DEAD) return [0, 0];
  const k = curve(Math.min(1, (r - DEAD) / (1 - DEAD))) / r;
  return [x * k, y * k];
}

function readSticks(pad: Gamepad): Sticks {
  const [lx, ly] = stick(pad.axes[0] ?? 0, -(pad.axes[1] ?? 0));
  const [rx, ry] = stick(pad.axes[2] ?? 0, -(pad.axes[3] ?? 0));
  const trig = (i: number) => {
    const v = pad.buttons[i]?.value ?? 0;
    return v < TRIGGER_DEAD ? 0 : curve((v - TRIGGER_DEAD) / (1 - TRIGGER_DEAD));
  };
  const btn = (i: number) => (pad.buttons[i]?.pressed ? 1 : 0);
  return { lx, ly, rx, ry, lt: trig(BTN.LT), rt: trig(BTN.RT), lb: btn(BTN.LB), rb: btn(BTN.RB) };
}

// --- Mappings -------------------------------------------------------------------------------------

/** The controls in the diagram. */
type Ctl = 'LT' | 'RT' | 'LB' | 'RB' | 'LS' | 'RS' | 'DPAD' | 'A' | 'B' | 'X' | 'Y' | 'VIEW' | 'MENU';

/** A mapping gives the twist in units of the speed step: [vx, vy, vz, ωx, ωy, ωz], each −1…1, in the
 *  command frame. Base frame: x ahead, y to the left, z up. Tool frame: the flange axes (z = the approach).
 *  "Tilt" assumes the tool points down: the tip moves in that direction. `labels`: the diagram text of each
 *  control that the mapping uses (one or two lines). */
export interface Mapping {
  name: string;
  labels: Partial<Record<Ctl, string[]>>;
  twist(s: Sticks): Twist6;
}

export const MAPPINGS: Record<string, Mapping> = {
  twin: {
    name: 'Twin stick',
    labels: {
      LS: ['↕ Ahead / back (x)', '↔ Left / right (y)'],
      RS: ['↕ Tilt ahead / back (y)', '↔ Turn (z)'],
      LT: ['Down (z)'], RT: ['Up (z)'],
      LB: ['Tilt left (x)'], RB: ['Tilt right (x)'],
    },
    twist: (s) => [s.ly, -s.lx, s.rt - s.lt, s.lb - s.rb, -s.ry, -s.rx],
  },
  moveit: {
    name: 'MoveIt Servo',
    labels: {
      LS: ['↕ Rotate about y', '↔ Rotate about x'],
      RS: ['↕ Up / down (z)', '↔ Right / left (y)'],
      LT: ['Back (x)'], RT: ['Ahead (x)'],
      LB: ['Rotate about z −'], RB: ['Rotate about z +'],
    },
    twist: (s) => [s.rt - s.lt, -s.rx, s.ry, -s.lx, s.ly, s.rb - s.lb],
  },
  drone: {
    name: 'Drone (mode 2)',
    labels: {
      LS: ['↕ Up / down (z)', '↔ Turn (z)'],
      RS: ['↕ Ahead / back (x)', '↔ Right / left (y)'],
      LT: ['Tilt back (y)'], RT: ['Tilt ahead (y)'],
      LB: ['Tilt left (x)'], RB: ['Tilt right (x)'],
    },
    twist: (s) => [s.ry, -s.rx, s.ly, s.lb - s.rb, s.lt - s.rt, -s.lx],
  },
};

/** The buttons, the same in every mapping. */
const COMMON_LABELS: Partial<Record<Ctl, string[]>> = {
  A: ['Close the gripper'], B: ['Open the gripper'], X: ['Ready pose'], Y: ['Tool down'],
  DPAD: ['↑ ↓ Faster / slower'], VIEW: ['Base / tool frame'], MENU: ['Start / stop'],
};
const CTL_NAMES: Record<Ctl, string> = {
  LT: 'LT', RT: 'RT', LB: 'LB', RB: 'RB', LS: 'Left stick', RS: 'Right stick', DPAD: 'D-pad',
  A: 'A (hold)', B: 'B (hold)', X: 'X (hold)', Y: 'Y (hold)', VIEW: 'View', MENU: 'Menu',
};

/** The control that an input belongs to (to light it in the diagram). */
const INPUT_CTL: Record<string, Ctl> = {
  lx: 'LS', ly: 'LS', rx: 'RS', ry: 'RS', lt: 'LT', rt: 'RT', lb: 'LB', rb: 'RB',
  A: 'A', B: 'B', X: 'X', Y: 'Y', UP: 'DPAD', DOWN: 'DPAD', VIEW: 'VIEW', MENU: 'MENU',
};

// --- The diagram: the controller (Kenney's "Input Prompts" 1.5, Xbox Series/Vector/controller_xboxseries.svg,
// CC0, www.kenney.nl), scaled ×6, with our parts over its holes and a callout per control. Units: the SVG viewBox
// (700 × 300). Left labels end at x = 200, right labels start at x = 500; View and Menu are labelled above.
const SVGNS = 'http://www.w3.org/2000/svg';
const BODY = 'M29 32.5 Q29 31.05 28 30 27 29 25.5 29 L24.55 29.15 Q23.7 29.35 23.05 30 22 31.05 22 32.5 22 33.95 23.05 35 L24.55 35.9 25.5 36 Q27 36 28 35 29 33.95 29 32.5 M39 32 Q39 30.75 38.15 29.85 37.3 29 36 29 34.75 29 33.9 29.85 33 30.75 33 32 33 33.25 33.9 34.15 34.75 35 36 35 37.3 35 38.15 34.15 39 33.25 39 32 M41 25 Q41 24.15 40.4 23.55 39.85 23 39 23 38.15 23 37.6 23.55 37 24.15 37 25 37 25.8 37.6 26.4 38.15 27 39 27 39.85 27 40.4 26.4 41 25.8 41 25 M34 20 Q34 19.15 33.45 18.55 32.85 18 32 18 31.15 18 30.6 18.55 30 19.15 30 20 30 20.85 30.6 21.45 31.15 22 32 22 32.85 22 33.45 21.45 34 20.85 34 20 M24 23.5 Q24 22.05 23 21 22 20 20.5 20 19.05 20 18.05 21 17 22.05 17 23.5 17 24.95 18.05 26 19.05 27 20.5 27 22 27 23 26 24 24.95 24 23.5 M45 29 Q45 28.15 44.4 27.55 43.85 27 43 27 42.15 27 41.6 27.55 41 28.15 41 29 41 29.8 41.6 30.4 42.15 31 43 31 43.85 31 44.4 30.4 45 29.8 45 29 M49 25 Q49 24.15 48.4 23.55 47.85 23 47 23 46.15 23 45.6 23.55 45 24.15 45 25 45 25.8 45.6 26.4 46.15 27 47 27 47.85 27 48.4 26.4 49 25.8 49 25 M45 21 Q45 20.15 44.4 19.55 43.85 19 43 19 42.15 19 41.6 19.55 41 20.15 41 21 41 21.8 41.6 22.4 42.15 23 43 23 43.85 23 44.4 22.4 45 21.8 45 21 M23.6 38.65 Q22.1 38.65 20.9 39.6 20.15 40.15 19.5 41.1 L17.45 43.75 Q14.65 47.25 12.7 48 9.55 47.55 8.25 43.95 7.95 42.5 8 40.75 8.1 38 9.1 34.5 L9.35 33.55 Q10.2 30.3 11.35 27 L12.1 25 13.35 21.9 14.35 19.6 15.3 18.6 15.45 18.35 15.85 17.65 Q17.55 15.4 21.55 15 L23.5 15 24.25 15.85 39.7 15.85 40.5 15 42.45 15 Q46.45 15.4 48.1 17.65 L48.55 18.35 48.65 18.6 49.65 19.6 50.65 21.9 51.95 25 52.65 27 54.65 33.55 54.9 34.5 Q55.9 38 56 40.75 56.05 42.5 55.75 43.95 54.45 47.55 51.25 48 49.35 47.25 46.5 43.75 L44.5 41.1 43.1 39.6 Q41.9 38.65 40.4 38.65 L23.6 38.65';
const BODY_TRANSFORM = 'translate(158 0) scale(6)';
interface Part { shape: string; label: { x: number; y: number; anchor: 'start' | 'end' }; line: number[] }
// The triggers and bumpers sit behind the body, in its units (x mirrored about 32), so the body hides their lower
// edge: the bumper follows the shoulder, and the trigger shows above it.
const LT_SHAPE = 'M17.2 15.5 Q17.5 10.3 20.7 10 L22.5 10 Q23.8 10 23.8 11.4 L23.8 15 Z';
const LB_SHAPE = 'M15.45 18.35 Q16.9 13.5 21.6 13 L23.8 13 Q24.7 13 24.7 13.9 L24.7 16.5 L16.2 18.5 Z';
const mirror = (d: string) => d.replace(/(-?[\d.]+) (-?[\d.]+)/g, (_, x, y) => `${+(64 - +x).toFixed(2)} ${y}`);
const BEHIND: Partial<Record<Ctl, true>> = { LT: true, RT: true, LB: true, RB: true };
const PARTS: Record<Ctl, Part> = {
  LT: { shape: `body:${LT_SHAPE}`, label: { x: 200, y: 71, anchor: 'end' }, line: [204, 66, 276, 66] },
  RT: { shape: `body:${mirror(LT_SHAPE)}`, label: { x: 500, y: 71, anchor: 'start' }, line: [496, 66, 424, 66] },
  LB: { shape: `body:${LB_SHAPE}`, label: { x: 200, y: 95, anchor: 'end' }, line: [204, 90, 260, 93] },
  RB: { shape: `body:${mirror(LB_SHAPE)}`, label: { x: 500, y: 95, anchor: 'start' }, line: [496, 90, 440, 93] },
  LS: { shape: 'circle:281,141,18', label: { x: 200, y: 140, anchor: 'end' }, line: [204, 144, 263, 141] },
  RS: { shape: 'circle:374,192,15', label: { x: 500, y: 214, anchor: 'start' }, line: [496, 209, 388, 197] },
  DPAD: { shape: 'path:M304 178h14v10h10v14h-10v10h-14v-10h-10v-14h10z', label: { x: 200, y: 200, anchor: 'end' }, line: [204, 196, 294, 195] },
  // The right labels go top to bottom as the parts do; X's callout passes between Y and B.
  Y: { shape: 'circle:416,126,10', label: { x: 500, y: 119, anchor: 'start' }, line: [496, 114, 426, 124.5] },
  X: { shape: 'circle:392,150,10', label: { x: 500, y: 142, anchor: 'start' }, line: [496, 137, 428, 137, 401.5, 146.8] },
  B: { shape: 'circle:440,150,10', label: { x: 500, y: 165, anchor: 'start' }, line: [496, 160, 450, 152] },
  A: { shape: 'circle:416,174,10', label: { x: 500, y: 188, anchor: 'start' }, line: [496, 183, 426, 177] },
  VIEW: { shape: 'rect:319,143,14,8,4', label: { x: 322, y: 26, anchor: 'end' }, line: [326, 16, 326, 143] },
  MENU: { shape: 'rect:367,143,14,8,4', label: { x: 378, y: 26, anchor: 'start' }, line: [374, 16, 374, 143] },
};
const BUTTON_TEXT: Partial<Record<Ctl, string>> = { A: 'A', B: 'B', X: 'X', Y: 'Y' };

function el<K extends keyof SVGElementTagNameMap>(tag: K, attrs: Record<string, string | number>): SVGElementTagNameMap[K] {
  const e = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  return e;
}

/** Draw the diagram of a mapping into `svg`; returns the groups of each control (to light them). */
function drawDiagram(svg: SVGSVGElement, m: Mapping): Map<Ctl, SVGGElement[]> {
  svg.replaceChildren();
  const behind = el('g', {});
  svg.append(behind, el('path', { d: BODY, transform: BODY_TRANSFORM, class: 'pd-body' }), el('circle', { cx: 350, cy: 120, r: 9, class: 'pd-guide' }));
  const groups = new Map<Ctl, SVGGElement[]>();
  const labels = { ...COMMON_LABELS, ...m.labels };
  for (const [ctl, part] of Object.entries(PARTS) as [Ctl, Part][]) {
    const g = el('g', { class: 'pd-ctl' });
    const [kind, args] = part.shape.split(':');
    const n = args.split(',').map(Number);
    const shape = kind === 'rect' ? el('rect', { x: n[0], y: n[1], width: n[2], height: n[3], rx: n[4], class: 'pd-part' })
      : kind === 'circle' ? el('circle', { cx: n[0], cy: n[1], r: n[2], class: 'pd-part' })
      : kind === 'body' ? el('path', { d: args, transform: BODY_TRANSFORM, class: 'pd-part' })
      : el('path', { d: args, class: 'pd-part' });
    const gs = [g];
    if (BEHIND[ctl]) {
      const gb = el('g', { class: g.getAttribute('class')! });
      gb.append(shape);
      behind.append(gb);
      gs.push(gb);
    } else g.append(shape);
    if (BUTTON_TEXT[ctl]) g.append(Object.assign(el('text', { x: n[0], y: n[1] + 3.5, class: 'pd-key', 'text-anchor': 'middle' }), { textContent: BUTTON_TEXT[ctl] }));
    const text = labels[ctl];
    if (text) {
      const L = part.line;
      g.append(el('polyline', { points: L.join(' '), class: 'pd-line' }));
      const t = el('text', { x: part.label.x, y: part.label.y, 'text-anchor': part.label.anchor, class: 'pd-label' });
      text.forEach((line, i) => t.append(Object.assign(el('tspan', { x: part.label.x, dy: i ? 17 : 0 }), { textContent: line })));
      g.append(t);
    } else gs.forEach((x) => x.classList.add('pd-unused'));
    svg.append(g);
    groups.set(ctl, gs);
  }
  return groups;
}

/** Speed steps (D-pad up/down): TCP mm/s and °/s. */
export const SPEEDS = [
  { lin: 10, ang: 10 },
  { lin: 25, ang: 20 },
  { lin: 50, ang: 40 },
  { lin: 100, ang: 60 },
];

// --- The robot ------------------------------------------------------------------------------------

/** X (hold): the ready pose, in joint space. The elbow is bent (away from J3 = 0) and the tool points straight
 *  down, with the TCP 165 mm ahead of J1. From the zero pose (straight up, singular) the TCP cannot move
 *  down: go here first. */
export const READY = [0, 0, -90, 0, 0, 0];
/** The ATOM stops the arm 10° before the singular angles J3 = 0° and J5 = ±90° (twist.h). */
const KEEP_OUT = [
  { joint: 2, angles: [0], text: 'the arm is nearly straight (J3)' },
  { joint: 4, angles: [-90, 90], text: 'the wrist is near J5 = ±90°' },
];
const GRIPPER_DEG_S = 30; // J7 speed with A / B (the ATOM's JOG limit)
const LEVEL_GAIN = 3; // 1/s, Y: turn the tool to point down
const PERIOD_MS = 20;
const STILL_S = 0.3; // commanded, but the goals did not move for this long: say why

export function setupGamepad(host: TeleopHost) {
  const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
  const box = $<HTMLInputElement>('pad-on');
  const mapSel = $<HTMLSelectElement>('pad-map');
  const frameSeg = $('pad-frame');
  const aheadSeg = $('pad-ahead');
  const speedSeg = $('pad-speed');
  const nameEl = $('pad-name');
  const tcpEl = $('pad-tcp');
  const jspeedEl = $('pad-jspeed');
  const noteEl = $('pad-note');
  const rowsEl = $('pad-rows');
  const diagram = document.getElementById('pad-diagram') as unknown as SVGSVGElement;
  const { link } = host;

  const load = (k: string, d: string) => {
    try {
      return localStorage.getItem(`mycobot-control.${k}`) ?? d;
    } catch {
      return d;
    }
  };
  const save = (k: string, v: string) => {
    try {
      localStorage.setItem(`mycobot-control.${k}`, v);
    } catch {
      /* private window */
    }
  };
  for (const [key, m] of Object.entries(MAPPINGS)) mapSel.add(new Option(m.name, key));
  mapSel.value = MAPPINGS[load('pad-map', 'twin')] ? load('pad-map', 'twin') : 'twin';
  let frame: 'base' | 'tool' = load('pad-frame', 'base') === 'tool' ? 'tool' : 'base';
  let ahead = ['0', '90', '180', '-90'].includes(load('pad-ahead', '0')) ? Number(load('pad-ahead', '0')) : 0;
  let speedStep = 1; // every start: 25 mm/s
  SPEEDS.forEach((sp, i) => {
    const b = Object.assign(document.createElement('button'), { type: 'button', textContent: String(sp.lin), title: `TCP: ${sp.lin} mm/s, ${sp.ang} °/s` });
    b.dataset.v = String(i);
    speedSeg.append(b);
  });

  // The diagram of the mapping, and the same as a table for screen readers. A control lights up while in use.
  let groups = new Map<Ctl, SVGGElement[]>();
  function showRows() {
    const m = MAPPINGS[mapSel.value];
    groups = drawDiagram(diagram, m);
    diagram.setAttribute('aria-label', `Gamepad: ${m.name} mapping. The table below lists each control.`);
    rowsEl.replaceChildren();
    const labels = { ...m.labels, ...COMMON_LABELS };
    for (const ctl of Object.keys(PARTS) as Ctl[]) {
      const text = labels[ctl];
      if (!text) continue;
      const tr = document.createElement('tr');
      tr.append(Object.assign(document.createElement('td'), { textContent: CTL_NAMES[ctl] }), Object.assign(document.createElement('td'), { textContent: text.join('; ').replace(/[↕↔↑↓] ?/g, '') }));
      rowsEl.append(tr);
    }
  }
  function showSettings() {
    const press = (seg: HTMLElement, v: string) => seg.querySelectorAll<HTMLButtonElement>('button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.v === v)));
    press(frameSeg, frame);
    press(aheadSeg, String(ahead));
    press(speedSeg, String(speedStep));
    aheadSeg.querySelectorAll<HTMLButtonElement>('button').forEach((b) => (b.disabled = frame === 'tool'));
    frameSeg.title = frame === 'base'
      ? `Base frame: x ahead (the base's ${['+x', '+y', '−x', '−y'][[0, 90, 180, -90].indexOf(ahead)]} axis), y to the left, z up. Rotations turn about the TCP. View on the gamepad switches.`
      : 'Tool frame: the flange axes, z along the fingers. Rotations turn about the TCP. View on the gamepad switches.';
  }
  showRows();
  showSettings();
  mapSel.addEventListener('change', () => (save('pad-map', mapSel.value), showRows()));
  const onSeg = (seg: HTMLElement, f: (v: string) => void) =>
    seg.addEventListener('click', (e) => {
      const b = (e.target as HTMLElement).closest('button');
      if (b && !b.disabled && b.dataset.v !== undefined) (f(b.dataset.v), showSettings());
    });
  onSeg(frameSeg, (v) => save('pad-frame', (frame = v === 'tool' ? 'tool' : 'base')));
  onSeg(aheadSeg, (v) => ((ahead = Number(v)), save('pad-ahead', v)));
  onSeg(speedSeg, (v) => (speedStep = Number(v)));

  let chain: Chain | null = null;
  fetch(host.urdfUrl)
    .then((r) => r.text())
    .then((t) => (chain = parseChain(t)))
    .catch((e) => (noteEl.textContent = `No robot model: ${e}`));

  let active = false;
  let prev: boolean[] = [];
  let lastRumble = 0;
  let wasStopped = false;
  let note = '';
  let padName = '';
  let lastPose: Pose | null = null;
  let stillSince = 0;

  const getPad = (): Gamepad | null => {
    for (const p of navigator.getGamepads?.() ?? []) if (p?.connected && p.mapping === 'standard') return p;
    return null;
  };
  const firmwareOk = () => {
    const [a, b] = link.version.split('.').map(Number);
    return a > 5 || (a === 5 && b >= 1);
  };
  /** The TCP pose of the goals that the robot follows (or holds). */
  const goalPose = (): Pose | null => {
    const s = link.last;
    if (!chain || !s) return null;
    return tcpPose(chain, s.goal.slice(0, P.N_ARM), s.q.length > P.N_ARM ? GRIPPER_TCP_MM : [0, 0, 0]);
  };

  function start() {
    if (active) return;
    const why = !chain || !link.last ? 'connect to the robot first'
      : !link.inControl ? 'take control first'
      : !firmwareOk() ? `it needs controller firmware 5.1 or later (this robot has ${link.version}). Update it on the Setup page`
      : host.cannotStart();
    if (why) return host.toast(`Gamepad: ${why}.`, 'warning');
    active = true;
    wasStopped = false;
    lastPose = null;
    host.onActive(true);
    host.toast('Gamepad on: the sticks move the gripper. Menu or Esc stops.', 'warning');
  }

  /** Stop: HOLD, so the ATOM brakes now (not 0.2 s later, after the deadman). */
  function stop(why: string) {
    if (!active) return;
    active = false;
    note = '';
    link.holdPose();
    host.onActive(false);
    if (why) host.toast(`Gamepad off: ${why}.`, 'info');
  }

  function rumble(pad: Gamepad, ms: number) {
    const now = performance.now();
    if (now - lastRumble < 600) return;
    lastRumble = now;
    (pad as Gamepad & { vibrationActuator?: { playEffect?: (t: string, p: object) => Promise<unknown> } }).vibrationActuator
      ?.playEffect?.('dual-rumble', { duration: ms, strongMagnitude: 0.6, weakMagnitude: 0.4 })
      ?.catch(() => {});
  }

  box.addEventListener('change', () => (box.checked ? start() : stop('you switched it off')));
  window.addEventListener('gamepadconnected', (e) => {
    if (e.gamepad.mapping !== 'standard') host.toast(`Gamepad "${e.gamepad.id}" has no standard mapping: the page cannot use it.`, 'warning');
  });

  function tick() {
    const pad = getPad();
    padName = pad ? pad.id.replace(/\s*\(.*$/, '') : '';
    if (!pad) {
      if (active) stop('the gamepad disconnected');
      return showStatus(null);
    }
    const pressed = pad.buttons.map((b) => b.pressed);
    const edge = (i: number) => pressed[i] && !prev[i];
    if (edge(BTN.MENU)) (active ? stop('Menu') : start());
    if (edge(BTN.VIEW)) save('pad-frame', (frame = frame === 'base' ? 'tool' : 'base'));
    if (edge(BTN.UP)) speedStep = Math.min(SPEEDS.length - 1, speedStep + 1);
    if (edge(BTN.DOWN)) speedStep = Math.max(0, speedStep - 1);
    if ([BTN.VIEW, BTN.UP, BTN.DOWN].some(edge)) showSettings();
    prev = pressed;
    // Light the table rows of the inputs in use (also while stopped, to try a mapping out).
    const sticks = readSticks(pad);
    const lit = new Set<Ctl>();
    for (const [k, v] of Object.entries(sticks)) if (v !== 0) lit.add(INPUT_CTL[k]);
    for (const [k, i] of Object.entries(BTN)) if (pressed[i] && INPUT_CTL[k]) lit.add(INPUT_CTL[k]);
    groups.forEach((gs, ctl) => gs.forEach((g) => g.classList.toggle('pd-on', lit.has(ctl))));
    const s = link.last;
    const pose = goalPose();
    if (!active || !s || !pose) return showStatus(pad);
    host.showGoals(s.goal);
    const n = s.q.length;
    const vmax = host.speed();

    // X (hold): to the ready pose in joint space (TRACK; the ATOM switches the run to it), at half the Speed setting.
    if (pressed[BTN.X]) {
      link.send(P.track([...READY, ...s.goal.slice(P.N_ARM, n)], vmax / 2));
      const far = Math.max(...READY.map((r, j) => Math.abs(r - s.goal[j])));
      note = far > 0.1 ? 'to the ready pose (X)' : 'at the ready pose';
      lastPose = null;
      return showStatus(pad);
    }

    // The twist: mm/s and °/s, in the base frame turned by "Ahead", or in the tool frame (frame 2, on the ATOM).
    const sp = SPEEDS[speedStep];
    const u = MAPPINGS[mapSel.value].twist(sticks);
    let lin: Vec3 = [u[0] * sp.lin, u[1] * sp.lin, u[2] * sp.lin];
    let ang: Vec3 = [u[3] * sp.ang, u[4] * sp.ang, u[5] * sp.ang];
    let f: 1 | 2 = frame === 'tool' ? 2 : 1;
    if (frame === 'base') {
      const a = (ahead * Math.PI) / 180; // which base axis is "ahead" of the operator
      const rz = (v: Vec3): Vec3 => [Math.cos(a) * v[0] - Math.sin(a) * v[1], Math.sin(a) * v[0] + Math.cos(a) * v[1], v[2]];
      lin = rz(lin);
      ang = rz(ang);
    }
    if (pressed[BTN.Y]) {
      // Turn the approach axis (tool z) to point straight down, the shortest way (base frame).
      if (f === 2) {
        lin = mulv(pose.R, lin);
        ang = mulv(pose.R, ang);
        f = 1;
      }
      const z: Vec3 = [pose.R[2], pose.R[5], pose.R[8]];
      const axis: Vec3 = [-z[1], z[0], 0]; // z × (0, 0, −1)
      const th = Math.acos(Math.max(-1, Math.min(1, -z[2])));
      const k = norm(axis);
      if (k > 1e-6) {
        const w = Math.min((LEVEL_GAIN * th * 180) / Math.PI, sp.ang);
        ang = ang.map((x, i) => x + (axis[i] / k) * w) as Vec3;
      }
    }
    const j7 = n > P.N_ARM ? ((pressed[BTN.B] ? 1 : 0) - (pressed[BTN.A] ? 1 : 0)) * GRIPPER_DEG_S : 0; // open = toward 0°
    link.send(P.eeJog(f, lin, ang, j7, vmax));

    // Why the arm does not move: the ATOM stops it before a singular pose or a joint limit.
    const commanded = norm(lin) + norm(ang) > 1e-9;
    const now = performance.now() / 1000;
    const moved = !lastPose || norm(pose.p.map((x, i) => x - lastPose!.p[i])) > 0.01 || pose.R.some((x, i) => Math.abs(x - lastPose!.R[i]) > 2e-4);
    lastPose = pose;
    if (!commanded || moved) stillSince = now;
    const stopped = commanded && now - stillSince > STILL_S;
    note = stopped ? `stopped: ${whyStopped(s.goal)} (X: ready pose)` : '';
    if (stopped && !wasStopped) rumble(pad, 150);
    wasStopped = stopped;
    showStatus(pad, pose.p);
  }

  /** The likely reason the ATOM stopped the arm, from the goals. */
  function whyStopped(g: number[]): string {
    for (const k of KEEP_OUT) for (const a of k.angles) if (Math.abs(Math.abs(g[k.joint] - a) - 10) < 1) return k.text;
    for (let j = 0; j < P.N_ARM; j++)
      if (g[j] - P.LIMIT_MIN[j] < P.JOG_MARGIN + 1.5 || P.LIMIT_MAX[j] - g[j] < P.JOG_MARGIN + 1.5) return `J${j + 1} is at its limit`;
    return 'a singular pose in this direction';
  }

  function showStatus(pad: Gamepad | null, p?: Vec3) {
    const name = !pad ? 'No gamepad. Pair it with this computer, then press a button on it.' : `${padName} · ${active ? 'on' : 'off: Menu starts'}`;
    if (nameEl.textContent !== name) nameEl.textContent = name;
    p ??= goalPose()?.p;
    const tcp = p ? `TCP ${p.map((x) => x.toFixed(0)).join(', ')} mm` : 'TCP –';
    if (tcpEl.textContent !== tcp) tcpEl.textContent = tcp;
    const js = String(host.speed());
    if (jspeedEl.textContent !== js) jspeedEl.textContent = js;
    if (noteEl.textContent !== note) { noteEl.textContent = note; noteEl.title = note; }
    if (!pad) groups.forEach((gs) => gs.forEach((g) => g.classList.remove('pd-on')));
    box.checked = active;
    box.disabled = !pad && !active;
  }

  window.setInterval(tick, PERIOD_MS);
  return { stop, isActive: () => active };
}
