// The Control page: connection, joint strips (state, jog, goal fader), six plots, and the controls.
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { AtomLink } from './connection';
import * as P from './protocol';
import type { Stream } from './protocol';
import type { ArmView } from './viewer3d';

const $ = <T extends HTMLElement>(sel: string, root: ParentNode = document) => root.querySelector(sel) as T;
const fmt = (x: number | null, d = 1) => (x !== null && Number.isFinite(x) ? x.toFixed(d) : '–');

const HISTORY_KEY = 'mycobot-control.addresses';
const WINDOW_S = 20; // plot window
const MAX_POINTS = WINDOW_S * 50 + 10;
const JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

// Six plots. Joint plots have one series per joint (colours J1-J6); the IMU plots have x, y, z.
type PlotKey = 'q' | 'dq' | 'load' | 'temp' | 'acc' | 'gyro';
interface PlotDef {
  key: PlotKey;
  title: string;
  unit: string;
  digits: number;
  minSpan: number; // smallest y range, so that noise does not fill the plot
  names: string[];
}
const PLOTS: PlotDef[] = [
  { key: 'q', title: 'Angle', unit: '°', digits: 1, minSpan: 2, names: JOINT_NAMES },
  { key: 'dq', title: 'Speed', unit: '°/s', digits: 1, minSpan: 2, names: JOINT_NAMES },
  { key: 'load', title: 'Load', unit: '%', digits: 1, minSpan: 2, names: JOINT_NAMES },
  { key: 'temp', title: 'Temperature', unit: '°C', digits: 0, minSpan: 2, names: JOINT_NAMES },
  { key: 'acc', title: 'IMU acceleration', unit: 'g', digits: 2, minSpan: 0.2, names: ['x', 'y', 'z'] },
  { key: 'gyro', title: 'IMU angular rate', unit: '°/s', digits: 1, minSpan: 2, names: ['x', 'y', 'z'] },
];

// --- Address history (per browser; it can be unavailable) ---------------------------------------

function loadHistory(): string[] {
  try {
    return JSON.parse(localStorage.getItem(HISTORY_KEY) || '[]');
  } catch {
    return [];
  }
}

function saveHistory(address: string) {
  const list = [address, ...loadHistory().filter((a) => a !== address)].slice(0, 8);
  try {
    localStorage.setItem(HISTORY_KEY, JSON.stringify(list));
  } catch {
    /* private window: no history */
  }
  renderHistory();
}

function renderHistory() {
  const dl = $('#addr-history');
  dl.replaceChildren(...loadHistory().map((a) => Object.assign(document.createElement('option'), { value: a })));
}

// --- Plots ---------------------------------------------------------------------------------------

const cssVar = (name: string) => getComputedStyle($('#ctl')).getPropertyValue(name).trim();

const GAP_S = 0.5; // a pause in the stream longer than this shows as a gap, not as a straight line

class History {
  t: number[] = [];
  cols: (number | null)[][];
  constructor(n: number) {
    this.cols = Array.from({ length: n }, () => []);
  }
  push(t: number, values: number[]) {
    const last = this.t.at(-1);
    if (last !== undefined && t - last > GAP_S) {
      this.t.push((last + t) / 2);
      this.cols.forEach((c) => c.push(null));
    }
    this.t.push(t);
    values.forEach((v, k) => this.cols[k].push(v));
    if (this.t.length > MAX_POINTS) {
      this.t.shift();
      this.cols.forEach((c) => c.shift());
    }
  }
  clear() {
    this.t = [];
    this.cols.forEach((c) => (c.length = 0));
  }
}

/** The decimals that a tick step needs: 5 → 0, 0.5 → 1, 0.25 → 2. */
function stepDecimals(step: number): number {
  let d = 0;
  while (d < 3 && Math.abs(step * 10 ** d - Math.round(step * 10 ** d)) > 1e-6) d++;
  return d;
}

function padRange(lo: number, hi: number, minSpan: number): [number, number] {
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [-minSpan / 2, minSpan / 2];
  const span = Math.max(hi - lo, minSpan);
  const mid = (hi + lo) / 2;
  return [mid - span * 0.55, mid + span * 0.55];
}

/** One small multiple: n series, a recessive grid, the time axis in seconds, a crosshair synced
 *  with the other plots. The legend above the plot shows the values at the crosshair (or the
 *  latest values) as text, so the colours never carry the numbers alone. */
class Plot {
  u: uPlot;
  private legend: HTMLElement;
  private values: HTMLElement[] = [];
  constructor(private def: PlotDef, private fig: HTMLElement, private hist: History) {
    this.legend = $('.p-legend', fig);
    this.legend.replaceChildren(
      ...def.names.map((name, k) => {
        const span = document.createElement('span');
        const key = document.createElement('i');
        key.style.background = `var(--series-${k + 1})`;
        const b = document.createElement('b');
        this.values.push(b);
        span.append(key, `${name} `, b);
        return span;
      }),
    );
    this.u = this.make();
  }
  private make(): uPlot {
    const area = $('.p-area', this.fig);
    const font = '11px system-ui, sans-serif';
    const axis = { stroke: cssVar('--axis'), grid: { stroke: cssVar('--grid'), width: 1 }, ticks: { show: false }, font };
    return new uPlot(
      {
        width: area.clientWidth || 300,
        height: area.clientHeight || 100,
        legend: { show: false },
        cursor: { y: false, sync: { key: 'ctl' }, points: { size: 6 } },
        scales: { x: { time: false }, y: { range: (_u, lo, hi) => padRange(lo, hi, this.def.minSpan) } },
        axes: [
          { ...axis, size: 18, gap: 2, values: (_u, v) => v.map((x) => `${Math.round(x)} s`) },
          {
            ...axis,
            size: 40,
            gap: 3,
            // As many decimals as the tick step needs (0.25 → 2), so that no two labels read the same.
            values: (_u, v, _ax, _space, incr) => v.map((x) => fmt(x, stepDecimals(incr))),
          },
        ],
        series: [
          {},
          ...this.def.names.map((name, k) => ({
            label: name,
            stroke: cssVar(`--series-${k + 1}`),
            width: 1.5,
            points: { show: false },
          })),
        ],
        hooks: { setCursor: [() => this.updateLegend()] },
      },
      [this.hist.t, ...this.hist.cols],
      area,
    );
  }
  rebuild() {
    this.u.destroy();
    this.u = this.make();
  }
  resize() {
    const area = this.u.root.parentElement as HTMLElement;
    if (area.clientWidth && area.clientHeight) this.u.setSize({ width: area.clientWidth, height: area.clientHeight });
  }
  render() {
    this.u.setData([this.hist.t, ...this.hist.cols]);
    this.updateLegend();
  }
  private updateLegend() {
    const idx = this.u.cursor.idx;
    const n = this.hist.t.length;
    const i = idx != null && idx >= 0 && idx < n ? idx : n - 1;
    this.values.forEach((b, k) => (b.textContent = i >= 0 ? fmt(this.hist.cols[k][i], this.def.digits) : '–'));
  }
}

// --- Page ----------------------------------------------------------------------------------------

export function start() {
  const link = new AtomLink();
  const root = $('#ctl');
  const hist: Record<PlotKey, History> = {
    q: new History(6),
    dq: new History(6),
    load: new History(6),
    temp: new History(6),
    acc: new History(3),
    gyro: new History(3),
  };
  let t0 = 0;
  let dirty = false;

  // 3D view: loaded after the page, because three.js and the meshes are large.
  let arm: ArmView | null = null;
  const armEl = $('#arm-view');
  import('./viewer3d').then(({ ArmView }) => {
    arm = new ArmView(armEl, armEl.dataset.urdf!);
    dirty = true;
  });
  $('#arm-reset').addEventListener('click', () => arm?.resetView());

  // Joint strips: name, angle, jog buttons, the fader (limits, measured position, goal), meta.
  const stripTpl = $<HTMLTemplateElement>('#strip');
  const strips = $('#strips');
  const goalInputs: HTMLInputElement[] = [];
  for (let j = 0; j < 6; j++) {
    const s = stripTpl.content.firstElementChild!.cloneNode(true) as HTMLElement;
    const name = JOINT_NAMES[j];
    s.dataset.joint = String(j);
    s.style.setProperty('--key', `var(--series-${j + 1})`);
    $('.j-name', s).textContent = name;
    $('.f-top', s).textContent = `+${P.LIMITS[j]}`;
    $('.f-bot', s).textContent = `−${P.LIMITS[j]}`;
    s.querySelectorAll<HTMLButtonElement>('.jog').forEach((b) =>
      b.setAttribute('aria-label', `Jog ${name} ${b.dataset.dir === '1' ? 'positive' : 'negative'} (hold)`),
    );
    const input = $<HTMLInputElement>('.goal-input', s);
    input.min = String(-P.LIMITS[j] + 1);
    input.max = String(P.LIMITS[j] - 1);
    input.value = '0';
    input.setAttribute('aria-label', `${name} goal angle`);
    input.setAttribute('orient', 'vertical'); // older Firefox
    goalInputs.push(input);
    strips.append(s);
  }
  const stripEls = [...strips.querySelectorAll<HTMLElement>('.strip')];
  const pct = (j: number, deg: number) => Math.max(0, Math.min(1, (deg + P.LIMITS[j]) / (2 * P.LIMITS[j]))) * 100;

  // Plots
  const plotTpl = $<HTMLTemplateElement>('#plot');
  const plotGrid = $('#plot-grid');
  const plots: Plot[] = [];
  for (const def of PLOTS) {
    const fig = plotTpl.content.firstElementChild!.cloneNode(true) as HTMLElement;
    $('.p-title', fig).textContent = `${def.title} (${def.unit})`;
    plotGrid.append(fig);
  }
  requestAnimationFrame(() => {
    plotGrid.querySelectorAll<HTMLElement>('.plot').forEach((fig, k) => plots.push(new Plot(PLOTS[k], fig, hist[PLOTS[k].key])));
  });
  new ResizeObserver(() => plots.forEach((p) => p.resize())).observe(plotGrid);

  // Repaint the plots when the theme changes (the colours are CSS variables).
  new MutationObserver(() => {
    plots.forEach((p) => p.rebuild());
    dirty = true;
  }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

  // --- Connection ---
  const form = $<HTMLFormElement>('#connect-form');
  const addr = $<HTMLInputElement>('#addr');
  const connectBtn = $<HTMLButtonElement>('#connect');
  renderHistory();
  const params = new URLSearchParams(location.search);
  addr.value = params.get('atom') || loadHistory()[0] || '';

  form.addEventListener('submit', (e) => {
    e.preventDefault();
    if (link.status !== 'disconnected') return link.disconnect();
    const a = addr.value.trim();
    if (!a) return addr.focus();
    if (location.protocol === 'https:' && !P.isLocalAddress(a)) {
      hint(`This page uses HTTPS. The browser can reach a robot only at a private IP address (for example 192.168.1.107) or a .local name.`, 'warning');
    } else hint('');
    saveHistory(a);
    for (const h of Object.values(hist)) h.clear();
    t0 = 0;
    link.connect(a);
  });

  function hint(text: string, kind: 'warning' | 'info' = 'info') {
    const el = $('#hint');
    el.textContent = text;
    el.dataset.kind = kind;
    el.hidden = !text;
  }

  link.on('status', ({ status, detail }) => {
    const pill = $('#link-status');
    pill.dataset.status = status;
    pill.querySelector('.label')!.textContent =
      status === 'connected' ? 'Connected' : status === 'connecting' ? 'Connecting…' : 'Disconnected';
    pill.title = status === 'connected' ? `Connected to ${link.address}` : '';
    connectBtn.textContent = status === 'disconnected' ? 'Connect' : 'Disconnect';
    addr.disabled = status !== 'disconnected';
    if (status === 'connecting' && detail) {
      const https = location.protocol === 'https:';
      hint(
        `${detail}. Check that the robot is on, on the same network, and runs the controller firmware 4.2 or later.` +
          (https ? ' If the browser asks for access to devices on the local network, allow it.' : ''),
        'warning',
      );
    } else if (status === 'connected') hint('');
    if (status === 'connected' && !$('#target').dataset.target) setTarget('real');
    if (status !== 'connected') {
      setTarget('none');
      $('#fw').textContent = '';
      $('#robot-state').textContent = '';
    }
    updateControls();
  });

  link.on('pong', (p) => {
    $('#fw').textContent = `Firmware ${p.version}`;
  });

  // The simulated ATOM's status log starts with "atom-sim". Until the first line arrives (1 s), the
  // page assumes the real robot: the safe side.
  link.on('log', (line) => {
    $('#log').textContent = line;
    $('#log').title = line;
    setTarget(line.startsWith('atom-sim') ? 'sim' : 'real');
  });

  function setTarget(t: 'real' | 'sim' | 'none') {
    const el = $('#target');
    el.dataset.target = t === 'none' ? '' : t;
    el.textContent = t === 'real' ? '⚠ Real robot — it moves' : t === 'sim' ? 'Simulator — nothing moves' : '';
    root.dataset.target = el.dataset.target;
  }

  link.on('ack', (a) => {
    if (a.status !== 0) toast(`${P.CODE_NAMES[a.code] ?? a.code}: ${P.ackText(a.code, a.status)}`, 'warning');
    updateControls();
  });

  link.on('done', (d) => {
    const r = P.DONE_RESULTS[d.result] ?? `result ${d.result}`;
    toast(`Move ended: ${r}`, d.result === 0 ? 'info' : 'warning');
  });

  link.on('stream', (s: Stream) => {
    // The time comes from the ATOM. If it goes back (the ATOM restarted), start the plots again.
    if (!t0 || s.tMs < t0 + (hist.q.t.at(-1) ?? 0) * 1000) {
      for (const h of Object.values(hist)) h.clear();
      t0 = s.tMs;
    }
    const t = (s.tMs - t0) / 1000;
    hist.q.push(t, s.q);
    hist.dq.push(t, s.dq);
    hist.load.push(t, s.load);
    hist.temp.push(t, s.temp);
    hist.acc.push(t, s.acc);
    hist.gyro.push(t, s.gyro);
    dirty = true;
  });

  // --- Rendering at the display rate ---
  function render() {
    requestAnimationFrame(render);
    if (!dirty) return;
    dirty = false;
    plots.forEach((p) => p.render());
    const s = link.last;
    if (!s) return;
    $('#robot-state').textContent = `Robot: ${P.STATES[s.state] ?? s.state}`;
    $('#robot-state').dataset.state = String(s.state);
    stripEls.forEach((row, j) => {
      $('.j-angle', row).textContent = `${fmt(s.q[j])}°`;
      $('.j-temp', row).textContent = `${s.temp[j]} °C`;
      $('.j-volt', row).textContent = `${fmt(s.volt[j])} V`;
      // The track runs from −limit (bottom) to +limit (top). The fill goes from zero to the angle.
      const y = 100 - pct(j, s.q[j]);
      $('.f-pos', row).style.top = `${y}%`;
      const fill = $('.f-fill', row);
      fill.style.top = `${Math.min(y, 50)}%`;
      fill.style.height = `${Math.abs(y - 50)}%`;
      row.classList.toggle('servo-fault', s.status[j] !== 0);
    });
    arm?.setPose(s.q);
    updateGoals();
    updateControls();
  }
  requestAnimationFrame(render);

  // --- Controls ---
  const controlBtn = $<HTMLButtonElement>('#control-btn');
  const takeoverBtn = $<HTMLButtonElement>('#takeover-btn');
  const speed = $<HTMLInputElement>('#jog-speed');

  function updateControls() {
    const connected = link.status === 'connected';
    const s = link.last;
    const other = s?.control === 2;
    const busy = s ? [P.STATE_PLAYING, P.STATE_MOVING].includes(s.state) : false;
    const mine = connected && link.inControl;
    root.dataset.control = mine ? 'mine' : other ? 'other' : 'none';
    $('#control-text').textContent = !connected
      ? 'Not connected'
      : mine
        ? 'You have control'
        : other
          ? 'Another client has control: you can watch'
          : 'Nobody has control';
    controlBtn.textContent = mine ? 'Release control' : 'Take control';
    controlBtn.disabled = !connected || (other && !mine);
    takeoverBtn.hidden = !(connected && other && !mine);
    $<HTMLButtonElement>('#stop-btn').disabled = !connected;
    $<HTMLButtonElement>('#hold-btn').disabled = !connected;
    root.querySelectorAll<HTMLButtonElement>('.needs-control').forEach((b) => (b.disabled = !mine || busy));
    goalInputs.forEach((i) => (i.disabled = !mine));
  }

  controlBtn.addEventListener('click', () => link.requestControl(link.inControl ? 0 : 1));
  takeoverBtn.addEventListener('click', () => link.requestControl(2));
  $('#stop-btn').addEventListener('click', () => link.stopRobot());
  $('#hold-btn').addEventListener('click', () => link.holdPose());
  $('#zero-btn').addEventListener('click', () => link.moveTo([0, 0, 0, 0, 0, 0]));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && link.status === 'connected') link.stopRobot();
  });

  // Jog: press and hold. The page sends JOG every 50 ms; the ATOM stops 200 ms after the last one.
  speed.addEventListener('input', () => ($('#jog-speed-value').textContent = `${speed.value} °/s`));
  strips.querySelectorAll<HTMLButtonElement>('.jog').forEach((b) => {
    const j = Number((b.closest('.strip') as HTMLElement).dataset.joint);
    const dir = Number(b.dataset.dir);
    const end = () => link.setJog(j, 0);
    b.addEventListener('pointerdown', (e) => {
      if (b.disabled) return;
      b.setPointerCapture(e.pointerId);
      link.setJog(j, dir * Number(speed.value));
    });
    b.addEventListener('pointerup', end);
    b.addEventListener('pointercancel', end);
    b.addEventListener('lostpointercapture', end);
    b.addEventListener('contextmenu', (e) => e.preventDefault());
  });
  window.addEventListener('blur', () => link.stopJog());
  document.addEventListener('visibilitychange', () => document.hidden && link.stopJog());

  // Goal pose: the faders, then MOVE_TO. The goal shows as a see-through arm when it differs from
  // the measured pose, and its value turns blue.
  function updateGoals() {
    const s = link.last;
    const goal = goalInputs.map((i) => Number(i.value));
    let differs = false;
    stripEls.forEach((row, j) => {
      const out = $('output', row);
      out.textContent = `goal ${goal[j]}°`;
      const same = !s || Math.abs(goal[j] - s.q[j]) <= 1;
      out.toggleAttribute('data-same', same);
      differs ||= !same;
    });
    arm?.setGoal(s && differs ? goal : null);
  }
  goalInputs.forEach((i) => i.addEventListener('input', updateGoals));
  updateGoals();
  const setGoals = (q: number[]) => {
    goalInputs.forEach((i, j) => (i.value = String(Math.round(q[j]))));
    updateGoals();
  };
  $('#goal-current').addEventListener('click', () => link.last && setGoals(link.last.q));
  $('#goal-move').addEventListener('click', () => link.moveTo(goalInputs.map((i) => Number(i.value))));

  // Toasts for refused commands and finished moves.
  function toast(text: string, kind: 'info' | 'warning') {
    const el = document.createElement('div');
    el.className = 'toast';
    el.dataset.kind = kind;
    el.setAttribute('role', 'status');
    el.textContent = text;
    $('#toasts').append(el);
    setTimeout(() => el.remove(), 4000);
  }

  updateControls();
  if (params.get('atom')) form.requestSubmit();
}
