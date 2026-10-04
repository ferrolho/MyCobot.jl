// The Control page: connection, joint state with small plots, IMU, and the controls.
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { AtomLink } from './connection';
import * as P from './protocol';
import type { ArmView } from './viewer3d';

const $ = <T extends HTMLElement>(sel: string, root: ParentNode = document) => root.querySelector(sel) as T;
const fmt = (x: number, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '–');

const HISTORY_KEY = 'mycobot-control.addresses';
const WINDOW_S = 20; // plot window
const MAX_POINTS = WINDOW_S * 50 + 10;
const SPARK_H = 56; // px

type Quantity = 'q' | 'dq' | 'load' | 'temp';
const QUANTITIES: Record<Quantity, { label: string; unit: string; digits: number }> = {
  q: { label: 'Angle', unit: '°', digits: 1 },
  dq: { label: 'Speed', unit: '°/s', digits: 1 },
  load: { label: 'Load', unit: '%', digits: 1 },
  temp: { label: 'Temperature', unit: '°C', digits: 0 },
};

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

class History {
  t: number[] = [];
  cols: number[][];
  constructor(n: number) {
    this.cols = Array.from({ length: n }, () => []);
  }
  push(t: number, values: number[]) {
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

function sparkline(el: HTMLElement, unit: string): uPlot {
  const opts: uPlot.Options = {
    width: el.clientWidth || 200,
    height: SPARK_H,
    legend: { show: false },
    cursor: { y: false, points: { size: 8 } },
    scales: { x: { time: false }, y: { range: (_u, lo, hi) => padRange(lo, hi) } },
    axes: [
      { show: false },
      {
        side: 1,
        size: 44,
        gap: 2,
        ticks: { show: false },
        grid: { stroke: cssVar('--grid'), width: 1 },
        stroke: cssVar('--muted'),
        font: '11px system-ui, sans-serif',
        splits: (_u, _ax, lo, hi) => [lo, hi],
        values: (_u, v) => v.map((x) => fmt(x, Math.abs(x) < 10 ? 1 : 0)),
      },
    ],
    series: [{}, { stroke: cssVar('--series-1'), width: 2, points: { show: false }, label: unit }],
  };
  return new uPlot(opts, [[], []], el);
}

function padRange(lo: number, hi: number): [number, number] {
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [-1, 1];
  const span = Math.max(hi - lo, 2);
  const mid = (hi + lo) / 2;
  return [mid - span * 0.6, mid + span * 0.6];
}

// --- Page ----------------------------------------------------------------------------------------

export function start() {
  const link = new AtomLink();
  const root = $('#ctl');
  let quantity: Quantity = 'q';
  const hist: Record<Quantity, History> = { q: new History(6), dq: new History(6), load: new History(6), temp: new History(6) };
  const acc = new History(3);
  let t0 = 0;
  let dirty = false;

  // 3D view: loaded after the page, because three.js and the meshes are large.
  let arm: ArmView | null = null;
  const armEl = $('#arm-view');
  import('./viewer3d').then(({ ArmView }) => {
    arm = new ArmView(armEl, armEl.dataset.urdf!);
    dirty = true;
  });

  // Joint rows
  const rowTpl = $<HTMLTemplateElement>('#joint-row');
  const rows = $('#joint-rows');
  const plots: uPlot[] = [];
  for (let j = 0; j < 6; j++) {
    const row = rowTpl.content.firstElementChild!.cloneNode(true) as HTMLElement;
    row.dataset.joint = String(j);
    $('.j-name', row).textContent = `J${j + 1}`;
    $('.j-limits', row).textContent = `±${P.LIMITS[j]}°`;
    rows.append(row);
  }
  requestAnimationFrame(() => {
    rows.querySelectorAll<HTMLElement>('.j-plot').forEach((el) => plots.push(sparkline(el, QUANTITIES[quantity].unit)));
  });

  // IMU plot: three series, legend and direct values beside it.
  const imuEl = $('#imu-plot');
  let imuPlot: uPlot | null = null;
  const makeImuPlot = () => {
    imuPlot?.destroy();
    imuPlot = new uPlot(
      {
        width: imuEl.clientWidth || 300,
        height: 120,
        legend: { show: false },
        cursor: { y: false },
        scales: { x: { time: false }, y: { range: (_u, lo, hi) => padRange(lo, hi) } },
        axes: [
          { show: false },
          { side: 1, size: 48, stroke: cssVar('--muted'), grid: { stroke: cssVar('--grid'), width: 1 }, ticks: { show: false }, font: '11px system-ui, sans-serif' },
        ],
        series: [{}, ...['x', 'y', 'z'].map((a, k) => ({ label: a, stroke: cssVar(`--series-${k + 1}`), width: 2, points: { show: false } }))],
      },
      [[], [], [], []],
      imuEl,
    );
  };
  requestAnimationFrame(makeImuPlot);

  const resize = () => {
    plots.forEach((p) => p.setSize({ width: (p.root.parentElement as HTMLElement).clientWidth, height: SPARK_H }));
    imuPlot?.setSize({ width: imuEl.clientWidth, height: 120 });
  };
  new ResizeObserver(resize).observe(rows);

  // Repaint the plots when the theme changes (the colours are CSS variables).
  new MutationObserver(() => {
    plots.forEach((p, j) => {
      const el = p.root.parentElement as HTMLElement;
      p.destroy();
      plots[j] = sparkline(el, QUANTITIES[quantity].unit);
    });
    makeImuPlot();
    dirty = true;
  }).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

  // Quantity selector
  root.querySelectorAll<HTMLButtonElement>('[data-quantity]').forEach((b) =>
    b.addEventListener('click', () => {
      quantity = b.dataset.quantity as Quantity;
      root.querySelectorAll('[data-quantity]').forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
      $('#plot-title').textContent = `${QUANTITIES[quantity].label} (${QUANTITIES[quantity].unit}), last ${WINDOW_S} s`;
      dirty = true;
    }),
  );

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
    acc.clear();
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
      status === 'connected' ? `Connected to ${link.address}` : status === 'connecting' ? 'Connecting…' : 'Disconnected';
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

  link.on('stream', (s) => {
    if (!t0) t0 = s.tMs;
    const t = (s.tMs - t0) / 1000;
    hist.q.push(t, s.q);
    hist.dq.push(t, s.dq);
    hist.load.push(t, s.load);
    hist.temp.push(t, s.temp);
    acc.push(t, s.acc);
    dirty = true;
  });

  // --- Rendering at the display rate ---
  function render() {
    requestAnimationFrame(render);
    if (!dirty) return;
    dirty = false;
    const s = link.last;
    const h = hist[quantity];
    const q = QUANTITIES[quantity];
    plots.forEach((p, j) => p.setData([h.t, h.cols[j]]));
    imuPlot?.setData([acc.t, ...acc.cols]);
    if (!s) return;
    $('#robot-state').textContent = `Robot: ${P.STATES[s.state] ?? s.state}`;
    $('#robot-state').dataset.state = String(s.state);
    rows.querySelectorAll<HTMLElement>('.joint').forEach((row, j) => {
      $('.j-angle', row).textContent = `${fmt(s.q[j])}°`;
      $('.j-value', row).textContent = quantity === 'q' ? '' : `${fmt(s[quantity][j], q.digits)} ${q.unit}`;
      $('.j-temp', row).textContent = `${s.temp[j]} °C`;
      $('.j-volt', row).textContent = `${fmt(s.volt[j])} V`;
      const x = (s.q[j] + P.LIMITS[j]) / (2 * P.LIMITS[j]);
      $('.j-marker', row).style.left = `${Math.max(0, Math.min(1, x)) * 100}%`;
      row.classList.toggle('servo-fault', s.status[j] !== 0);
    });
    arm?.setPose(s.q);
    updateGhost();
    $('#acc-values').textContent = s.acc.map((a, k) => `${'xyz'[k]} ${fmt(a, 3)}`).join('  ');
    $('#gyro-values').textContent = s.gyro.map((g, k) => `${'xyz'[k]} ${fmt(g, 1)}`).join('  ');
    updateControls();
  }
  requestAnimationFrame(render);

  // --- Controls ---
  const controlBtn = $<HTMLButtonElement>('#control-btn');
  const takeoverBtn = $<HTMLButtonElement>('#takeover-btn');
  const speed = $<HTMLInputElement>('#jog-speed');
  const goalInputs: HTMLInputElement[] = [];

  function updateControls() {
    const connected = link.status === 'connected';
    const s = link.last;
    const other = s?.control === 2;
    const busy = s ? [P.STATE_PLAYING, P.STATE_MOVING].includes(s.state) : false;
    const mine = connected && link.inControl;
    root.dataset.control = mine ? 'mine' : other ? 'other' : 'none';
    $('#control-text').textContent = !connected
      ? 'Not connected.'
      : mine
        ? 'You have control. Press Esc to stop the robot.'
        : other
          ? 'Another client has control. You can watch.'
          : 'Nobody has control. Take control to move the robot.';
    controlBtn.textContent = mine ? 'Release control' : 'Take control';
    controlBtn.disabled = !connected || (other && !mine);
    takeoverBtn.hidden = !(connected && other && !mine);
    $<HTMLButtonElement>('#stop-btn').disabled = !connected;
    $<HTMLButtonElement>('#hold-btn').disabled = !connected;
    root.querySelectorAll<HTMLButtonElement>('.needs-control').forEach((b) => (b.disabled = !mine || busy));
    root.querySelectorAll<HTMLInputElement>('.goal input').forEach((i) => (i.disabled = !mine));
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
  rows.querySelectorAll<HTMLButtonElement>('.jog').forEach((b) => {
    const j = Number((b.closest('.joint') as HTMLElement).dataset.joint);
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

  // Goal pose: one slider per joint, then MOVE_TO.
  const goalTpl = $<HTMLTemplateElement>('#goal-row');
  const goals = $('#goals');
  for (let j = 0; j < 6; j++) {
    const row = goalTpl.content.firstElementChild!.cloneNode(true) as HTMLElement;
    const input = $<HTMLInputElement>('input', row);
    const out = $('output', row);
    $('.g-name', row).textContent = `J${j + 1}`;
    input.min = String(-P.LIMITS[j] + 1);
    input.max = String(P.LIMITS[j] - 1);
    input.value = '0';
    input.setAttribute('aria-label', `J${j + 1} goal angle`);
    input.addEventListener('input', () => (out.textContent = `${input.value}°`));
    goalInputs.push(input);
    goals.append(row);
  }
  // The goal is shown as a see-through arm when it differs from the measured pose.
  function updateGhost() {
    const s = link.last;
    const goal = goalInputs.map((i) => Number(i.value));
    const differs = !!s && goal.some((g, j) => Math.abs(g - s.q[j]) > 1);
    arm?.setGoal(differs ? goal : null);
  }
  goalInputs.forEach((i) => i.addEventListener('input', updateGhost));
  const setGoals = (q: number[]) =>
    goalInputs.forEach((i, j) => {
      i.value = String(Math.round(q[j]));
      i.dispatchEvent(new Event('input'));
    });
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
