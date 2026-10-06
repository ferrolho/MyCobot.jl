// The Control page: connection, joint strips (state, jog, goal fader), six plots, and the controls.
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { AtomLink } from './connection';
import * as P from './protocol';
import type { Stream } from './protocol';
import type { ArmView } from './viewer3d';
import { startSessionLog } from './sessionlog';

const $ = <T extends HTMLElement>(sel: string, root: ParentNode = document) => root.querySelector(sel) as T;
const fmt = (x: number | null, d = 1) => (x !== null && Number.isFinite(x) ? x.toFixed(d) : '–');

const HISTORY_KEY = 'mycobot-control.addresses';
let relay = ''; // the lab service's relay to the ATOM ("raspberrypi5:8280/atom/ws"), when the Pi serves the page
let direct = ''; // the ATOM's own address, when the Pi serves the page and this browser reaches the ATOM
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
  const list = [...new Set([direct, relay, ...loadHistory()].filter(Boolean))];
  dl.replaceChildren(...list.map((a) => Object.assign(document.createElement('option'), { value: a })));
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

// "4.4.0" ≥ 4.4? Live mode needs controller firmware 4.4 (TRACK).
const versionAtLeast = (v: string, major: number, minor: number) => {
  const [a, b] = v.split('.').map(Number);
  return a > major || (a === major && b >= minor);
};

export function start() {
  const link = new AtomLink();
  const root = $('#ctl');
  startSessionLog(link); // only when the lab service on the Pi serves the page
  const hist: Record<PlotKey, History> = {
    q: new History(6),
    dq: new History(6),
    load: new History(6),
    temp: new History(6),
    acc: new History(3),
    gyro: new History(3),
  };
  let t0 = 0; // the ATOM's t_ms at the first sample: plot time 0
  let dirty = false;

  // 3D view: loaded after the page, because three.js and the meshes are large.
  let arm: ArmView | null = null;
  const armEl = $('#arm-view');
  import('./viewer3d').then(({ ArmView }) => {
    arm = new ArmView(armEl, armEl.dataset.urdf!);
    if (gripperShown) arm.setGripper(true); // found before the 3D view was ready
    dirty = true;
    // Lab scene (objects near the robot): only the lab service on the Pi has it. Poll while it answers.
    const loadScene = () =>
      fetch('/lab/scene.json', { cache: 'no-store' })
        .then((r) => (r.ok ? r.json() : Promise.reject(new Error(String(r.status)))))
        .then((scene) => {
          arm?.setScene(scene);
          setTimeout(loadScene, 5000);
        })
        .catch(() => {});
    loadScene();
  });
  $('#arm-reset').addEventListener('click', () => arm?.resetView());

  // Joint strips: name, angle, the fader (limits, measured position, goal), typed goal, jog, meta.
  const stripTpl = $<HTMLTemplateElement>('#strip');
  const strips = $('#strips');
  const faders: HTMLInputElement[] = [];
  const goalNums: HTMLInputElement[] = [];
  const goal = [0, 0, 0, 0, 0, 0];
  // MOVE_TO and JOG both stay inside these. The limits need not be symmetric (J6: −225° to +135°).
  const goalMin = (j: number) => P.LIMIT_MIN[j] + P.JOG_MARGIN;
  const goalMax = (j: number) => P.LIMIT_MAX[j] - P.JOG_MARGIN;
  const clampGoal = (j: number, deg: number) => Math.round(Math.max(goalMin(j), Math.min(goalMax(j), deg)) * 10) / 10;
  // Fader track: LIMIT_MIN (bottom) to LIMIT_MAX (top), in % from the bottom.
  const pct = (j: number, deg: number) => Math.max(0, Math.min(1, (deg - P.LIMIT_MIN[j]) / (P.LIMIT_MAX[j] - P.LIMIT_MIN[j]))) * 100;
  for (let j = 0; j < 6; j++) {
    const s = stripTpl.content.firstElementChild!.cloneNode(true) as HTMLElement;
    const name = JOINT_NAMES[j];
    s.dataset.joint = String(j);
    s.style.setProperty('--key', `var(--series-${j + 1})`);
    $('.j-name', s).textContent = name;
    const signed = (x: number) => (x > 0 ? `+${x}` : `−${-x}`);
    $('.f-top', s).textContent = signed(P.LIMIT_MAX[j]);
    $('.f-bot', s).textContent = signed(P.LIMIT_MIN[j]);
    $('.f-zero', s).style.top = `${100 - pct(j, 0)}%`;
    s.querySelectorAll<HTMLButtonElement>('.jog').forEach((b) =>
      b.setAttribute('aria-label', `Jog ${name} ${b.dataset.dir === '1' ? 'positive' : 'negative'} (hold)`),
    );
    const fader = $<HTMLInputElement>('.goal-input', s);
    fader.min = String(goalMin(j));
    fader.max = String(goalMax(j));
    fader.value = '0';
    fader.setAttribute('aria-label', `${name} goal angle`);
    fader.setAttribute('orient', 'vertical'); // older Firefox
    faders.push(fader);
    const num = $<HTMLInputElement>('.goal-num', s);
    num.min = fader.min;
    num.max = fader.max;
    num.value = '0';
    num.setAttribute('aria-label', `${name} goal angle, degrees`);
    goalNums.push(num);
    strips.append(s);
  }
  const stripEls = [...strips.querySelectorAll<HTMLElement>('.strip')];

  // J7, the gripper (firmware 4.6+): shown while the ATOM finds it (servo ID 7). It works as the other
  // joints: the fader and the typed value set the goal (Move sends it; in Live mode it goes at once), and
  // ▼/▲ move it while held (not in Live mode). 0 % closed, 100 % open.
  const gStrip = stripTpl.content.firstElementChild!.cloneNode(true) as HTMLElement;
  gStrip.classList.add('gripper-strip');
  gStrip.hidden = true;
  gStrip.style.setProperty('--key', 'var(--sl-color-gray-3)');
  $('.j-name', gStrip).textContent = 'J7';
  gStrip.title = 'J7: the gripper';
  $('.f-top', gStrip).textContent = '100'; // % open; words do not fit next to the fader
  $('.f-bot', gStrip).textContent = '0';
  $('.f-zero', gStrip).hidden = true;
  $('.goal-field .sr-only', gStrip).textContent = 'Goal (%)';
  const gFader = $<HTMLInputElement>('.goal-input', gStrip);
  const gNum = $<HTMLInputElement>('.goal-num', gStrip);
  for (const i of [gFader, gNum]) Object.assign(i, { min: '0', max: '100', step: '1', value: '0' });
  gFader.setAttribute('aria-label', 'J7 (gripper) goal opening');
  gFader.setAttribute('orient', 'vertical');
  gNum.setAttribute('aria-label', 'J7 (gripper) goal opening, percent');
  // ▼ closes and ▲ opens while held, as the jog buttons of the other joints: on release the goal
  // becomes the measured opening, so the gripper stops where it is.
  const [gClose, gOpen] = [...gStrip.querySelectorAll<HTMLButtonElement>('.jog')];
  for (const [b, label] of [[gClose, 'Close J7 (hold)'], [gOpen, 'Open J7 (hold)']] as const) {
    b.className = 'btn jog grip-btn needs-control not-live'; // .jog styling; the joint jog handlers skip .grip-btn
    b.setAttribute('aria-label', label);
  }
  $('.j-meta', gStrip).replaceChildren(Object.assign(document.createElement('span'), { className: 'g-load', title: 'Servo load' }));
  strips.append(gStrip);
  let gripperShown = false;
  let gGoal = 0; // %

  // At most 20 GRIPPER messages a second; the last value is always sent.
  let gTimer: number | undefined;
  let gPending: { v: number | null } | null = null;
  const sendGripper = (opening: number | null) => {
    if (gTimer !== undefined) {
      gPending = { v: opening };
      return;
    }
    link.send(P.gripper(opening));
    gTimer = window.setTimeout(() => {
      gTimer = undefined;
      const p = gPending;
      gPending = null;
      if (p) sendGripper(p.v);
    }, 50);
  };
  const gMeasured = () => (link.last?.gripper ? Math.max(0, Math.min(1, link.last.gripper.opening)) * 100 : null);
  const setGripperGoal = (pct: number, send: boolean) => {
    gGoal = Math.round(Math.max(0, Math.min(100, pct)));
    gFader.value = String(gGoal);
    if (document.activeElement !== gNum) gNum.value = String(gGoal);
    if (send && link.inControl) sendGripper(gGoal / 100);
    updateGoals();
  };
  gFader.addEventListener('input', () => setGripperGoal(Number(gFader.value), live));
  gNum.addEventListener('change', () => {
    const v = Number(gNum.value.replace(',', '.'));
    if (Number.isFinite(v) && gNum.value.trim() !== '') setGripperGoal(v, live);
    gNum.value = String(gGoal);
  });
  for (const [b, end] of [[gClose, 0], [gOpen, 100]] as const) {
    let held = false;
    b.addEventListener('pointerdown', (e) => {
      if (b.disabled) return;
      b.setPointerCapture(e.pointerId);
      held = true;
      setGripperGoal(end, true);
    });
    const stop = () => {
      if (!held) return;
      held = false;
      const m = gMeasured();
      if (m !== null) setGripperGoal(m, true);
    };
    b.addEventListener('pointerup', stop);
    b.addEventListener('pointercancel', stop);
    b.addEventListener('lostpointercapture', stop);
    b.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  function renderGripper(g: P.Stream['gripper']) {
    if (!!g !== gripperShown) {
      gripperShown = !!g;
      gStrip.hidden = !g;
      arm?.setGripper(!!g);
      if (g) setGripperGoal(g.opening * 100, false); // start from where it is: no jump to an old goal
      updateControls();
    }
    if (!g) return;
    const pctOpen = Math.max(0, Math.min(1, g.opening)) * 100;
    $('.j-angle', gStrip).textContent = `${Math.round(pctOpen)}\u00a0%`;
    $('.f-pos', gStrip).style.top = `${100 - pctOpen}%`;
    const fill = $('.f-fill', gStrip);
    fill.style.top = `${100 - pctOpen}%`;
    fill.style.height = `${pctOpen}%`;
    $('.g-load', gStrip).textContent = `${fmt(g.load)}\u00a0%`;
    arm?.setGripperOpening(g.opening);
    arm?.setGoalGripperOpening(Math.abs(gGoal - pctOpen) > 3 ? gGoal / 100 : null);
  }

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

  function clearHistory() {
    for (const h of Object.values(hist)) h.clear();
    t0 = 0;
  }

  // --- Connection ---
  const form = $<HTMLFormElement>('#connect-form');
  const addr = $<HTMLInputElement>('#addr');
  const connectBtn = $<HTMLButtonElement>('#connect');
  renderHistory();
  const params = new URLSearchParams(location.search);
  addr.value = params.get('atom') || loadHistory()[0] || '';
  // Served by the lab service on the Pi: on the home network, connect straight to the ATOM; away from
  // home, through the Pi's relay. The relay shares the Pi's link with the camera stream, which delayed
  // the robot's messages by up to 2 s (2026-10-06).
  fetch('/atom.json', { cache: 'no-store' })
    .then((r) => (r.ok ? r.json() : null))
    .then(async (j: { atom?: string; ws?: string } | null) => {
      if (!j?.ws) return;
      relay = location.host + j.ws;
      direct = j.atom && (await reachable(j.atom)) ? j.atom : '';
      renderHistory();
      if (!params.get('atom') && link.status === 'disconnected') addr.value = direct || relay;
    })
    .catch(() => {});

  // Does a WebSocket to this ATOM address open within 1.5 s? (It closes again at once.)
  function reachable(address: string): Promise<boolean> {
    return new Promise((resolve) => {
      let ws: WebSocket;
      try {
        ws = new WebSocket(P.wsUrl(address));
      } catch {
        return resolve(false);
      }
      const done = (ok: boolean) => {
        clearTimeout(timer);
        ws.onopen = ws.onerror = null;
        ws.close();
        resolve(ok);
      };
      const timer = setTimeout(() => done(false), 1500);
      ws.onopen = () => done(true);
      ws.onerror = () => done(false);
    });
  }

  form.addEventListener('submit', (e) => {
    e.preventDefault();
    if (link.status !== 'disconnected') return link.disconnect();
    const a = addr.value.trim();
    if (!a) return addr.focus();
    if (location.protocol === 'https:' && !P.isLocalAddress(a)) {
      hint(`This page uses HTTPS. The browser can reach a robot only at a private IP address (for example 192.168.1.107) or a .local name.`, 'warning');
    } else hint('');
    saveHistory(a);
    clearHistory();
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
      const where = link.address === relay ? 'and that the Pi reaches it on the home network' : 'on the same network';
      hint(
        `${detail}. Check that the robot is on, ${where}, and runs the controller firmware 4.2 or later.` +
          (https ? ' If the browser asks for access to devices on the local network, allow it.' : ''),
        'warning',
      );
    } else if (status === 'connected') hint('');
    if (status === 'connected' && !$('#target').dataset.target) setTarget('real');
    if (status !== 'connected') {
      setLive(false, 'the connection closed');
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
    $('#log').textContent = statusSummary(line);
    $('#log').title = line;
    setTarget(line.startsWith('atom-sim') ? 'sim' : 'real');
  });

  // The status line is long (firmware, git, IP, RSSI, state, plan, IMU, retries, heap, uptime). The
  // footer shows the useful part; the tooltip and the session log keep the whole line.
  function statusSummary(line: string): string {
    const v = (key: string) => line.match(new RegExp(`\\b${key}=(-?\\d+)`))?.[1];
    const variant = line.match(/, (lab|public)\)/)?.[1];
    const up = v('up');
    const parts = [
      line.startsWith('atom-sim') ? 'simulator' : variant,
      v('rssi') && `RSSI ${v('rssi')} dBm`,
      up && `up ${Number(up) < 120 ? `${up} s` : `${Math.round(Number(up) / 60)} min`}`,
      v('write_retries') && `write retries ${v('write_retries')}`,
    ].filter(Boolean);
    return parts.length ? parts.join(' · ') : line;
  }

  function setTarget(t: 'real' | 'sim' | 'none') {
    const el = $('#target');
    el.dataset.target = t === 'none' ? '' : t;
    el.textContent = t === 'real' ? '⚠ Real robot — it moves' : t === 'sim' ? 'Simulator — nothing moves' : '';
    root.dataset.target = el.dataset.target;
  }

  link.on('ack', (a) => {
    if (a.status !== 0 && a.code === P.Code.TRACK) setLive(false, `the robot refused it (${P.ackText(a.code, a.status)})`);
    else if (a.status !== 0) toast(`${P.CODE_NAMES[a.code] ?? a.code}: ${P.ackText(a.code, a.status)}`, 'warning');
    updateControls();
  });

  link.on('done', (d) => {
    const r = P.DONE_RESULTS[d.result] ?? `result ${d.result}`;
    const detail = d.result === 1 && d.joint ? ` (J${d.joint} was ${d.errorDeg.toFixed(0)}° from its goal)` : '';
    if (live) setLive(false, `the robot stopped: ${r}${detail}`);
    else toast(`Move ended: ${r}${detail}`, d.result === 0 ? 'info' : 'warning');
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
    if (!link.inControl && live) setLive(false, 'control was released');
    if (live && s.state === P.STATE_ERROR) setLive(false, 'the robot reported an error');
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
      $('.j-temp', row).textContent = `${s.temp[j]}\u00a0°C`; // no-break space: the unit stays with its number
      $('.j-volt', row).textContent = `${fmt(s.volt[j])}\u00a0V`;
      // The track runs from the min limit (bottom) to the max limit (top). The fill goes from zero to the angle.
      const y = 100 - pct(j, s.q[j]), y0 = 100 - pct(j, 0);
      $('.f-pos', row).style.top = `${y}%`;
      const fill = $('.f-fill', row);
      fill.style.top = `${Math.min(y, y0)}%`;
      fill.style.height = `${Math.abs(y - y0)}%`;
      row.classList.toggle('servo-fault', s.status[j] !== 0);
    });
    arm?.setPose(s.q);
    renderGripper(s.gripper);
    updateGoals();
    updateControls();
  }
  requestAnimationFrame(render);

  // --- Controls ---
  const controlBtn = $<HTMLButtonElement>('#control-btn');
  const takeoverBtn = $<HTMLButtonElement>('#takeover-btn');
  const watchBtn = $<HTMLButtonElement>('#watch-btn');
  const speed = $<HTMLInputElement>('#jog-speed');
  const liveBox = $<HTMLInputElement>('#live');

  function updateControls() {
    const connected = link.status === 'connected';
    const s = link.last;
    const other = s?.control === 2;
    const busy = s ? [P.STATE_PLAYING, P.STATE_MOVING].includes(s.state) : false;
    const mine = connected && link.inControl;
    root.dataset.control = mine ? 'mine' : other ? 'other' : 'none';
    root.dataset.live = live ? 'on' : '';
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

    // The banner over the joint controls when this page cannot move the robot.
    const banner = $('#watch-banner');
    banner.hidden = mine;
    $('#watch-text').textContent = !connected
      ? 'Not connected. Connect to a robot to see the joints.'
      : other
        ? 'Watching only. Another client has control.'
        : 'Watching only.';
    watchBtn.hidden = !connected;
    watchBtn.textContent = other ? 'Take over' : 'Take control';

    // The controls work only with control. Live mode replaces Move, Go to zero and the jog buttons.
    root.querySelectorAll<HTMLButtonElement>('.needs-control').forEach((b) => (b.disabled = !mine || busy || (live && b.matches('.not-live'))));
    faders.forEach((i) => (i.disabled = !mine));
    goalNums.forEach((i) => (i.disabled = !mine));
    speed.disabled = !mine;
    liveBox.disabled = !mine || busy || !liveSupported();
    liveBox.parentElement!.title = connected && !liveSupported()
      ? 'Live mode needs controller firmware 4.4 or later. Update it on the Setup page.'
      : 'Live: the arm follows the goals at once (up to the speed setting). Off: set the goals, then Move.';
    liveBox.checked = live;
    $<HTMLButtonElement>('#goal-current').disabled = !mine;
    gFader.disabled = gNum.disabled = !mine;
  }

  controlBtn.addEventListener('click', () => {
    if (link.inControl) setLive(false, 'you released control');
    link.requestControl(link.inControl ? 0 : 1);
  });
  takeoverBtn.addEventListener('click', () => link.requestControl(2));
  watchBtn.addEventListener('click', () => link.requestControl(link.last?.control === 2 ? 2 : 1));
  const stopAll = () => {
    setLive(false, '');
    link.stopRobot();
  };
  $('#stop-btn').addEventListener('click', stopAll);
  $('#hold-btn').addEventListener('click', () => {
    setLive(false, '');
    link.holdPose();
  });
  // Move and Go to zero use the Speed setting too. A minimum-jerk move peaks at 1.875 × distance / T, so
  // T = 1.875 × (largest joint distance) / speed. The ATOM refuses a T below its own minimum (speed and
  // acceleration limits, as motion::move_min_duration); then the page asks for the shortest move (0).
  function moveDuration(target: number[]): number {
    const s = link.last;
    if (!s) return 0;
    const v = Number(speed.value);
    let tSpeed = 0;
    let tMin = 0.2;
    target.forEach((g, j) => {
      const d = Math.abs(g - s.q[j]);
      tSpeed = Math.max(tSpeed, (1.875 * d) / v);
      tMin = Math.max(tMin, (1.875 * d) / P.MOVE_VMAX, Math.sqrt((5.77 * d) / P.AMAX[j]));
    });
    return tSpeed > tMin * 1.02 ? Math.min(tSpeed, 65) : 0; // u16 ms on the wire
  }
  const moveToGoal = (target: number[]) => link.moveTo(target, moveDuration(target));
  $('#zero-btn').addEventListener('click', () => moveToGoal([0, 0, 0, 0, 0, 0]));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && link.status === 'connected') stopAll();
  });

  // Jog: press and hold. The page sends JOG every 50 ms; the ATOM stops 200 ms after the last one.
  // The Speed setting is for Move, Go to zero and Live mode (up to 90 °/s); the jog buttons use at most 30 °/s.
  speed.addEventListener('input', () => {
    $('#jog-speed-value').textContent = `${speed.value} °/s`;
    sendLive();
  });
  strips.querySelectorAll<HTMLButtonElement>('.jog:not(.grip-btn)').forEach((b) => {
    const j = Number((b.closest('.strip') as HTMLElement).dataset.joint);
    const dir = Number(b.dataset.dir);
    const end = () => link.setJog(j, 0);
    b.addEventListener('pointerdown', (e) => {
      if (b.disabled) return;
      b.setPointerCapture(e.pointerId);
      link.setJog(j, dir * Math.min(P.JOG_VMAX, Number(speed.value))); // the jog buttons: at most 30 °/s
    });
    b.addEventListener('pointerup', end);
    b.addEventListener('pointercancel', end);
    b.addEventListener('lostpointercapture', end);
    b.addEventListener('contextmenu', (e) => e.preventDefault());
  });
  window.addEventListener('blur', () => {
    setLive(false, 'the page lost focus');
    link.stopJog();
  });
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) return;
    setLive(false, 'the page was hidden');
    link.stopJog();
  });

  // --- Goal pose: the faders and the typed values. The goal shows as a see-through arm when it
  // differs from the measured pose, and its value turns blue. ---
  function updateGoals() {
    const s = link.last;
    let differs = false;
    stripEls.forEach((row, j) => {
      const num = goalNums[j];
      if (document.activeElement !== num) num.value = goal[j].toFixed(1);
      const same = !s || Math.abs(goal[j] - s.q[j]) <= 1;
      num.toggleAttribute('data-same', same);
      differs ||= !same;
    });
    const gm = gMeasured();
    const gSame = !gripperShown || gm === null || Math.abs(gGoal - gm) <= 3;
    gNum.toggleAttribute('data-same', gSame);
    differs ||= !gSame;
    arm?.setGoal(s && differs ? goal : null);
  }
  const setGoal = (j: number, deg: number) => {
    goal[j] = clampGoal(j, deg);
    faders[j].value = String(goal[j]);
    updateGoals();
    sendLive();
  };
  faders.forEach((f, j) => f.addEventListener('input', () => setGoal(j, Number(f.value))));
  goalNums.forEach((num, j) => {
    const commit = () => {
      const v = Number(num.value.replace(',', '.'));
      if (num.value.trim() === '' || !Number.isFinite(v)) num.value = goal[j].toFixed(1);
      else setGoal(j, v);
      num.value = goal[j].toFixed(1);
    };
    num.addEventListener('change', commit);
    num.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        commit();
        num.select();
      } else if (e.key === 'Escape') {
        num.value = goal[j].toFixed(1);
        num.blur();
      }
    });
  });
  const setGoals = (q: number[]) => q.forEach((x, j) => setGoal(j, x));
  updateGoals();
  const gripperToCurrent = () => {
    const m = gMeasured();
    if (gripperShown && m !== null) setGripperGoal(m, false);
  };
  $('#goal-current').addEventListener('click', () => {
    if (link.last) setGoals(link.last.q);
    gripperToCurrent();
  });
  $('#goal-move').addEventListener('click', () => {
    moveToGoal(goal.slice());
    const m = gMeasured();
    if (gripperShown && m !== null && Math.abs(gGoal - m) > 1) sendGripper(gGoal / 100); // J7 with the joints
  });

  // --- Live mode (firmware 4.4+): the ATOM moves the joints to the goals at once (TRACK), at up to the
  // speed setting and the joints' acceleration limits, and stops on them. Never MOVE_TO. ---
  let live = false;
  const liveSupported = () => versionAtLeast(link.version, 4, 4);
  function setLive(on: boolean, why: string) {
    if (on === live) return;
    if (on) {
      if (!link.inControl || !link.last || !liveSupported()) return updateControls();
      setGoals(link.last.q); // start from where the arm is: no jump to an old goal
      gripperToCurrent();
      live = true;
      sendLive();
      toast('Live: the arm follows the faders and the typed goals. Esc stops.', 'warning');
    } else {
      live = false;
      link.stopTrack(); // HOLD: the arm brakes at once
      if (why) toast(`Live off: ${why}.`, 'info');
    }
    updateControls();
  }
  function sendLive() {
    if (live) link.setTrack(goal, Number(speed.value));
  }
  liveBox.addEventListener('change', () => setLive(liveBox.checked, 'you switched it off'));

  // Toasts for refused commands and finished moves.
  // The same message again within 3 s shows once.
  const recentToasts = new Map<string, number>();
  function toast(text: string, kind: 'info' | 'warning') {
    const now = performance.now();
    if (now - (recentToasts.get(text) ?? -Infinity) < 3000) return;
    recentToasts.set(text, now);
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
