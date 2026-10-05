// Session log for the lab: when the lab service on the Raspberry Pi serves this page (it answers
// /log.json), the page sends what happens to the Pi, which appends it to one JSONL file per session
// (~/myCobot/lab-logs/). The GitHub Pages site has no lab service, so nothing is logged there.
//
// Events: the user's clicks and input changes, every command sent (JOG and TRACK only when they change;
// no SUBSCRIBE or CONTROL lease renewals), every reply, the ATOM's status lines, the link status, the
// stream state at 10 Hz, every stream gap over 100 ms (with the ATOM's own time gap, to tell "the ATOM
// did not send" from "the network held the packets"), the stream statistics once a second (packets/s,
// largest gap and arrival jitter over the last 5 s), and page errors.
import type { AtomLink } from './connection';
import * as P from './protocol';

const FLUSH_MS = 2000;
const STATE_EVERY_MS = 100; // state at 10 Hz
const GAP_MS = 100; // log arrival gaps longer than this
const STATS_WINDOW_MS = 5000;

type Event = Record<string, unknown>;

const r2 = (x: number) => Math.round(x * 100) / 100;

export async function startSessionLog(link: AtomLink) {
  const ok = await fetch('/log.json', { cache: 'no-store' })
    .then((r) => r.ok)
    .catch(() => false);
  if (!ok) return;

  const session = `${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}-${Math.random().toString(36).slice(2, 6)}`;
  let queue: Event[] = [];
  const log = (type: string, data: Event = {}) => queue.push({ t: Date.now(), pt: Math.round(performance.now()), type, ...data });
  const flush = (beacon = false) => {
    if (!queue.length) return;
    const body = queue.map((e) => JSON.stringify(e)).join('\n') + '\n';
    queue = [];
    const url = `/log?session=${session}`;
    if (beacon) navigator.sendBeacon(url, body);
    else fetch(url, { method: 'POST', body, keepalive: true }).catch(() => {});
  };
  setInterval(flush, FLUSH_MS);
  addEventListener('pagehide', () => {
    log('page', { event: 'hide' });
    flush(true);
  });
  document.addEventListener('visibilitychange', () => log('page', { event: document.visibilityState }));

  log('session', { url: location.href, agent: navigator.userAgent, screen: [screen.width, screen.height], dpr: devicePixelRatio });

  // The user's actions: clicks on buttons and links, and changes of inputs (with the new value).
  const describe = (el: Element) => {
    const h = el as HTMLElement & { value?: string; checked?: boolean; type?: string };
    return {
      id: h.id || undefined,
      text: (h.getAttribute('aria-label') || h.textContent || '').trim().slice(0, 40) || undefined,
      joint: h.closest('[data-joint]')?.getAttribute('data-joint') ?? undefined,
    };
  };
  document.addEventListener('click', (e) => {
    const el = (e.target as Element).closest('button, a, input[type=checkbox]');
    if (el) log('click', describe(el));
  }, true);
  document.addEventListener('change', (e) => {
    const el = e.target as HTMLInputElement;
    log('input', { ...describe(el), value: el.type === 'checkbox' ? el.checked : el.value });
  }, true);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' || e.key === 'Enter') log('key', { key: e.key, ...describe(e.target as Element) });
  }, true);
  addEventListener('error', (e) => log('error', { message: e.message, source: `${e.filename}:${e.lineno}` }));
  addEventListener('unhandledrejection', (e) => log('error', { message: String(e.reason) }));

  // Commands sent to the ATOM.
  const send = link.send.bind(link);
  let lastJog = '';
  let lastTrack = '';
  let renewing = false;
  link.send = (bytes: P.Bytes) => {
    const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const code = v.getUint8(0);
    const name = P.CODE_NAMES[code] ?? `0x${code.toString(16)}`;
    if (code === P.Code.JOG) {
      const vel = Array.from({ length: 6 }, (_, j) => v.getInt16(2 + 2 * j, true) / 10);
      const key = vel.join(',');
      if (key !== lastJog) log('tx', { cmd: name, vel });
      lastJog = key;
    } else if (code === P.Code.TRACK) {
      const goal = Array.from({ length: 6 }, (_, j) => v.getInt16(1 + 2 * j, true) / 100);
      const vmax = v.getUint16(13, true) / 10;
      const key = `${goal.join(',')}@${vmax}`;
      if (key !== lastTrack) log('tx', { cmd: name, goal, vmax });
      lastTrack = key;
    } else if (code === P.Code.MOVE_TO) {
      log('tx', { cmd: name, goal: Array.from({ length: 6 }, (_, j) => v.getInt16(1 + 2 * j, true) / 100) });
    } else if (code === P.Code.CONTROL && v.getUint8(1) === 1 && link.inControl) {
      renewing = true; // the lease renewal every second: not logged
    } else if (code !== P.Code.SUBSCRIBE) {
      log('tx', { cmd: name, data: Array.from(bytes.slice(1)) });
    }
    send(bytes);
  };

  // Replies, status and the stream.
  link.on('status', ({ status, detail }) => {
    log('link', { status, detail, address: link.address });
    if (status !== 'connected') resetStream(); // no false gap or mixed statistics after a reconnect
  });
  link.on('pong', (p) => log('rx', { msg: 'PONG', version: p.version }));
  link.on('ack', (a) => {
    if (renewing && a.code === P.Code.CONTROL && a.status === 0) return void (renewing = false);
    log('rx', { msg: 'ACK', cmd: P.CODE_NAMES[a.code] ?? a.code, status: a.status, text: P.ackText(a.code, a.status) });
  });
  link.on('done', (d) => log('rx', { msg: 'DONE', ...d }));
  link.on('log', (line) => log('atom', { line }));

  let lastArrival = 0;
  let lastT = 0;
  let lastState = 0;
  const arrivals: { at: number; offset: number }[] = []; // arrival time, and arrival − ATOM time
  function resetStream() {
    lastArrival = 0;
    arrivals.length = 0;
  }
  link.on('stream', (s) => {
    const now = performance.now();
    arrivals.push({ at: now, offset: now - s.tMs });
    while (arrivals.length && now - arrivals[0].at > STATS_WINDOW_MS) arrivals.shift();
    if (lastArrival && now - lastArrival > GAP_MS) {
      log('gap', { gapMs: Math.round(now - lastArrival), atomGapMs: s.tMs - lastT, state: s.state });
    }
    lastArrival = now;
    lastT = s.tMs;
    if (now - lastState >= STATE_EVERY_MS) {
      lastState = now;
      log('state', { tMs: s.tMs, state: s.state, control: s.control, q: s.q.map(r2), dq: s.dq.map(r2), load: s.load.map(r2), temp: s.temp });
    }
  });
  setInterval(() => {
    if (link.status !== 'connected') return;
    const a = arrivals;
    if (a.length < 2) return log('stats', { rate: 0 });
    const now = performance.now();
    let maxGap = now - a[a.length - 1].at; // a stall counts too
    for (let i = 1; i < a.length; i++) maxGap = Math.max(maxGap, a[i].at - a[i - 1].at);
    const d = a.map((x) => x.offset).sort((x, y) => x - y); // arrival delay; the minimum is the clock offset
    const jitter = d[Math.min(d.length - 1, Math.floor(d.length * 0.95))] - d[0];
    log('stats', { rate: r2(a.length / ((now - a[0].at) / 1000 || 1)), maxGapMs: Math.round(maxGap), jitterMs: Math.round(jitter) });
  }, 1000);
}
