// Setup page: shows the firmware version from the installer manifest, and finds the robot on the
// network with a WebSocket PING (ws://<address>/ws, firmware 4.2+; docs: comms/websocket-api).

const STATES = ['booting', 'holding', 'ready', 'playing', 'error', 'updating', 'moving', 'jogging'];

type Manifest = { name: string; version: string; git?: string };

const $ = <T extends HTMLElement = HTMLElement>(sel: string) => document.querySelector(sel) as T;

/** A private IPv4 address or a .local name: what an https page may reach on the local network. */
function isLocalAddress(a: string): boolean {
  if (/\.local\.?$/i.test(a) || a === 'localhost') return true;
  const m = a.match(/^(\d+)\.(\d+)\.(\d+)\.(\d+)$/);
  if (!m) return false;
  const [x, y] = [Number(m[1]), Number(m[2])];
  return x === 10 || x === 127 || (x === 172 && y >= 16 && y <= 31) || (x === 192 && y === 168) || (x === 169 && y === 254);
}

/** PONG: u8 0x81, u16 major, u8 state, u32 plan samples, u16 plan rate, u8 imu_ok, u8 gains_ok, u8 minor, u8 patch. */
function parsePong(b: Uint8Array) {
  if (b.length < 4 || b[0] !== 0x81) return null;
  const major = b[1] | (b[2] << 8);
  const version = b.length >= 14 ? `${major}.${b[12]}.${b[13]}` : `${major}`;
  return { version, state: STATES[b[3]] ?? `state ${b[3]}`, imu: b.length > 10 ? b[10] === 1 : undefined };
}

function newer(a: string, b: string): boolean {
  const pa = a.split('.').map(Number), pb = b.split('.').map(Number);
  for (let i = 0; i < 3; i++) if ((pa[i] ?? 0) !== (pb[i] ?? 0)) return (pa[i] ?? 0) > (pb[i] ?? 0);
  return false;
}

/** Opens ws://address/ws, sends PING and waits for PONG. */
function ping(address: string, timeoutMs = 5000): Promise<ReturnType<typeof parsePong>> {
  return new Promise((resolve, reject) => {
    let ws: WebSocket;
    try {
      ws = new WebSocket(`ws://${address}/ws`);
    } catch (e) {
      return reject(e);
    }
    ws.binaryType = 'arraybuffer';
    const timer = setTimeout(() => { ws.close(); reject(new Error('timeout')); }, timeoutMs);
    ws.onopen = () => ws.send(new Uint8Array([0x01]));
    ws.onmessage = (ev) => {
      if (!(ev.data instanceof ArrayBuffer)) return;   // text frames are the status log
      const pong = parsePong(new Uint8Array(ev.data));
      if (!pong) return;
      clearTimeout(timer);
      ws.close();
      resolve(pong);
    };
    ws.onerror = () => { clearTimeout(timer); reject(new Error('no connection')); };
  });
}

export function initSetup() {
  const root = $('#setup');
  if (!root) return;
  const manifestUrl = root.dataset.manifest!;
  const controlUrl = root.dataset.control!;
  let latest: string | null = null;

  // Web Serial is needed for steps 2 and 3 (the installer shows its own message too). Browsers
  // offer it only on secure pages: https, or http://localhost.
  const warning = $('#browser-warning');
  if (!window.isSecureContext) {
    warning.textContent = `This page is not secure (${location.protocol}//${location.host}), so the browser cannot use USB serial ports. Open the Setup page on https://ferrolho.github.io/mycobot-280-lab/setup/ or on localhost. Step 4 works here.`;
    warning.hidden = false;
  } else if (!('serial' in navigator)) warning.hidden = false;

  // Firmware version from the manifest. Without it (a dev server that did not build the firmware),
  // the install button stays off.
  const installer = $('#installer');
  const button = installer.querySelector('button') as HTMLButtonElement;
  fetch(manifestUrl, { cache: 'no-cache' })
    .then((r) => (r.ok ? (r.json() as Promise<Manifest>) : Promise.reject(new Error(`${r.status}`))))
    .then((m) => {
      latest = m.version;
      $('#fw-version').textContent = m.version;
      $('#fw-detail').textContent = `Public build${m.git ? ` (${m.git})` : ''}: no WiFi password inside. WiFi is set up in step 3.`;
      installer.setAttribute('manifest', manifestUrl);
    })
    .catch(() => {
      $('#fw-version').textContent = '';
      $('#fw-detail').textContent = 'The firmware files are not on this server. On a development server, run tools/build-public-firmware.sh.';
      button.disabled = true;
    });

  // Find the robot
  const form = $<HTMLFormElement>('#find-form');
  const addr = $<HTMLInputElement>('#find-addr');
  const result = $('#find-result');
  const text = $('#find-text');
  const open = $<HTMLAnchorElement>('#find-open');
  const btn = $<HTMLButtonElement>('#find-btn');
  const params = new URLSearchParams(location.search);
  if (params.get('atom')) addr.value = params.get('atom')!;

  const show = (status: string, msg: string) => {
    result.dataset.status = status;
    text.textContent = msg;
  };

  form.addEventListener('submit', async (e) => {
    e.preventDefault();
    const a = addr.value.trim().replace(/^wss?:\/\//, '').replace(/\/.*$/, '');
    if (!a) return addr.focus();
    open.hidden = true;
    if (location.protocol === 'https:' && !isLocalAddress(a)) {
      show('error', 'From this https page the browser can reach only a private IP address (for example 192.168.1.107) or a .local name.');
      return;
    }
    btn.disabled = true;
    show('searching', `Searching for ${a}…`);
    try {
      const pong = await ping(a);
      if (!pong) throw new Error('no reply');
      let msg = `Found the robot at ${a}: controller firmware ${pong.version}, ${pong.state}.`;
      if (latest && newer(latest, pong.version)) msg += ` Firmware ${latest} is available: update it in step 2.`;
      show('found', msg);
      open.href = `${controlUrl}?atom=${encodeURIComponent(a)}`;
      open.hidden = false;
    } catch {
      show('error', `No robot found at ${a}. Check that the 12 V supply is on, the LED matrix is green, and this computer is on the same WiFi network. If you use mycobot.local, try the IP address.`);
    } finally {
      btn.disabled = false;
    }
  });
}
