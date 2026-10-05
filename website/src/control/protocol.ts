// The ATOM's WebSocket API (draft for controller firmware 4.2): message codes, encoders and
// decoders. The bytes are the same as the UDP protocol. Docs: src/content/docs/comms/websocket-api.md.

export const SIGN = [-1, -1, 1, -1, -1, -1] as const;
export const STEPS_PER_DEG = 4096 / 360;
export const LIMITS = [165, 140, 150, 150, 160, 180] as const; // degrees, from the URDF
export const JOG_MARGIN = 2; // JOG stops this far inside the limits (°)
export const JOG_VMAX = 30; // °/s
export const TRACK_VMAX = 90; // °/s, Live mode (TRACK, firmware 4.4+): as MOVE_TO
export const MOVE_VMAX = 90; // °/s, MOVE_TO (firmware motion_limits.h)
export const AMAX = [400, 400, 400, 2000, 2000, 2000]; // °/s², per joint (firmware motion_limits.h)

export const STATES = ['booting', 'holding', 'ready', 'playing', 'error', 'OTA update', 'moving', 'jogging', 'tracking'];
export const STATE_PLAYING = 3;
export const STATE_ERROR = 4;
export const STATE_MOVING = 6;
export const STATE_JOGGING = 7;

export const enum Code {
  PING = 0x01,
  STATE = 0x02,
  HOLD = 0x03,
  PLAY = 0x07,
  STOP = 0x08,
  REG_READ = 0x09,
  SUBSCRIBE = 0x0c,
  CONTROL = 0x0d,
  MOVE_TO = 0x0e,
  JOG = 0x0f,
  TRACK = 0x10,
  PONG = 0x81,
  ACK = 0x83,
  TELEM = 0x84,
  DONE = 0x85,
  STREAM = 0x88,
}

export const CODE_NAMES: Record<number, string> = {
  0x01: 'PING', 0x02: 'STATE', 0x03: 'HOLD', 0x04: 'PLAN_BEGIN', 0x05: 'PLAN_DATA', 0x06: 'PLAN_END',
  0x07: 'PLAY', 0x08: 'STOP', 0x09: 'REG_READ', 0x0a: 'REG_WRITE', 0x0b: 'PLAY_SIGNAL', 0x0c: 'SUBSCRIBE',
  0x0d: 'CONTROL', 0x0e: 'MOVE_TO', 0x0f: 'JOG', 0x10: 'TRACK',
};

export function ackText(code: number, status: number): string {
  if (status === 0) return 'OK';
  if (status === -1) return code === Code.CONTROL ? 'the robot moves' : 'busy';
  if (status === -2) return code === Code.CONTROL ? 'another client has control' : 'you do not have control';
  if ((code === Code.MOVE_TO || code === Code.TRACK) && status <= -11 && status >= -16) return `goal of J${-10 - status} is outside the limits`;
  return `status ${status}`;
}

export const DONE_RESULTS = ['done', 'tracking error', 'stopped', 'not at the start pose', 'bus error'];

// --- Decoding ----------------------------------------------------------------------------------

const signed15 = (v: number) => (v & 0x8000 ? -(v & 0x7fff) : v);
const signed10 = (v: number) => (v & 0x400 ? -(v & 0x3ff) : v & 0x3ff);

export const posToDeg = (j: number, pos: number) => (SIGN[j] * (pos - 2048)) / STEPS_PER_DEG;

export interface Stream {
  tMs: number;
  state: number;
  ok: boolean;
  q: number[]; // °
  dq: number[]; // °/s
  load: number[]; // %
  temp: number[]; // °C
  volt: number[]; // V
  status: number[]; // servo status register (65)
  acc: number[]; // g
  gyro: number[]; // °/s
  control: number | null; // 0 nobody, 1 you, 2 another client; null before firmware 4.2
}

export function decodeStream(v: DataView): Stream {
  const u16 = (o: number) => v.getUint16(o, true);
  const s: Stream = {
    tMs: v.getUint32(1, true),
    state: v.getUint8(5),
    ok: v.getUint8(6) === 1,
    q: [], dq: [], load: [], temp: [], volt: [], status: [], acc: [], gyro: [],
    control: v.byteLength >= 74 ? v.getUint8(73) : null,
  };
  for (let j = 0; j < 6; j++) {
    s.q.push(posToDeg(j, u16(7 + 2 * j)));
    s.dq.push((SIGN[j] * signed15(u16(19 + 2 * j))) / STEPS_PER_DEG);
    s.load.push((SIGN[j] * signed10(u16(31 + 2 * j))) / 10);
    s.temp.push(v.getUint8(43 + j));
    s.volt.push(v.getUint8(49 + j) / 10);
    s.status.push(v.getUint8(55 + j));
  }
  for (let k = 0; k < 3; k++) {
    s.acc.push(v.getInt16(61 + 2 * k, true) / 4096);
    s.gyro.push(v.getInt16(67 + 2 * k, true) / 16.4);
  }
  return s;
}

export interface Pong {
  version: string;
  state: number;
  planSamples: number;
  imuOk: boolean;
}

export function decodePong(v: DataView): Pong {
  const major = v.getUint16(1, true);
  const minor = v.byteLength >= 14 ? v.getUint8(12) : 0;
  const patch = v.byteLength >= 14 ? v.getUint8(13) : 0;
  return { version: `${major}.${minor}.${patch}`, state: v.getUint8(3), planSamples: v.getUint32(4, true), imuOk: v.getUint8(10) === 1 };
}

export interface Ack {
  code: number;
  status: number;
  value: number;
}

export const decodeAck = (v: DataView): Ack => ({ code: v.getUint8(1), status: v.getInt8(2), value: v.byteLength >= 7 ? v.getUint32(3, true) : 0 });

/** DONE: the result; for a tracking error (result 1) also the joint (1-6) and how far it was from its goal. */
export const decodeDone = (v: DataView) => ({
  result: v.getUint8(1),
  joint: v.byteLength >= 21 ? v.getUint8(18) : 0,
  errorDeg: v.byteLength >= 21 ? Math.abs(v.getInt16(19, true)) / STEPS_PER_DEG : 0,
});

// --- Encoding ----------------------------------------------------------------------------------

export type Bytes = Uint8Array<ArrayBuffer>;

export const ping = () => new Uint8Array([Code.PING]);
export const hold = () => new Uint8Array([Code.HOLD]);
export const stop = () => new Uint8Array([Code.STOP]);

export function subscribe(rateHz: number): Bytes {
  const b = new DataView(new ArrayBuffer(3));
  b.setUint8(0, Code.SUBSCRIBE);
  b.setUint16(1, rateHz, true);
  return new Uint8Array(b.buffer);
}

/** 0 release, 1 take, 2 take over. */
export const control = (action: 0 | 1 | 2) => new Uint8Array([Code.CONTROL, action]);

/** Goal in degrees; duration in seconds (0 = the ATOM chooses it). */
export function moveTo(goalDeg: number[], durationS = 0): Bytes {
  const b = new DataView(new ArrayBuffer(15));
  b.setUint8(0, Code.MOVE_TO);
  goalDeg.forEach((g, j) => b.setInt16(1 + 2 * j, Math.round(g * 100), true));
  b.setUint16(13, Math.round(durationS * 1000), true);
  return new Uint8Array(b.buffer);
}

/** Live mode (firmware 4.4+): the goal pose (°) and a speed cap (°/s, ≤ TRACK_VMAX). */
export function track(goalDeg: number[], vmaxDegS: number): Bytes {
  const b = new DataView(new ArrayBuffer(15));
  b.setUint8(0, Code.TRACK);
  goalDeg.forEach((g, j) => b.setInt16(1 + 2 * j, Math.round(g * 100), true));
  b.setUint16(13, Math.round(Math.max(0, Math.min(TRACK_VMAX, vmaxDegS)) * 10), true);
  return new Uint8Array(b.buffer);
}

/** Joint velocities in °/s. */
export function jog(velDegS: number[]): Bytes {
  const b = new DataView(new ArrayBuffer(14));
  b.setUint8(0, Code.JOG);
  b.setUint8(1, 0); // frame 0: joints
  velDegS.forEach((v, j) => b.setInt16(2 + 2 * j, Math.round(v * 10), true));
  return new Uint8Array(b.buffer);
}

// --- Addresses ---------------------------------------------------------------------------------

/** "192.168.1.107", "mycobot.local", "100.69.15.110:8281" or a full ws:// URL → the WebSocket URL. */
export function wsUrl(address: string): string {
  const a = address.trim();
  if (/^wss?:\/\//i.test(a)) return a;
  const host = a.replace(/^https?:\/\//i, '').replace(/\/.*$/, '');
  return `ws://${host}/ws`;
}

/** True if the browser treats the address as the local network (private IP or .local). */
export function isLocalAddress(address: string): boolean {
  const host = wsUrl(address).replace(/^wss?:\/\//, '').replace(/[:/].*$/, '');
  if (host.endsWith('.local') || host === 'localhost') return true;
  const m = host.match(/^(\d+)\.(\d+)\.(\d+)\.(\d+)$/);
  if (!m) return false;
  const [a, b] = [Number(m[1]), Number(m[2])];
  return a === 10 || a === 127 || (a === 172 && b >= 16 && b <= 31) || (a === 192 && b === 168) || (a === 169 && b === 254);
}
