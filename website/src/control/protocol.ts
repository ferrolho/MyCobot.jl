// The ATOM's WebSocket API (draft for controller firmware 4.2): message codes, encoders and
// decoders. The bytes are the same as the UDP protocol. Docs: src/content/docs/comms/websocket-api.md.

import { N_ARM, SIGN, STEPS_PER_DEG, LIMIT_MIN, LIMIT_MAX, AMAX, VMAX } from './robot_params';

// Joint tables (J1-J6, then J7 = the gripper): SIGN, STEPS_PER_DEG, LIMIT_MIN/LIMIT_MAX (°) and AMAX
// (°/s²), generated from mycobot_description/config. The joint messages carry n joints: 6, or 7 when the
// ATOM finds the gripper (firmware 5.0+; the STREAM tells n).
export { N_ARM, SIGN, STEPS_PER_DEG, LIMIT_MIN, LIMIT_MAX, AMAX };
export const JOG_MARGIN = 2; // J1-J6: JOG stops and goals stay this far inside the limits (°)
/** The margin of joint j (°), as the firmware's lim::margin: J7 (the gripper) has none, end stop to end stop. */
export const margin = (j: number) => (j < N_ARM ? JOG_MARGIN : 0);
export const JOG_VMAX = 30; // °/s
export const TRACK_VMAX = VMAX; // °/s, Live mode (TRACK, firmware 4.4+): as MOVE_TO
export const MOVE_VMAX = VMAX; // °/s, MOVE_TO

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
  if ((code === Code.MOVE_TO || code === Code.TRACK) && status <= -11 && status >= -17) return `goal of J${-10 - status} is outside the limits`;
  if (status === -7) return 'J7 given, but no gripper found';
  return `status ${status}`;
}

export const DONE_RESULTS = ['done', 'tracking error', 'stopped', 'not at the start pose', 'bus error'];

// --- Decoding ----------------------------------------------------------------------------------

const signed15 = (v: number) => (v & 0x8000 ? -(v & 0x7fff) : v);
const signed10 = (v: number) => (v & 0x400 ? -(v & 0x3ff) : v & 0x3ff);

export const posToDeg = (j: number, pos: number) => (SIGN[j] * (pos - 2048)) / STEPS_PER_DEG;

/** One STREAM sample. The joint arrays have n entries: 6, or 7 with J7 (the gripper). */
export interface Stream {
  tMs: number;
  state: number;
  ok: boolean;
  q: number[]; // °
  goal: number[]; // °, the goal each joint holds or follows (firmware 5.0+; before: q)
  dq: number[]; // °/s
  load: number[]; // %
  temp: number[]; // °C
  volt: number[]; // V
  status: number[]; // servo status register (65)
  acc: number[]; // g
  gyro: number[]; // °/s
  control: number | null; // 0 nobody, 1 you, 2 another client; null before firmware 4.2
}

/** The STREAM of firmware 5.0+ (per-joint arrays of n, 21 + 11 n bytes), or of 4.2-4.7 (J1-J6 at fixed
 *  offsets, 74 or 79 bytes; the 4.6 gripper bytes are not read). */
export function decodeStream(v: DataView): Stream {
  const u16 = (o: number) => v.getUint16(o, true);
  const v5 = v.byteLength > 79;
  const n = v5 ? v.getUint8(8) : N_ARM;
  // Offsets of pos, goal, spd, load (u16 arrays), temp, volt, status (u8 arrays) and the IMU.
  const o = v5
    ? { pos: 9, goal: 9 + 2 * n, spd: 9 + 4 * n, load: 9 + 6 * n, temp: 9 + 8 * n, volt: 9 + 9 * n, status: 9 + 10 * n, imu: 9 + 11 * n }
    : { pos: 7, goal: 7, spd: 19, load: 31, temp: 43, volt: 49, status: 55, imu: 61 };
  const s: Stream = {
    tMs: v.getUint32(1, true),
    state: v.getUint8(5),
    ok: v.getUint8(6) === 1,
    q: [], goal: [], dq: [], load: [], temp: [], volt: [], status: [], acc: [], gyro: [],
    control: v5 ? v.getUint8(7) : v.byteLength >= 74 ? v.getUint8(73) : null,
  };
  for (let j = 0; j < n; j++) {
    s.q.push(posToDeg(j, u16(o.pos + 2 * j)));
    s.goal.push(posToDeg(j, u16(o.goal + 2 * j)));
    s.dq.push((SIGN[j] * signed15(u16(o.spd + 2 * j))) / STEPS_PER_DEG);
    s.load.push((SIGN[j] * signed10(u16(o.load + 2 * j))) / 10);
    s.temp.push(v.getUint8(o.temp + j));
    s.volt.push(v.getUint8(o.volt + j) / 10);
    s.status.push(v.getUint8(o.status + j));
  }
  for (let k = 0; k < 3; k++) {
    s.acc.push(v.getInt16(o.imu + 2 * k, true) / 4096);
    s.gyro.push(v.getInt16(o.imu + 6 + 2 * k, true) / 16.4);
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

/** Goal in degrees (6 joints, or 7 with J7); duration in seconds (0 = the ATOM chooses it). */
export function moveTo(goalDeg: number[], durationS = 0): Bytes {
  const n = goalDeg.length;
  const b = new DataView(new ArrayBuffer(3 + 2 * n));
  b.setUint8(0, Code.MOVE_TO);
  goalDeg.forEach((g, j) => b.setInt16(1 + 2 * j, Math.round(g * 100), true));
  b.setUint16(1 + 2 * n, Math.round(durationS * 1000), true);
  return new Uint8Array(b.buffer);
}

/** Live mode (firmware 4.4+): the goal pose (°; 6 joints, or 7 with J7) and a speed cap (°/s, ≤ TRACK_VMAX). */
export function track(goalDeg: number[], vmaxDegS: number): Bytes {
  const n = goalDeg.length;
  const b = new DataView(new ArrayBuffer(3 + 2 * n));
  b.setUint8(0, Code.TRACK);
  goalDeg.forEach((g, j) => b.setInt16(1 + 2 * j, Math.round(g * 100), true));
  b.setUint16(1 + 2 * n, Math.round(Math.max(0, Math.min(TRACK_VMAX, vmaxDegS)) * 10), true);
  return new Uint8Array(b.buffer);
}

/** Joint velocities in °/s (6 joints, or 7 with J7). */
export function jog(velDegS: number[]): Bytes {
  const b = new DataView(new ArrayBuffer(2 + 2 * velDegS.length));
  b.setUint8(0, Code.JOG);
  b.setUint8(1, 0); // frame 0: joints
  velDegS.forEach((v, j) => b.setInt16(2 + 2 * j, Math.round(v * 10), true));
  return new Uint8Array(b.buffer);
}

/** End-effector JOG (firmware 5.1): a twist of the TCP in the base frame (frame 1) or the tool frame (frame 2):
 *  linear mm/s, angular °/s; the J7 velocity (°/s, 0 without the gripper); the joint speed cap (°/s). The ATOM
 *  turns it into joint goals at 500 Hz (twist.h). Send it at least every 200 ms (deadman). */
export function eeJog(frame: 1 | 2, linMmS: readonly number[], angDegS: readonly number[], j7DegS: number, vmaxDegS: number): Bytes {
  const b = new DataView(new ArrayBuffer(18));
  b.setUint8(0, Code.JOG);
  b.setUint8(1, frame);
  const i16 = (x: number) => Math.max(-32768, Math.min(32767, Math.round(x * 10)));
  [...linMmS, ...angDegS, j7DegS].forEach((x, i) => b.setInt16(2 + 2 * i, i16(x), true));
  b.setUint16(16, Math.round(Math.max(0, Math.min(MOVE_VMAX, vmaxDegS)) * 10), true);
  return new Uint8Array(b.buffer);
}

// --- Addresses ---------------------------------------------------------------------------------

/**
 * "192.168.1.107", "mycobot.local", "localhost:8282" or a full ws:// URL → the WebSocket URL.
 * An address with a path keeps it: "raspberrypi5:8280/atom/ws" is the relay on the Pi.
 */
export function wsUrl(address: string): string {
  const a = address.trim();
  if (/^wss?:\/\//i.test(a)) return a;
  const rest = a.replace(/^https?:\/\//i, '');
  const i = rest.indexOf('/');
  const [host, path] = i < 0 ? [rest, ''] : [rest.slice(0, i), rest.slice(i)];
  return `ws://${host}${path.length > 1 ? path : '/ws'}`;
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
