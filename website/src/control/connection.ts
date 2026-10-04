// One WebSocket connection to an ATOM (or to the simulated ATOM): state stream, control lease and
// jogging. The page uses the events; it never builds packets itself.
import * as P from './protocol';

export type LinkStatus = 'disconnected' | 'connecting' | 'connected';

export interface AtomEvents {
  status: { status: LinkStatus; detail?: string };
  pong: P.Pong;
  stream: P.Stream;
  ack: P.Ack;
  done: { result: number };
  log: string;
}

const STREAM_RATE_HZ = 50;
const RENEW_MS = 1000; // SUBSCRIBE and CONTROL renewals
const JOG_PERIOD_MS = 50; // the deadman fires after 200 ms
const RETRY_MS = [1000, 2000, 5000];

export class AtomLink {
  private ws: WebSocket | null = null;
  private target = new EventTarget();
  private renewTimer = 0;
  private jogTimer = 0;
  private retryTimer = 0;
  private retries = 0;
  private wanted = false; // the user wants to be connected
  private jogVel = [0, 0, 0, 0, 0, 0];
  address = '';
  status: LinkStatus = 'disconnected';
  version = '';
  last: P.Stream | null = null;
  /** True while this client has control (from the ACKs; from the STREAM byte with firmware 4.2). */
  inControl = false;

  on<K extends keyof AtomEvents>(type: K, fn: (e: AtomEvents[K]) => void) {
    this.target.addEventListener(type, (e) => fn((e as CustomEvent).detail));
  }

  private emit<K extends keyof AtomEvents>(type: K, detail: AtomEvents[K]) {
    this.target.dispatchEvent(new CustomEvent(type, { detail }));
  }

  private setStatus(status: LinkStatus, detail?: string) {
    this.status = status;
    this.emit('status', { status, detail });
  }

  connect(address: string) {
    this.disconnect();
    this.address = address;
    this.wanted = true;
    this.retries = 0;
    this.open();
  }

  disconnect() {
    this.wanted = false;
    clearTimeout(this.retryTimer);
    this.stopJog();
    if (this.ws) {
      if (this.inControl) this.send(P.control(0));
      this.ws.onclose = null;
      this.ws.close();
      this.ws = null;
    }
    this.cleanup();
    this.setStatus('disconnected');
  }

  private cleanup() {
    clearInterval(this.renewTimer);
    this.inControl = false;
    this.last = null;
  }

  private open() {
    let url: string;
    try {
      url = P.wsUrl(this.address);
      this.ws = new WebSocket(url);
    } catch (err) {
      this.setStatus('disconnected', String(err));
      return;
    }
    this.setStatus('connecting');
    const ws = this.ws;
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => {
      this.retries = 0;
      this.setStatus('connected');
      this.send(P.ping());
      this.send(P.subscribe(STREAM_RATE_HZ));
      this.renewTimer = window.setInterval(() => {
        this.send(P.subscribe(STREAM_RATE_HZ));
        if (this.inControl) this.send(P.control(1));
      }, RENEW_MS);
    };
    ws.onmessage = (e) => this.receive(e.data);
    ws.onclose = (e) => {
      this.ws = null;
      this.cleanup();
      this.stopJog(false);
      if (!this.wanted) return this.setStatus('disconnected');
      const wait = RETRY_MS[Math.min(this.retries++, RETRY_MS.length - 1)];
      this.setStatus('connecting', e.code === 1006 ? `no answer from ${url}, retry in ${wait / 1000} s` : `closed (${e.code}), retry`);
      this.retryTimer = window.setTimeout(() => this.wanted && this.open(), wait);
    };
  }

  private receive(data: ArrayBuffer | string) {
    if (typeof data === 'string') return this.emit('log', data);
    const v = new DataView(data);
    if (v.byteLength === 0) return;
    switch (v.getUint8(0)) {
      case P.Code.STREAM: {
        const s = P.decodeStream(v);
        if (s.control !== null) this.inControl = s.control === 1;
        this.last = s;
        if (s.state !== P.STATE_JOGGING && this.jogTimer && !this.jogVel.some((x) => x)) this.stopJog(false);
        this.emit('stream', s);
        break;
      }
      case P.Code.PONG: {
        const p = P.decodePong(v);
        this.version = p.version;
        this.emit('pong', p);
        break;
      }
      case P.Code.ACK: {
        const a = P.decodeAck(v);
        if (a.code === P.Code.CONTROL) this.inControl = a.status === 0 && this.pendingControl !== 0;
        if (a.status === -2 && a.code !== P.Code.CONTROL) this.inControl = false;
        if (a.code === P.Code.JOG && a.status !== 0) this.stopJog(false);
        this.emit('ack', a);
        break;
      }
      case P.Code.DONE:
        this.emit('done', P.decodeDone(v));
        break;
    }
  }

  send(bytes: P.Bytes) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(bytes);
  }

  private pendingControl: 0 | 1 | 2 = 0;

  /** 0 release, 1 take, 2 take over. */
  requestControl(action: 0 | 1 | 2) {
    this.pendingControl = action;
    if (action === 0) this.inControl = false;
    this.send(P.control(action));
  }

  stopRobot() {
    this.stopJog(false);
    this.send(P.stop());
  }

  holdPose() {
    this.stopJog(false);
    this.send(P.hold());
  }

  moveTo(goalDeg: number[], durationS = 0) {
    this.send(P.moveTo(goalDeg, durationS));
  }

  /** Jog one joint at `vel` °/s (0 stops that joint). The deadman stops the robot if the page stops sending. */
  setJog(joint: number, vel: number) {
    this.jogVel[joint] = Math.max(-P.JOG_VMAX, Math.min(P.JOG_VMAX, vel));
    if (this.jogVel.some((x) => x)) {
      if (!this.jogTimer) {
        this.send(P.jog(this.jogVel));
        this.jogTimer = window.setInterval(() => this.send(P.jog(this.jogVel)), JOG_PERIOD_MS);
      }
    } else {
      this.stopJog();
    }
  }

  /** Stop every jog. With `send`, also tell the ATOM (all velocities zero). */
  stopJog(send = true) {
    const wasJogging = !!this.jogTimer;
    clearInterval(this.jogTimer);
    this.jogTimer = 0;
    this.jogVel = [0, 0, 0, 0, 0, 0];
    if (send && wasJogging) this.send(P.jog(this.jogVel));
  }
}
