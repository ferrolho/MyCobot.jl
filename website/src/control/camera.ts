// Camera panel. Two sources:
// - the Pi camera, when the page is served by the lab service on the Pi (it answers /camera.json);
// - a camera on this computer (getUserMedia), for example the laptop's webcam pointed at the robot.
// With the Pi camera, the Overlay switch draws the robot model through the calibrated camera model
// (/lab/camera.json, camera_overlay.ts), if the lab service has one.
import type { ArmView } from './viewer3d';
import type { CameraModel, CameraOverlay } from './camera_overlay';

const $ = <T extends HTMLElement>(sel: string, root: ParentNode = document) => root.querySelector(sel) as T;

const PI = 'pi';
const OVERLAY_KEY = 'mycobot-control.camera-overlay';
const CAMERA_MODEL_POLL_MS = 5000;

const getJson = (url: string) =>
  fetch(url, { cache: 'no-store' })
    .then((r) => (r.ok ? r.json() : null))
    .catch(() => null);

/** `arm`: the 3D view, for the camera overlay. */
export async function setupCamera(arm?: Promise<ArmView>) {
  const card = $('#camera-card');
  const source = $<HTMLSelectElement>('#camera-source');
  const startBtn = $<HTMLButtonElement>('#camera-start');
  const img = $<HTMLImageElement>('#camera-img');
  const video = $<HTMLVideoElement>('#camera-video');
  const note = $('#camera-note');
  const overlaySwitch = $('#camera-overlay-switch');
  const overlayBox = $<HTMLInputElement>('#camera-overlay');
  let stream: MediaStream | null = null;

  const piCamera = await getJson('/camera.json');
  const local = window.isSecureContext && !!navigator.mediaDevices?.getUserMedia;

  const options: HTMLOptionElement[] = [];
  if (piCamera?.camera) options.push(new Option('Pi camera (next to the robot)', PI));
  if (local) options.push(new Option('A camera on this computer', 'local'));
  if (!options.length) {
    card.hidden = true;
    return;
  }
  source.replaceChildren(...options);

  // Camera overlay: only for the Pi camera, and only if the lab service has a camera model.
  let model: CameraModel | null = null;
  let modelText = '';
  let overlay: Promise<CameraOverlay> | null = null;
  let pollTimer = 0;
  const loadModel = async () => {
    const m = piCamera?.camera ? await getJson('/lab/camera.json') : null;
    const { isCameraModel } = m ? await import('./camera_overlay') : { isCameraModel: () => false };
    const text = JSON.stringify(m);
    if (text === modelText) return;
    modelText = text;
    model = isCameraModel(m) ? m : null;
    updateOverlay(); // it gives the new model to the overlay
  };
  try {
    overlayBox.checked = localStorage.getItem(OVERLAY_KEY) === 'on';
  } catch {
    overlayBox.checked = false;
  }
  function updateOverlay() {
    overlaySwitch.hidden = !model || source.value !== PI;
    const on = !overlaySwitch.hidden && overlayBox.checked && card.dataset.running === 'true';
    if (on && !overlay) {
      overlay = Promise.all([import('./camera_overlay'), arm]).then(([{ CameraOverlay }, a]) => {
        const o = new CameraOverlay($('.camera-view', card), img);
        if (a) o.setArm(a);
        return o;
      });
    }
    overlay?.then((o) => {
      if (model) o.setModel(model);
      o.setShown(!overlaySwitch.hidden && overlayBox.checked && card.dataset.running === 'true');
    });
    // Poll the camera model while the overlay is on: a calibration tool can change it.
    clearTimeout(pollTimer);
    if (on) pollTimer = window.setTimeout(() => loadModel().finally(updateOverlay), CAMERA_MODEL_POLL_MS);
  }
  overlayBox.addEventListener('change', () => {
    try {
      localStorage.setItem(OVERLAY_KEY, overlayBox.checked ? 'on' : 'off');
    } catch {
      // private window or blocked storage: the switch works, without memory
    }
    updateOverlay();
  });

  function stop() {
    stream?.getTracks().forEach((t) => t.stop());
    stream = null;
    video.srcObject = null;
    video.hidden = true;
    img.removeAttribute('src');
    img.hidden = true;
    card.dataset.running = 'false';
    startBtn.textContent = 'Start';
    updateOverlay();
  }

  async function start() {
    stop();
    const value = source.value;
    card.dataset.running = 'true';
    startBtn.textContent = 'Stop';
    if (value === PI) {
      img.hidden = false;
      img.src = `/camera.mjpg?t=${Date.now()}`;
      updateOverlay();
      return;
    }
    try {
      const deviceId = value.startsWith('device:') ? value.slice(7) : undefined;
      stream = await navigator.mediaDevices.getUserMedia({
        video: deviceId ? { deviceId: { exact: deviceId } } : { width: { ideal: 1280 } },
        audio: false,
      });
      video.srcObject = stream;
      video.hidden = false;
      await listDevices(stream.getVideoTracks()[0]?.getSettings().deviceId);
    } catch (err) {
      stop();
      note.textContent = `The browser did not open the camera: ${(err as Error).message}`;
    }
  }

  // After the first permission, the browser gives the camera names: list them as sources.
  async function listDevices(current?: string) {
    const cams = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
    const keep = options.filter((o) => o.value === PI);
    const opts = cams.map((d, k) => new Option(d.label || `Camera ${k + 1}`, `device:${d.deviceId}`));
    source.replaceChildren(...keep, ...opts);
    if (current) source.value = `device:${current}`;
  }

  function describe() {
    note.textContent =
      source.value === PI
        ? 'The webcam on the Raspberry Pi next to the robot.'
        : 'This camera must be connected to this computer (for example the laptop), pointed at the robot. The browser asks for permission first.';
  }

  source.addEventListener('change', () => {
    describe();
    if (card.dataset.running === 'true') start();
    else updateOverlay();
  });
  startBtn.addEventListener('click', () => (card.dataset.running === 'true' ? stop() : start()));
  img.addEventListener('error', () => {
    if (card.dataset.running === 'true') note.textContent = 'The Pi camera does not answer. Another program can be using it.';
  });
  card.hidden = false;
  describe();
  await loadModel();
  stop();
  if (source.value === PI) start(); // the Pi stream needs no permission
}
