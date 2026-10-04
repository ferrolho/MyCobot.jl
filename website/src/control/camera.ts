// Camera panel. Two sources:
// - the Pi camera, when the page is served by the lab service on the Pi (it answers /camera.json);
// - a camera on this computer (getUserMedia), for example the laptop's webcam pointed at the robot.
const $ = <T extends HTMLElement>(sel: string, root: ParentNode = document) => root.querySelector(sel) as T;

const PI = 'pi';

export async function setupCamera() {
  const card = $('#camera-card');
  const source = $<HTMLSelectElement>('#camera-source');
  const startBtn = $<HTMLButtonElement>('#camera-start');
  const img = $<HTMLImageElement>('#camera-img');
  const video = $<HTMLVideoElement>('#camera-video');
  const note = $('#camera-note');
  let stream: MediaStream | null = null;

  const piCamera = await fetch('/camera.json', { cache: 'no-store' })
    .then((r) => (r.ok ? r.json() : null))
    .catch(() => null);
  const local = window.isSecureContext && !!navigator.mediaDevices?.getUserMedia;

  const options: HTMLOptionElement[] = [];
  if (piCamera?.camera) options.push(new Option('Pi camera (next to the robot)', PI));
  if (local) options.push(new Option('A camera on this computer', 'local'));
  if (!options.length) {
    card.hidden = true;
    return;
  }
  source.replaceChildren(...options);

  function stop() {
    stream?.getTracks().forEach((t) => t.stop());
    stream = null;
    video.srcObject = null;
    video.hidden = true;
    img.removeAttribute('src');
    img.hidden = true;
    card.dataset.running = 'false';
    startBtn.textContent = 'Start';
  }

  async function start() {
    stop();
    const value = source.value;
    card.dataset.running = 'true';
    startBtn.textContent = 'Stop';
    if (value === PI) {
      img.hidden = false;
      img.src = `/camera.mjpg?t=${Date.now()}`;
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
  });
  startBtn.addEventListener('click', () => (card.dataset.running === 'true' ? stop() : start()));
  img.addEventListener('error', () => {
    if (card.dataset.running === 'true') note.textContent = 'The Pi camera does not answer. Another program can be using it.';
  });
  card.hidden = false;
  describe();
  stop();
  if (source.value === PI) start(); // the Pi stream needs no permission
}
