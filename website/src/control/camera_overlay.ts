// Camera overlay: the robot model, drawn through the calibrated camera model on top of the Pi camera
// image. If the drawing sits on the real robot, the camera calibration and the robot model agree.
// It draws, for the arm's meshes at the measured pose (the meshes and joints of the 3D view): the
// crease edges and the outline (the contour, where the surface turns away from the camera), both with
// hidden lines removed. Also the six screws on the base plate (A-F) and the marks of the camera model
// file. The camera model is /lab/camera.json (docs: software/control-page, "Camera overlay").
import * as THREE from 'three';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import { LineSegments2 } from 'three/examples/jsm/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/examples/jsm/lines/LineSegmentsGeometry.js';
import type { ArmView } from './viewer3d';

/**
 * The camera model (OpenCV pinhole). X_cam = R(rvec) · X_base + tvec (Rodrigues vector, mm, base
 * frame); u = f·x'·d + cx, v = f·y'·d + cy with x' = x/z, y' = y/z, d = 1 + k1·(x'² + y'²), in pixels of
 * a width × height image. `marks`: points found in that image, drawn as they are.
 */
export type CameraModel = {
  updated?: string;
  source?: string;
  width: number;
  height: number;
  rvec: number[];
  tvec: number[];
  f: number;
  cx: number;
  cy: number;
  k1?: number;
  marks?: { name: string; u: number; v: number }[];
};

/** The brass screws on the top face of the base plate: base frame, mm. CAD holes of G_base (mesh frame) + (0, 0, −32). */
export const BASE_SCREWS: [string, number, number, number][] = [
  ['A', 65, -46, 0],
  ['B', 47, -40, 0],
  ['C', 47, 40, 0],
  ['D', 65, 46, 0],
  ['E', -47, 40, 0],
  ['F', -65, 46, 0],
];

// Colours that show on a white arm and a light wooden table.
const ROBOT = '#ff1fd2'; // magenta
const OUTLINE = '#ff8c00'; // orange
const SCREW = '#2bff6a'; // green
const MARK = '#19d3ff'; // cyan
const EDGE_ANGLE = 30; // degrees: an edge between two faces is drawn if they meet at more than this
const LINE_WIDTH = 1.5; // CSS pixels

export function isCameraModel(m: unknown): m is CameraModel {
  const c = m as CameraModel;
  const num = (x: unknown) => typeof x === 'number' && Number.isFinite(x);
  const vec3 = (v: unknown) => Array.isArray(v) && v.length === 3 && v.every(num);
  return !!c && num(c.width) && num(c.height) && c.width > 0 && c.height > 0 && vec3(c.rvec) && vec3(c.tvec) && num(c.f) && num(c.cx) && num(c.cy);
}

/**
 * The edges of a mesh, for the outline and the crease edges. Built once for each geometry, in its own
 * coordinates. The vertices are welded by position (a glTF mesh splits them at creases).
 */
type Topology = {
  vertices: Float32Array; // xyz of each welded vertex
  planes: Float64Array; // each face: its normal n (not unit, from the winding) and n·p of a corner
  edges: Uint32Array; // each edge with two faces: vertex a, vertex b, face 1, face 2
  crease: Uint8Array; // each edge with two faces: 1 if its faces meet at more than EDGE_ANGLE
  lone: Float32Array; // the edges with one face or more than two (xyz xyz each): always crease edges
  front: Uint8Array; // each face: 1 if it faces the camera (work space)
};

function topology(g: THREE.BufferGeometry): Topology {
  const p = g.getAttribute('position');
  const index = g.getIndex();
  const id = new Uint32Array(p.count);
  const weld = new Map<string, number>();
  const verts: number[] = [];
  for (let i = 0; i < p.count; i++) {
    const x = p.getX(i), y = p.getY(i), z = p.getZ(i);
    const key = `${x},${y},${z}`;
    let k = weld.get(key);
    if (k === undefined) {
      weld.set(key, (k = verts.length / 3));
      verts.push(x, y, z);
    }
    id[i] = k;
  }
  const nv = verts.length / 3;
  const nFaces = Math.floor((index ? index.count : p.count) / 3);
  const planes = new Float64Array(nFaces * 4);
  const slot = new Map<number, number>(); // edge (a·nv + b, a < b) → its index in `edges`
  const edges: number[] = []; // a, b, face 1, face 2 (−1: none yet; −2: more than two faces)
  const v = [0, 0, 0];
  let f = 0;
  for (let t = 0; t < nFaces; t++) {
    for (let k = 0; k < 3; k++) v[k] = id[index ? index.getX(3 * t + k) : 3 * t + k];
    const a = 3 * v[0], b = 3 * v[1], c = 3 * v[2];
    const ux = verts[b] - verts[a], uy = verts[b + 1] - verts[a + 1], uz = verts[b + 2] - verts[a + 2];
    const wx = verts[c] - verts[a], wy = verts[c + 1] - verts[a + 1], wz = verts[c + 2] - verts[a + 2];
    const nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
    // A face with no area has no direction: leave it out (its edges then have one face, and no outline).
    if (nx * nx + ny * ny + nz * nz <= 1e-12 * (ux * ux + uy * uy + uz * uz) * (wx * wx + wy * wy + wz * wz)) continue;
    planes[4 * f] = nx;
    planes[4 * f + 1] = ny;
    planes[4 * f + 2] = nz;
    planes[4 * f + 3] = nx * verts[a] + ny * verts[a + 1] + nz * verts[a + 2];
    for (let k = 0; k < 3; k++) {
      const lo = Math.min(v[k], v[(k + 1) % 3]), hi = Math.max(v[k], v[(k + 1) % 3]);
      const key = lo * nv + hi;
      const e = slot.get(key);
      if (e === undefined) {
        slot.set(key, edges.length);
        edges.push(lo, hi, f, -1);
      } else edges[e + 3] = edges[e + 3] === -1 ? f : -2;
    }
    f++;
  }
  const kept: number[] = [], crease: number[] = [], lone: number[] = [];
  const cosMax = Math.cos((EDGE_ANGLE * Math.PI) / 180);
  for (let e = 0; e < edges.length; e += 4) {
    const a = 3 * edges[e], b = 3 * edges[e + 1], f1 = 4 * edges[e + 2], f2 = 4 * edges[e + 3];
    if (edges[e + 3] < 0) {
      lone.push(verts[a], verts[a + 1], verts[a + 2], verts[b], verts[b + 1], verts[b + 2]);
      continue;
    }
    kept.push(edges[e], edges[e + 1], edges[e + 2], edges[e + 3]);
    const dot = planes[f1] * planes[f2] + planes[f1 + 1] * planes[f2 + 1] + planes[f1 + 2] * planes[f2 + 2];
    const n1 = Math.hypot(planes[f1], planes[f1 + 1], planes[f1 + 2]), n2 = Math.hypot(planes[f2], planes[f2 + 1], planes[f2 + 2]);
    crease.push(dot < cosMax * n1 * n2 ? 1 : 0);
  }
  return {
    vertices: new Float32Array(verts), planes: planes.slice(0, 4 * f), edges: new Uint32Array(kept),
    crease: new Uint8Array(crease), lone: new Float32Array(lone), front: new Uint8Array(f),
  };
}

/**
 * The lines for a camera at `c` (geometry coordinates). The outline: the edges between a face that faces
 * the camera and a face that does not. The crease edges: the other edges whose faces meet at more than
 * EDGE_ANGLE, and the lone edges. An edge is in one of the two, never both: an outline edge is drawn
 * once, in the outline colour. Writes the end points (xyz xyz each) to `outline` and `crease`, which must
 * have room for every edge, and returns the numbers of edges.
 */
function splitEdges(t: Topology, c: THREE.Vector3, outline: Float32Array, crease: Float32Array): [number, number] {
  const { planes: P, edges: E, vertices: V, front } = t;
  for (let f = 0, i = 0; f < front.length; f++, i += 4) front[f] = P[i] * c.x + P[i + 1] * c.y + P[i + 2] * c.z > P[i + 3] ? 1 : 0;
  let no = 0, nc = 0;
  for (let e = 0, k = 0; e < E.length; e += 4, k++) {
    const out = front[E[e + 2]] !== front[E[e + 3]];
    if (!out && !t.crease[k]) continue;
    const a = 3 * E[e], b = 3 * E[e + 1], dst = out ? outline : crease, o = 6 * (out ? no++ : nc++);
    dst[o] = V[a];
    dst[o + 1] = V[a + 1];
    dst[o + 2] = V[a + 2];
    dst[o + 3] = V[b];
    dst[o + 4] = V[b + 1];
    dst[o + 5] = V[b + 2];
  }
  crease.set(t.lone, 6 * nc);
  return [no, nc + t.lone.length / 6];
}

/** R(rvec), the Rodrigues rotation. */
function rotation(rvec: number[]): THREE.Matrix4 {
  const axis = new THREE.Vector3(...rvec);
  const angle = axis.length();
  return angle < 1e-12 ? new THREE.Matrix4() : new THREE.Matrix4().makeRotationAxis(axis.divideScalar(angle), angle);
}

/** A base-frame point (mm) in image pixels of the model (u, v), or null if it is behind the camera. */
export function project(m: CameraModel, p: number[]): [number, number] | null {
  const c = new THREE.Vector3(p[0], p[1], p[2]).applyMatrix4(rotation(m.rvec)).add(new THREE.Vector3(...m.tvec));
  if (c.z <= 0) return null;
  const x = c.x / c.z, y = c.y / c.z;
  const d = 1 + (m.k1 ?? 0) * (x * x + y * y);
  return [m.f * x * d + m.cx, m.f * y * d + m.cy];
}

/** One mesh of the arm in the overlay. `room`: the edges that each line buffer holds; `seen`: what the lines were computed for. */
type Holder = { mesh: THREE.Mesh; holder: THREE.Group; lines: LineSegments2; outline: LineSegments2; room: number; seen: number[] };

export class CameraOverlay {
  private root = document.createElement('div');
  private gl: THREE.WebGLRenderer | null = null;
  private canvas2d = document.createElement('canvas');
  private legend = document.createElement('div');
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera();
  private k1 = { value: 0 };
  private lineMat: LineMaterial;
  private outlineMat: LineMaterial;
  private depthMat: THREE.MeshBasicMaterial;
  /** False if the line shader did not take the distortion patch (a three.js update): the legend then says that k1 is ignored. */
  private distorts = true;
  private topologies = new WeakMap<THREE.BufferGeometry, Topology>();
  /** One holder per mesh of the arm: its edges, its outline and a depth-only copy (so that hidden lines do not show). */
  private holders: Holder[] = [];
  /** Goes up when the camera model changes: every outline is then out of date. */
  private modelVersion = 0;
  private robot: unknown = null;
  private model: CameraModel | null = null;
  private arm: ArmView | null = null;
  private shown = false;
  private drawn = ''; // what the last frame drew: the arm's revision, the size and the model
  private frame = 0;
  private legendHtml = '';

  /** `view`: the element that holds the camera image; `img`: the image (object-fit: contain). */
  constructor(private view: HTMLElement, private img: HTMLImageElement) {
    this.root.className = 'camera-overlay';
    this.root.hidden = true;
    this.canvas2d.setAttribute('aria-hidden', 'true');
    this.legend.className = 'overlay-legend';
    try {
      this.gl = new THREE.WebGLRenderer({ antialias: true, alpha: true, premultipliedAlpha: false });
      this.gl.setClearColor(0x000000, 0);
      this.gl.domElement.setAttribute('aria-hidden', 'true');
      this.root.append(this.gl.domElement);
    } catch {
      this.gl = null; // no WebGL: the screws and marks only
    }
    this.root.append(this.canvas2d, this.legend);
    view.append(this.root);

    // OpenCV's radial distortion (k1) in the vertex shaders: in camera space, x and y (at the same z)
    // scale by 1 + k1·r², r² = (x² + y²) / z². The same for the lines and for the depth-only meshes.
    const r2 = (v: string) => `(dot(${v}.xy, ${v}.xy) / (${v}.z * ${v}.z))`;
    this.depthMat = new THREE.MeshBasicMaterial({ colorWrite: false, polygonOffset: true, polygonOffsetFactor: 2, polygonOffsetUnits: 2 });
    this.depthMat.onBeforeCompile = (shader) => {
      shader.uniforms.k1 = this.k1;
      shader.vertexShader = shader.vertexShader
        .replace('void main() {', 'uniform float k1;\nvoid main() {')
        .replace('#include <project_vertex>', `#include <project_vertex>\n  gl_Position = projectionMatrix * vec4(mvPosition.xy * (1.0 + k1 * ${r2('mvPosition')}), mvPosition.zw);`);
    };
    // Lines of a fixed width in CSS pixels (WebGL draws plain lines 1 device pixel wide: too thin to see).
    // They do not write depth: only the depth-only meshes hide lines.
    const end = 'vec4 end = modelViewMatrix * vec4( instanceEnd, 1.0 );';
    const lineMaterial = (color: string, opacity: number) => {
      const mat = new LineMaterial({ color, linewidth: LINE_WIDTH, transparent: true, opacity, depthWrite: false });
      this.distorts &&= mat.vertexShader.includes(end);
      mat.uniforms.k1 = this.k1;
      mat.vertexShader = mat.vertexShader
        .replace('void main() {', 'uniform float k1;\nvoid main() {')
        .replace(end, `${end}\n  start.xy *= 1.0 + k1 * ${r2('start')};\n  end.xy *= 1.0 + k1 * ${r2('end')};`);
      return mat;
    };
    this.lineMat = lineMaterial(ROBOT, 0.8);
    this.outlineMat = lineMaterial(OUTLINE, 0.9);
    this.camera.matrixAutoUpdate = false;
    this.camera.matrixWorldAutoUpdate = false;
  }

  setArm(arm: ArmView) {
    this.arm = arm;
    this.drawn = '';
  }

  setModel(model: CameraModel) {
    this.model = model;
    this.k1.value = model.k1 ?? 0;
    // View matrix: OpenCV camera (x right, y down, looking along +z) to three.js camera (y up, looking along −z).
    const view = new THREE.Matrix4().multiplyMatrices(
      new THREE.Matrix4().makeScale(1, -1, -1),
      rotation(model.rvec).setPosition(model.tvec[0] / 1000, model.tvec[1] / 1000, model.tvec[2] / 1000),
    );
    this.camera.matrixWorldInverse.copy(view);
    this.camera.matrixWorld.copy(view).invert();
    // Projection: model pixels (centre of the first pixel at 0, 0) to clip space. The canvas covers the
    // image exactly, so the same matrix works for any display size (and for the 640×480 stream).
    const { width: W, height: H, f, cx, cy } = model;
    const near = 0.05, far = 5;
    this.camera.projectionMatrix.set(
      (2 * f) / W, 0, 1 - (2 * (cx + 0.5)) / W, 0,
      0, (2 * f) / H, (2 * (cy + 0.5)) / H - 1, 0,
      0, 0, -(far + near) / (far - near), (-2 * far * near) / (far - near),
      0, 0, -1, 0,
    );
    this.camera.projectionMatrixInverse.copy(this.camera.projectionMatrix).invert();
    this.modelVersion++;
    this.drawn = '';
  }

  setShown(on: boolean) {
    this.shown = on;
    this.root.hidden = !on;
    this.drawn = '';
    if (on && !this.frame) this.loop();
  }

  private loop = () => {
    this.frame = this.shown ? requestAnimationFrame(this.loop) : 0;
    if (this.shown) this.draw();
  };

  /** The rectangle of the image in the view (object-fit: contain), in CSS pixels. */
  private imageRect() {
    const w = this.view.clientWidth, h = this.view.clientHeight;
    const m = this.model!;
    const aspect = this.img.naturalWidth > 0 ? this.img.naturalWidth / this.img.naturalHeight : m.width / m.height;
    const s = Math.min(w / aspect, h);
    return { left: (w - s * aspect) / 2, top: (h - s) / 2, width: s * aspect, height: s, aspect };
  }

  private syncArm(robot: THREE.Object3D) {
    if (robot === this.robot) return;
    for (const { holder } of this.holders) this.scene.remove(holder);
    this.holders = [];
    this.robot = robot;
    robot.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh) return;
      const holder = new THREE.Group();
      holder.matrixAutoUpdate = false;
      const depth = new THREE.Mesh(mesh.geometry, this.depthMat);
      depth.renderOrder = 0;
      // Both line sets change with the view (an outline edge is not drawn as a crease edge too): each
      // holder has its own buffers (updateLines).
      const lines = new LineSegments2(new LineSegmentsGeometry(), this.lineMat);
      lines.renderOrder = 1;
      const outline = new LineSegments2(new LineSegmentsGeometry(), this.outlineMat);
      outline.renderOrder = 2;
      for (const l of [lines, outline]) {
        l.frustumCulled = false;
        l.visible = false;
      }
      holder.add(depth, lines, outline);
      this.scene.add(holder);
      this.holders.push({ mesh, holder, lines, outline, room: 0, seen: [] });
    });
  }

  /** The outline and crease edges of one mesh for the camera model and the pose. Computed again only if the mesh moved or the camera model changed. */
  private updateLines(h: Holder, camera: THREE.Vector3) {
    const seen = [this.modelVersion, ...h.holder.matrix.elements];
    if (seen.every((x, k) => x === h.seen[k])) return;
    h.seen = seen;
    const g = h.mesh.geometry;
    let t = this.topologies.get(g);
    if (!t) this.topologies.set(g, (t = topology(g)));
    const need = t.edges.length / 4 + t.lone.length / 6;
    if (need > h.room) {
      // Room for every edge of the mesh, made once; each update sends only the edges drawn to the GPU.
      for (const l of [h.lines, h.outline]) {
        l.geometry.dispose();
        l.geometry = new LineSegmentsGeometry().setPositions(new Float32Array(6 * need));
      }
      h.room = need;
    }
    const data = (l: LineSegments2) => (l.geometry.getAttribute('instanceStart') as THREE.InterleavedBufferAttribute).data;
    const c = camera.clone().applyMatrix4(h.holder.matrix.clone().invert()); // the camera in the mesh's coordinates
    const counts = splitEdges(t, c, data(h.outline).array as Float32Array, data(h.lines).array as Float32Array);
    [h.outline, h.lines].forEach((l, k) => {
      const d = data(l);
      d.clearUpdateRanges();
      d.addUpdateRange(0, 6 * counts[k]);
      d.needsUpdate = true;
      l.geometry.instanceCount = counts[k];
      l.visible = counts[k] > 0;
    });
  }

  private draw() {
    const m = this.model;
    if (!m) return;
    const robot = this.arm?.measured ?? null;
    const r = this.imageRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const size = `${r.left},${r.top},${r.width},${r.height},${dpr}`;
    const key = `${this.arm?.revision}|${robot ? 1 : 0}|${size}`;
    if (key === this.drawn) return;
    const resized = !this.drawn.endsWith(`|${size}`);
    this.drawn = key;
    if (resized) {
      Object.assign(this.root.style, { left: `${r.left}px`, top: `${r.top}px`, width: `${r.width}px`, height: `${r.height}px` });
      this.gl?.setPixelRatio(dpr);
      this.gl?.setSize(r.width, r.height);
      this.lineMat.resolution.set(r.width, r.height);
      this.canvas2d.width = Math.round(r.width * dpr);
      this.canvas2d.height = Math.round(r.height * dpr);
    }

    if (this.gl) {
      if (robot) {
        this.syncArm(robot);
        // Each mesh in the base frame: (robot root)⁻¹ · (mesh in the 3D view's world).
        robot.updateMatrixWorld(true);
        const toBase = robot.matrixWorld.clone().invert();
        const camera = new THREE.Vector3().setFromMatrixPosition(this.camera.matrixWorld); // base frame, m
        for (const h of this.holders) {
          h.holder.matrix.multiplyMatrices(toBase, h.mesh.matrixWorld);
          h.holder.visible = h.mesh.visible;
          if (h.holder.visible) this.updateLines(h, camera);
        }
        this.scene.updateMatrixWorld(true);
      }
      this.gl.render(this.scene, this.camera);
    }

    // Screws and marks: 2D, in CSS pixels of the canvas (s: canvas pixels per model pixel).
    const ctx = this.canvas2d.getContext('2d')!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, r.width, r.height);
    const s = r.width / m.width;
    const at = (u: number, v: number): [number, number] => [(u + 0.5) * s, (v + 0.5) * s];
    ctx.font = '600 11px system-ui, sans-serif';
    ctx.lineJoin = 'round';
    const label = (text: string, x: number, y: number, color: string) => {
      const w = ctx.measureText(text).width;
      x = Math.min(x, r.width - w - 2); // keep it in the image
      ctx.lineWidth = 3;
      ctx.strokeStyle = 'rgb(0 0 0 / 0.75)';
      ctx.strokeText(text, x, y);
      ctx.fillStyle = color;
      ctx.fillText(text, x, y);
    };
    for (const [name, x, y, z] of BASE_SCREWS) {
      const p = project(m, [x, y, z]);
      if (!p) continue;
      const [cx, cy] = at(...p);
      ctx.beginPath();
      ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
      ctx.lineWidth = 3;
      ctx.strokeStyle = 'rgb(0 0 0 / 0.6)';
      ctx.stroke();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = SCREW;
      ctx.stroke();
      ctx.fillStyle = SCREW;
      ctx.fillRect(cx - 0.75, cy - 0.75, 1.5, 1.5);
      label(name, cx + 7, cy - 6, SCREW);
    }
    for (const mark of m.marks ?? []) {
      if (!Number.isFinite(mark.u) || !Number.isFinite(mark.v)) continue;
      const [cx, cy] = at(mark.u, mark.v);
      ctx.beginPath();
      ctx.moveTo(cx - 6, cy - 6);
      ctx.lineTo(cx + 6, cy + 6);
      ctx.moveTo(cx - 6, cy + 6);
      ctx.lineTo(cx + 6, cy - 6);
      ctx.lineWidth = 3;
      ctx.strokeStyle = 'rgb(0 0 0 / 0.6)';
      ctx.stroke();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = MARK;
      ctx.stroke();
      label(String(mark.name ?? ''), cx + 7, cy + 14, MARK);
    }

    // Legend: what each colour is, the date of the camera model, and any warnings.
    const notes: string[] = [];
    if (!this.gl) notes.push('No WebGL: the arm is not drawn.');
    else if (!robot) notes.push('The 3D model is not loaded: the arm is not drawn.');
    if (m.k1 && !this.distorts) notes.push('The arm is drawn without the distortion k1.');
    if (Math.abs(r.aspect - m.width / m.height) > 0.01 * r.aspect) notes.push(`The image is not ${m.width}:${m.height} like the camera model: the overlay is not correct.`);
    const item = (color: string, text: string) => `<span><i style="background:${color}"></i>${text}</span>`;
    const html =
      item(OUTLINE, 'Robot model outline, measured pose') +
      item(ROBOT, 'Robot model edges (creases)') +
      item(SCREW, 'Base screws A–F') +
      (m.marks?.length ? item(MARK, 'Marks from the calibration') : '') +
      `<span class="overlay-date">Camera model: ${escapeHtml(m.updated ?? 'no date')}${m.k1 ? `, k1 = ${m.k1}` : ''}</span>` +
      notes.map((n) => `<span class="overlay-warn">${escapeHtml(n)}</span>`).join('');
    if (html !== this.legendHtml) this.legend.innerHTML = this.legendHtml = html;
  }
}

const escapeHtml = (t: string) => t.replace(/[&<>"]/g, (ch) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[ch]!);
