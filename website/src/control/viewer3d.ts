// 3D view of the arm from the URDF (three.js + urdf-loader). The meshes are the URDF's meshes,
// converted to compressed GLB by tools/web_meshes.py. A see-through "ghost" shows the goal pose.
// With setGripper(true) the view swaps to the URDF with the adaptive gripper (<name>_gripper.urdf).
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import URDFLoader, { type URDFRobot } from 'urdf-loader';
import { LIMIT_MIN, LIMIT_MAX } from './robot_params';

const JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'];
/** The actuated finger joint. The other five finger joints follow it (URDF `mimic`, which urdf-loader applies).
 *  J7 (the gripper servo, 7th entry of a pose) sets it linearly: J7's limits (its end stops) map to the
 *  joint's URDF limits (closed = lower, open = upper). Not calibrated: the URDF angle is not the servo angle. */
const GRIPPER_JOINT = 'gripper_controller';
const J7 = 6;

/**
 * Objects near the robot, for the lab (served by the lab service as /lab/scene.json). Base frame,
 * millimetres, z up. `yaw` turns the object about z (degrees).
 */
export type SceneObject = { name: string; shape: 'box' | 'ellipsoid'; center: number[]; size: number[]; yaw?: number; color?: string };
export type Scene = { table_z?: number; objects: SceneObject[] };
/** This arm's joint calibration (calibration.json, served by the lab service as /lab/calibration.json). */
export type Calibration = { encoder_correction: { sin: number[]; cos: number[] }[]; zero_offset: number[] };

/** The solid arm (measured pose) and the see-through ghost (goal pose), from one URDF. */
type Arm = { robot: URDFRobot; ghost: URDFRobot };

export class ArmView {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(40, 1, 0.01, 10);
  private controls: OrbitControls;
  private robot: URDFRobot | null = null;
  private ghost: URDFRobot | null = null;
  private needsRender = true;
  // The state, kept so that a swap between the URDFs keeps the pose and the goal (with J7).
  private plainArm: Arm | null = null;
  private gripperArm: Promise<Arm> | null = null;
  private wantGripper = false;
  private q: number[] | null = null;
  private calibration: Calibration | null = null;
  private goal: number[] | null = null;
  private grid: THREE.GridHelper;
  private objects = new THREE.Group();
  /** Goes up at each change of the measured arm (its pose with J7, or a swap of the URDF). The camera overlay redraws then. */
  revision = 0;

  /** `interactive: false` shows the arm only (no drag or zoom), so the page scrolls over it (the home page). */
  constructor(private el: HTMLElement, private urdfUrl: string, { interactive = true } = {}) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    el.append(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enabled = interactive;
    this.resetView();
    this.controls.enableDamping = true;
    this.controls.addEventListener('change', () => (this.needsRender = true));

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 2.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(1, 2, 1.5);
    this.scene.add(sun);
    const grid = new THREE.GridHelper(0.6, 12, 0x888888, 0x888888);
    (grid.material as THREE.Material).opacity = 0.25;
    (grid.material as THREE.Material).transparent = true;
    this.grid = grid;
    this.scene.add(grid);
    this.objects.rotation.x = -Math.PI / 2; // base frame (z up) as for the robot
    this.scene.add(this.objects);

    this.loadArm(urdfUrl)
      .then((arm) => {
        this.plainArm = arm;
        if (!this.wantGripper) this.show(arm);
        el.dataset.loaded = 'true';
      })
      .catch((err) => {
        el.dataset.loaded = 'error';
        el.title = String(err);
      });

    new ResizeObserver(() => this.resize()).observe(el);
    this.resize();
    const loop = () => {
      requestAnimationFrame(loop);
      this.controls.update();
      if (this.needsRender) {
        this.needsRender = false;
        this.renderer.render(this.scene, this.camera);
      }
    };
    loop();
  }

  private loadArm(url: string): Promise<Arm> {
    const load = (ghost: boolean) =>
      new Promise<URDFRobot>((resolve, reject) => {
        // urdf-loader tracks only the URDF file in its manager, not meshes from loadMeshCb. With the
        // GLTF loader on the same manager, onLoad waits for every mesh (before, the robot could be
        // drawn with some meshes missing until the next redraw).
        const manager = new THREE.LoadingManager();
        const gltf = new GLTFLoader(manager).setMeshoptDecoder(MeshoptDecoder);
        const loader = new URDFLoader(manager);
        loader.loadMeshCb = (path, _manager, _material, done) =>
          gltf.load(
            path,
            (g) => {
              if (ghost) g.scene.traverse((o) => {
                const mesh = o as THREE.Mesh;
                if (mesh.isMesh) mesh.material = new THREE.MeshStandardMaterial({ color: 0x3987e5, transparent: true, opacity: 0.28, depthWrite: false });
              });
              done(g.scene);
              this.needsRender = true; // draw each mesh as it arrives
            },
            undefined,
            (err) => done(null as unknown as THREE.Object3D, err as Error),
          );
        let robot: URDFRobot;
        manager.onLoad = () => resolve(robot);
        manager.onError = (u) => reject(new Error(`could not load ${u}`));
        loader.load(url, (r) => (robot = r));
      });

    return Promise.all([load(false), load(true)]).then(([robot, ghost]) => {
      // A joint axis that does not parse (for example a leading space in <axis xyz=" 0 0 1">,
      // which urdf-loader does not trim) makes the joint's transform NaN at its first nonzero
      // angle, and every link after it disappears. Fail loudly instead.
      for (const [name, j] of Object.entries(robot.joints)) {
        const a = (j as unknown as { axis?: THREE.Vector3 }).axis;
        if (a && ![a.x, a.y, a.z].every(Number.isFinite)) throw new Error(`joint ${name}: invalid axis`);
      }
      for (const r of [robot, ghost]) r.rotation.x = -Math.PI / 2; // URDF is z-up; three.js is y-up
      return { robot, ghost };
    });
  }

  /** Put this arm in the scene in place of the present one, with the present pose and goal. */
  private show(arm: Arm) {
    if (this.robot === arm.robot) return;
    if (this.robot) this.scene.remove(this.robot);
    if (this.ghost) this.scene.remove(this.ghost);
    this.robot = arm.robot;
    this.ghost = arm.ghost;
    this.scene.add(arm.robot, arm.ghost);
    if (this.q) this.apply(this.robot, this.q);
    this.ghost.visible = !!this.goal;
    if (this.goal) this.apply(this.ghost, this.goal);
    this.el.dataset.gripper = GRIPPER_JOINT in arm.robot.joints ? 'shown' : 'hidden';
    this.needsRender = true;
    this.revision++;
  }

  /** The solid arm (measured pose), or null before it is loaded. Its root frame is the robot base frame (metres, z up). */
  get measured(): URDFRobot | null {
    return this.robot;
  }

  /** The default camera: front-right, a little above, the whole arm in view. */
  resetView() {
    this.camera.position.set(0.5, 0.38, 0.5);
    this.controls.target.set(0, 0.2, 0);
    this.controls.update();
    this.needsRender = true;
  }

  private resize() {
    const w = this.el.clientWidth || 300;
    const h = this.el.clientHeight || 260;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.needsRender = true;
  }

  private apply(r: URDFRobot | null, qDeg: number[]) {
    if (!r) return;
    JOINTS.forEach((name, j) => r.setJointValue(name, THREE.MathUtils.degToRad(this.trueAngle(j, qDeg[j]))));
    const g = r.joints[GRIPPER_JOINT];
    if (g && qDeg.length > J7) {
      const f = Math.max(0, Math.min(1, (qDeg[J7] - LIMIT_MIN[J7]) / (LIMIT_MAX[J7] - LIMIT_MIN[J7])));
      const { lower, upper } = g.limit as { lower: number; upper: number };
      g.setJointValue(lower + f * (upper - lower));
    }
    this.needsRender = true;
  }

  /**
   * This arm's joint calibration (the lab service's /lab/calibration.json), or null: the view then draws the
   * pose that the robot model gives for the calibrated angles. Only the drawing changes; the angles that the
   * page shows and sends are the ATOM's.
   */
  setCalibration(c: Calibration | null) {
    this.calibration = c;
    if (this.q) this.apply(this.robot, this.q);
    if (this.goal) this.apply(this.ghost, this.goal);
    this.needsRender = true;
    this.revision++;
  }

  /** The true angle of joint j (deg) for an ATOM angle: true = q − Σ sin[k]·sin(k·q) − Σ cos[k]·(cos(k·q) − 1) + zero offset. */
  private trueAngle(j: number, q: number) {
    const c = this.calibration;
    if (!c || j >= c.zero_offset.length) return q;
    const e = c.encoder_correction[j];
    let err = 0;
    const r = THREE.MathUtils.degToRad(q);
    e.sin.forEach((a, k) => (err += a * Math.sin((k + 1) * r)));
    e.cos.forEach((a, k) => (err += a * (Math.cos((k + 1) * r) - 1)));
    return q - err + c.zero_offset[j];
  }

  /** The measured pose, in degrees: J1-J6, and J7 (the gripper) when it is there. */
  setPose(qDeg: number[]) {
    this.q = [...qDeg];
    this.apply(this.robot, qDeg);
    this.revision++;
  }

  /** The goal pose (see-through, J1-J6 and J7 as in setPose), or null to hide it. */
  setGoal(qDeg: number[] | null) {
    this.goal = qDeg ? [...qDeg] : null;
    if (!this.ghost) return;
    this.ghost.visible = !!qDeg;
    if (qDeg) this.apply(this.ghost, qDeg);
    this.needsRender = true;
  }

  /**
   * Show the arm with the adaptive gripper (true) or without it (false). The first call with true
   * loads `<name>_gripper.urdf` (next to the arm's URDF); the view keeps the plain arm until it is loaded.
   * `el.dataset.gripper` is then 'shown', 'hidden', 'loading' or 'error'.
   */
  setGripper(present: boolean) {
    this.wantGripper = present;
    if (!present) {
      if (this.plainArm) this.show(this.plainArm);
      return;
    }
    if (!this.gripperArm) {
      this.el.dataset.gripper = 'loading';
      this.gripperArm = this.loadArm(this.urdfUrl.replace(/\.urdf$/, '_gripper.urdf'));
    }
    this.gripperArm
      .then((arm) => {
        if (this.wantGripper) this.show(arm);
      })
      .catch((err) => {
        this.gripperArm = null; // try again at the next call
        this.el.dataset.gripper = 'error';
        this.el.title = String(err);
      });
  }

  /** Draw the objects of a lab scene (in place of the previous ones), and the grid at the table height. */
  setScene(scene: Scene) {
    for (const o of [...this.objects.children]) {
      const m = o as THREE.Mesh;
      m.geometry.dispose();
      (m.material as THREE.Material).dispose();
      this.objects.remove(o);
    }
    this.grid.position.y = (scene.table_z ?? 0) / 1000;
    for (const o of scene.objects) {
      const [sx, sy, sz] = o.size.map((v) => v / 1000);
      const geometry = o.shape === 'box' ? new THREE.BoxGeometry(sx, sy, sz) : new THREE.SphereGeometry(0.5, 24, 16).scale(sx, sy, sz);
      const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: o.color ?? '#999999', roughness: 0.9 }));
      mesh.position.set(o.center[0] / 1000, o.center[1] / 1000, o.center[2] / 1000);
      mesh.rotation.z = THREE.MathUtils.degToRad(o.yaw ?? 0);
      mesh.name = o.name;
      this.objects.add(mesh);
    }
    this.needsRender = true;
  }
}
