// 3D view of the arm from the URDF (three.js + urdf-loader). The meshes are the URDF's meshes,
// converted to compressed GLB by tools/web_meshes.py. A see-through "ghost" shows the goal pose.
// With setGripper(true) the view swaps to the URDF with the adaptive gripper (<name>_gripper.urdf).
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import URDFLoader, { type URDFRobot } from 'urdf-loader';

const JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'];
/** The actuated finger joint. The other five finger joints follow it (URDF `mimic`, which urdf-loader applies). */
const GRIPPER_JOINT = 'gripper_controller';

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
  // The state, kept so that a swap between the URDFs keeps the pose, the goal and the opening.
  private plainArm: Arm | null = null;
  private gripperArm: Promise<Arm> | null = null;
  private wantGripper = false;
  private q: number[] | null = null;
  private goal: number[] | null = null;
  private opening = 0;
  private goalOpening: number | null = null;

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
    this.scene.add(grid);

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

  /** Put this arm in the scene in place of the present one, with the present pose, goal and opening. */
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
    this.applyOpening();
    this.el.dataset.gripper = GRIPPER_JOINT in arm.robot.joints ? 'shown' : 'hidden';
    this.needsRender = true;
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
    JOINTS.forEach((name, j) => r.setJointValue(name, THREE.MathUtils.degToRad(qDeg[j])));
    this.needsRender = true;
  }

  /** The measured pose, in degrees. */
  setPose(qDeg: number[]) {
    this.q = [...qDeg];
    this.apply(this.robot, qDeg);
  }

  /** The goal pose (see-through), or null to hide it. */
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

  /**
   * The measured gripper opening: 0 closed, 1 fully open (clamped). It sets the actuated finger joint
   * linearly between its URDF limits (closed = lower, open = upper). Not calibrated yet: the fraction
   * is not a pad distance, and it is not mapped to the servo position.
   */
  setGripperOpening(fraction: number) {
    this.opening = Math.max(0, Math.min(1, fraction));
    this.applyOpening();
  }

  /** The goal opening of the ghost (0 closed, 1 open), or null to show the measured opening. */
  setGoalGripperOpening(fraction: number | null) {
    this.goalOpening = fraction == null ? null : Math.max(0, Math.min(1, fraction));
    this.applyOpening();
  }

  private applyOpening() {
    const set = (r: URDFRobot | null, f: number) => {
      const j = r?.joints[GRIPPER_JOINT];
      if (!j) return;
      const { lower, upper } = j.limit as { lower: number; upper: number };
      j.setJointValue(lower + f * (upper - lower));
    };
    set(this.robot, this.opening);
    set(this.ghost, this.goalOpening ?? this.opening);
    this.needsRender = true;
  }
}
