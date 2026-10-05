// 3D view of the arm from the URDF (three.js + urdf-loader). The meshes are the URDF's meshes,
// converted to compressed GLB by tools/web_meshes.py. A see-through "ghost" shows the goal pose.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { MeshoptDecoder } from 'three/examples/jsm/libs/meshopt_decoder.module.js';
import URDFLoader, { type URDFRobot } from 'urdf-loader';

const JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6'];

export class ArmView {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(40, 1, 0.01, 10);
  private controls: OrbitControls;
  private robot: URDFRobot | null = null;
  private ghost: URDFRobot | null = null;
  private needsRender = true;

  constructor(private el: HTMLElement, urdfUrl: string) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    el.append(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
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

    const gltf = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    const load = (ghost: boolean) =>
      new Promise<URDFRobot>((resolve, reject) => {
        const manager = new THREE.LoadingManager();
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
            },
            undefined,
            (err) => done(null as unknown as THREE.Object3D, err as Error),
          );
        let robot: URDFRobot;
        manager.onLoad = () => resolve(robot);
        manager.onError = (url) => reject(new Error(`could not load ${url}`));
        loader.load(urdfUrl, (r) => (robot = r));
      });

    Promise.all([load(false), load(true)])
      .then(([robot, ghost]) => {
        // A joint axis that does not parse (for example a leading space in <axis xyz=" 0 0 1">,
        // which urdf-loader does not trim) makes the joint's transform NaN at its first nonzero
        // angle, and every link after it disappears. Fail loudly instead.
        for (const [name, j] of Object.entries(robot.joints)) {
          const a = (j as unknown as { axis?: THREE.Vector3 }).axis;
          if (a && ![a.x, a.y, a.z].every(Number.isFinite)) throw new Error(`joint ${name}: invalid axis`);
        }
        for (const r of [robot, ghost]) {
          r.rotation.x = -Math.PI / 2; // URDF is z-up; three.js is y-up
          this.scene.add(r);
        }
        ghost.visible = false;
        this.robot = robot;
        this.ghost = ghost;
        el.dataset.loaded = 'true';
        this.needsRender = true;
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
    this.apply(this.robot, qDeg);
  }

  /** The goal pose (see-through), or null to hide it. */
  setGoal(qDeg: number[] | null) {
    if (!this.ghost) return;
    this.ghost.visible = !!qDeg;
    if (qDeg) this.apply(this.ghost, qDeg);
    this.needsRender = true;
  }
}
