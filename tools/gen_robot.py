# /// script
# requires-python = ">=3.10"
# dependencies = ["xacro==2.1.1", "pyyaml>=6"]
# ///
"""
Generate the robot description and the parameter tables from one source:
mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf.xacro and
mycobot_description/config/mycobot_280_arduino/{joint_limits,servos,calibration}.yaml.

    uv run tools/gen_robot.py           # write the generated files
    uv run tools/gen_robot.py --check   # exit 1 if a generated file is not up to date

Writes:
    mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf   Julia (RigidBodyDynamics.jl)
    website/public/robot/mycobot_280_arduino.urdf                           Control page (.glb meshes)
    ..._gripper.urdf (both folders)                                         the same, with the adaptive gripper
    firmware/atom_controller/robot_params.h                                 controller firmware
    website/src/control/robot_params.ts                                     Control page
    tools/robot_params.py                                                   simulator (tools/atom_sim.py)
    src/robot_params.jl                                                     Julia package
"""
import math
import os
import sys

import xacro
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = os.path.join(ROOT, "mycobot_description")
XACRO = os.path.join(DESC, "urdf", "mycobot_280_arduino", "mycobot_280_arduino.urdf.xacro")
CONFIG = os.path.join(DESC, "config", "mycobot_280_arduino")
SOURCES = "mycobot_description/config/mycobot_280_arduino/{joint_limits,servos}.yaml"
CALIBRATION = "mycobot_description/config/mycobot_280_arduino/calibration.yaml"


def num(x):
    """A short decimal for code: 165.0 -> 165, 0.82 -> 0.82."""
    r = round(x, 6)
    return str(int(r)) if r == int(r) else repr(r)


def urdf(mesh_uri, mesh_ext, gripper=False):
    doc = xacro.process_file(XACRO, mappings={"mesh_uri": mesh_uri, "mesh_ext": mesh_ext, "gripper": str(gripper).lower()})
    return doc.toprettyxml(indent="  ").replace(ROOT + os.sep, "")   # the same output in every checkout


def params():
    limits = yaml.safe_load(open(os.path.join(CONFIG, "joint_limits.yaml")))["joint_limits"]
    servos = yaml.safe_load(open(os.path.join(CONFIG, "servos.yaml")))
    names = list(servos["joints"])
    if names != list(limits):
        sys.exit("joint_limits.yaml and servos.yaml list different joints")
    s = [servos["joints"][n] for n in names]
    limit_min = [math.degrees(limits[n]["min_position"]) for n in names]
    limit_max = [math.degrees(limits[n]["max_position"]) for n in names]
    multi_turn = [bool(x.get("multi_turn", False)) for x in s]
    for n, lo, hi, mt in zip(names, limit_min, limit_max, multi_turn):
        if not lo < 0 < hi:
            sys.exit(f"{n}: the position limits must include 0")
        if (lo < -180 or hi > 180) and not mt:
            sys.exit(f"{n}: limits past ±180° need multi_turn: true in servos.yaml")
        if hi - lo > 360:
            sys.exit(f"{n}: the limits must span one turn at most (the power-up reading is one turn)")
    vmax = {round(math.degrees(limits[n]["max_velocity"]), 4) for n in names}
    if len(vmax) != 1:
        sys.exit("the firmware has one speed limit for all joints: give every joint the same max_velocity")
    steps_per_deg = servos["steps_per_turn"] / 360
    calib_path = os.path.join(ROOT, CALIBRATION)
    calib = yaml.safe_load(open(calib_path)) if os.path.exists(calib_path) else {}
    corr = (calib or {}).get("encoder_correction") or {}
    if set(corr) - set(names):
        sys.exit(f"calibration.yaml: unknown joints {sorted(set(corr) - set(names))}")
    g = servos["gripper"]
    if g["id"] != len(names) + 1:
        sys.exit("servos.yaml: the gripper (J7) must have the bus ID after the last joint")
    j7 = sorted(g["sign"] * (st - servos["center_step"]) / steps_per_deg for st in (g["closed_step"], g["open_step"]))
    n_terms = max([len(c[k]) for c in corr.values() for k in ("sin", "cos")] + [0])
    pad = lambda xs: [float(x) for x in xs] + [0.0] * (n_terms - len(xs))
    return {
        "encoder_correction": [(pad(corr.get(n, {}).get("sin", [])), pad(corr.get(n, {}).get("cos", []))) for n in names],
        "names": names,
        "limit_min": [round(d, 4) for d in limit_min],
        "limit_max": [round(d, 4) for d in limit_max],
        "multi_turn": multi_turn,
        "vmax_dps": vmax.pop(),
        "amax_dps2": [round(math.degrees(limits[n]["max_acceleration"]), 4) for n in names],
        "ids": [x["id"] for x in s],
        "sign": [x["sign"] for x in s],
        "gains": [x["gains"] for x in s],
        "servo_amax_dps2": [x["max_acceleration_register"] * 100 / steps_per_deg for x in s],
        "wn": [x["wn"] for x in s],
        "zeta": [x["zeta"] for x in s],
        "center": servos["center_step"],
        "gripper": g,
        # J7, the gripper: its limits are the end stops. The tables of the firmware, the Control page and
        # the simulator have 7 joints (J7 last); in Julia JOINT_SIGN, MULTI_TURN and JOINT_LIMITS_DEG (for the
        # ATOM's messages), the other Julia tables are the arm (J1-J6).
        "j7": {"limit_min": round(j7[0], 4), "limit_max": round(j7[1], 4), "sign": g["sign"],
               "amax_dps2": g["max_acceleration"], "gains": g["gains"]},
        "steps_per_turn": servos["steps_per_turn"],
    }


def all7(p, key):
    """A per-joint table of the arm with J7 (the gripper) after J6."""
    return p[key] + [p["j7"][key]]


def header(comment, sources=SOURCES):
    return f"{comment} Generated by tools/gen_robot.py from {sources}. Do not edit.\n"


def c_params(p):
    arr = lambda xs: "{" + ", ".join(num(x) for x in xs) + "}"
    multi_turn = p["multi_turn"] + [False]
    return (header("//") + "#pragma once\n#include <stdint.h>\n\nnamespace robot {\n"
            f"const int N_ARM = {len(p['names'])};      // J1-J{len(p['names'])}\n"
            f"const int N_JOINTS = {len(p['names']) + 1};   // with J7, the adaptive gripper (bus ID 7), when it is found\n"
            f"const float LIMIT_MIN_DEG[N_JOINTS] = {arr(all7(p, 'limit_min'))};   // joint limits (°); J7: the end stops\n"
            f"const float LIMIT_MAX_DEG[N_JOINTS] = {arr(all7(p, 'limit_max'))};\n"
            f"const bool MULTI_TURN[N_JOINTS] = {{{', '.join('true' if m else 'false' for m in multi_turn)}}};   // phase bit 4 and angle limits 0/0 at power-up\n"
            f"const float VMAX_DPS = {num(p['vmax_dps'])};   // speed limit (°/s)\n"
            f"const float AMAX_DPS2[N_JOINTS] = {arr(all7(p, 'amax_dps2'))};   // acceleration limits (°/s²)\n"
            f"const int8_t JOINT_SIGN[N_JOINTS] = {arr(all7(p, 'sign'))};   // angle = sign × (step − {p['center']}) × 360 / {p['steps_per_turn']}\n"
            f"const uint8_t GAINS[N_JOINTS][3] = {{{', '.join(arr(g) for g in all7(p, 'gains'))}}};   // P, D, I (registers 21, 22, 23)\n"
            f"const uint16_t GRIPPER_MODEL = 0x{p['gripper']['model']:04X};   // J7 (registers 3-4)\n"
            f"const uint16_t GRIPPER_TORQUE = {p['gripper']['torque']};   // registers 16, 28 and 48 (0-1000), set when found\n"
            f"const uint8_t GRIPPER_HOT_C = {p['gripper']['hot_c']}, GRIPPER_COOL_C = {p['gripper']['cool_c']};   // derate above, restore below (°C)\n"
            f"const uint16_t GRIPPER_HOT_TORQUE = {p['gripper']['hot_torque']};   // torque limit while derated\n"
            "}  // namespace robot\n")


def ts_params(p):
    arr = lambda xs: "[" + ", ".join(num(x) for x in xs) + "]"
    return (header("//") +
            f"// J1-J{len(p['names'])}, then J7 (the adaptive gripper, when the ATOM finds it).\n"
            f"export const N_ARM = {len(p['names'])};\n"
            f"export const SIGN = {arr(all7(p, 'sign'))} as const; // angle = sign × (step − {p['center']}) × 360 / {p['steps_per_turn']}\n"
            f"export const STEPS_PER_DEG = {p['steps_per_turn']} / 360;\n"
            f"export const LIMIT_MIN = {arr(all7(p, 'limit_min'))} as const; // joint limits (°); J7: the end stops\n"
            f"export const LIMIT_MAX = {arr(all7(p, 'limit_max'))} as const;\n"
            f"export const VMAX = {num(p['vmax_dps'])}; // speed limit (°/s)\n"
            f"export const AMAX = {arr(all7(p, 'amax_dps2'))} as const; // acceleration limits (°/s²)\n")


def py_params(p):
    tup = lambda xs: "(" + ", ".join(num(x) for x in xs) + ")"
    return (header("#") +
            f"# J1-J{len(p['names'])}, then J7 (the adaptive gripper, when the ATOM finds it).\n"
            f"N_ARM = {len(p['names'])}\n"
            f"SIGN = {tup(all7(p, 'sign'))}   # angle = sign × (step − {p['center']}) × 360 / {p['steps_per_turn']}\n"
            f"STEPS_PER_DEG = {p['steps_per_turn']} / 360\n"
            f"LIMIT_MIN = {tup(all7(p, 'limit_min'))}   # joint limits (°); J7: the end stops\n"
            f"LIMIT_MAX = {tup(all7(p, 'limit_max'))}\n"
            f"MULTI_TURN = ({', '.join('True' if m else 'False' for m in p['multi_turn'] + [False])})   # servo reads and moves past one turn\n"
            f"VMAX = {num(p['vmax_dps'])}   # speed limit (°/s)\n"
            f"AMAX = {tup(all7(p, 'amax_dps2'))}   # acceleration limits (°/s²)\n")


def jl_correction(p):
    tup = lambda xs: "(" + ", ".join(num(x) + (".0" if num(x).lstrip("-").isdigit() else "") for x in xs) + ("," if len(xs) == 1 else "") + ")"
    rows = ",\n".join(f"    (sin = {tup(s)}, cos = {tup(c)})" for s, c in p["encoder_correction"])
    return ("\n\"\"\"Encoder correction of each joint for this robot (degrees; from " + CALIBRATION + "):\n"
            "true angle = encoder angle − Σ sin[k]·sin(k·q) − Σ cos[k]·(cos(k·q) − 1). See `encoder_error`.\"\"\"\n"
            f"const ENCODER_CORRECTION = (\n{rows},\n)\n")


def jl_params(p):
    jf = lambda x: (num(x) + ".0") if num(x).lstrip("-").isdigit() else num(x)   # a Julia Float64
    vec = lambda xs: "[" + ", ".join(jf(x) if isinstance(x, float) else str(x) for x in xs) + "]"
    return (header("#", SOURCES.replace("servos}", "servos,calibration}")) + "\n"
            f"\"Bus ID of each joint servo.\"\nconst SERVO_IDS = UInt8.({vec(p['ids'])})\n\n"
            f"\"Joint angle direction against the servo position: angle = sign × (step − {p['center']}) / STEPS_PER_DEG. J1-J6, then J7 (the gripper).\"\n"
            f"const JOINT_SIGN = ({', '.join(f'{s:+d}' for s in all7(p, 'sign'))})\n\n"
            f"const STEPS_PER_DEG = {p['steps_per_turn']} / 360\n\n"
            f"\"Joints whose servo reads and moves past one turn (the firmware sets phase bit 4 at power-up). J1-J6, then J7.\"\n"
            f"const MULTI_TURN = ({', '.join('true' if m else 'false' for m in p['multi_turn'] + [False])})\n\n"
            f"\"Joint limits (°) of J1-J7 as the controller firmware uses them (J7: the gripper's end stops).\"\n"
            f"const JOINT_LIMITS_DEG = ({', '.join('(' + jf(float(lo)) + ', ' + jf(float(hi)) + ')' for lo, hi in zip(all7(p, 'limit_min'), all7(p, 'limit_max')))})\n\n"
            f"\"Acceleration limits of the controller firmware (°/s²), below the servo limits `SERVO_AMAX`.\"\n"
            f"const FIRMWARE_AMAX = {vec([float(x) for x in p['amax_dps2']])}\n\n"
            f"\"Speed limit of the controller firmware (°/s).\"\nconst FIRMWARE_VMAX = {jf(float(p['vmax_dps']))}\n\n"
            f"\"Position-loop gains (P, D, I) that the controller firmware writes at power-up.\"\n"
            f"const GAINS = [{', '.join('(' + ', '.join(str(x) for x in g) + ')' for g in p['gains'])}]\n\n"
            f"\"Natural frequency ωn (rad/s) of each joint servo, from acceleration-limited chirps.\"\n"
            f"const SERVO_WN = {vec([float(x) for x in p['wn']])}\n\n"
            f"\"Damping ratio ζ of each joint servo.\"\nconst SERVO_ZETA = {vec([float(x) for x in p['zeta']])}\n\n"
            f"\"Acceleration limit of each servo (°/s²): factory register 85 × 100 steps/s².\"\n"
            f"const SERVO_AMAX = {vec([float(x) for x in p['servo_amax_dps2']])}\n" + jl_correction(p))


def outputs():
    p = params()
    return {
        "mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino.urdf":
            urdf("package://mycobot_description/urdf/mycobot_280_arduino/", "dae"),
        "website/public/robot/mycobot_280_arduino.urdf": urdf("", "glb"),
        "mycobot_description/urdf/mycobot_280_arduino/mycobot_280_arduino_gripper.urdf":
            urdf("package://mycobot_description/urdf/mycobot_280_arduino/", "dae", gripper=True),
        "website/public/robot/mycobot_280_arduino_gripper.urdf": urdf("", "glb", gripper=True),
        "firmware/atom_controller/robot_params.h": c_params(p),
        "website/src/control/robot_params.ts": ts_params(p),
        "tools/robot_params.py": py_params(p),
        "src/robot_params.jl": jl_params(p),
    }


def main():
    check = "--check" in sys.argv[1:]
    stale = []
    for rel, text in outputs().items():
        path = os.path.join(ROOT, rel)
        old = open(path).read() if os.path.exists(path) else None
        if old == text:
            continue
        if check:
            stale.append(rel)
        else:
            open(path, "w").write(text)
            print(f"wrote {rel}")
    if stale:
        print("not up to date (run: uv run tools/gen_robot.py):\n  " + "\n  ".join(stale))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
