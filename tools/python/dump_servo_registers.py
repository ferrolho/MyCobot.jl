"""
Dump registers 0-70 of servos 1-6, one byte at a time through the ATOM
(GET_SERVO_DATA 0x53). Read-only. Output format matches docs/servo-registers.md.
"""

from mycobot_bus import SERVO_IDS, atom_read_servo_register, open_port

NAMES = {
    0: "firmware major", 1: "firmware minor", 3: "model (L)", 4: "model (H)", 5: "ID",
    6: "baud rate (0 = 1M)", 7: "return delay", 8: "response level",
    9: "min angle (L)", 10: "min angle (H)", 11: "max angle (L)", 12: "max angle (H)",
    13: "max temperature", 14: "max voltage", 15: "min voltage",
    16: "max torque (L)", 17: "max torque (H)", 18: "phase", 19: "unloading condition",
    20: "LED alarm", 21: "P", 22: "D", 23: "I", 24: "min startup force (L)",
    25: "min startup force (H)", 26: "CW dead zone", 27: "CCW dead zone",
    28: "protection current (L)", 29: "protection current (H)", 30: "angular resolution",
    31: "offset (L)", 32: "offset (H)", 33: "MODE", 34: "protective torque",
    35: "protection time", 36: "overload torque", 37: "speed loop P",
    38: "overcurrent protection time", 39: "speed loop I", 40: "torque enable",
    41: "acceleration", 42: "goal position (L)", 43: "goal position (H)",
    44: "goal time / PWM (L)", 45: "goal time / PWM (H)", 46: "goal speed (L)",
    47: "goal speed (H)", 48: "torque limit (L)", 49: "torque limit (H)", 55: "EEPROM lock",
    56: "present position (L)", 57: "present position (H)", 58: "present speed (L)",
    59: "present speed (H)", 60: "present load (L)", 61: "present load (H)",
    62: "present voltage", 63: "present temperature", 64: "async write flag",
    65: "status", 66: "moving", 67: "? position-like (L)", 68: "? position-like (H)",
    69: "present current (L)", 70: "present current (H)",
}

if __name__ == "__main__":
    sp = open_port(timeout=0.05)
    table = {sid: [atom_read_servo_register(sp, sid, a) for a in range(71)] for sid in SERVO_IDS}
    print("| Addr | Name | " + " | ".join(f"J{s}" for s in SERVO_IDS) + " |")
    print("| ---: | --- | " + " | ".join("---:" for _ in SERVO_IDS) + " |")
    for a in range(71):
        print(f"| {a} | {NAMES.get(a, '')} | " + " | ".join(str(table[s][a]) for s in SERVO_IDS) + " |")
    sp.close()
