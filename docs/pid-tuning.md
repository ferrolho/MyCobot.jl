The default PID tuning parameters for the myCobot 280 (for Arduino) robotic arm are as follows:

|            Parameter (index) | J1–J3 |  J4   | J5–J6 |
| ---------------------------: | :---: | :---: | :---: |
|   LED alarm on/off flag (20) |   0   |   0   |   0   |
|       Proportional gain (21) |  32   |  25   |  25   |
|         Derivative gain (22) |   8   |  25   |  25   |
|           Integral gain (23) |   0   |   1   |   1   |
| Minimum starting torque (24) |   0   |   0   |   0   |
|                              |       |       |       |
|     CW insensitive zone (26) |   3   |   3   |   3   |
|    CCW insensitive zone (27) |   3   |   3   |   3   |

- The LED alarm on/off flag is used to turn the LED alarm on or off. (when? idk)
- The PID gains are used to control the position of the robot joints.
  - The proportional gain (P) is the most important parameter and is used to control the stiffness of the joints.
  - The integral gain (I) is used to control the steady-state error of the joints.
  - The derivative gain (D) is used to control the damping of the joints.
- The minimum starting torque is the minimum torque required to move the joints.
- The CW (clockwise) and CCW (counter-clockwise) insensitive zones are used to prevent the robot from moving when the joints are close to their target positions.

Here are the default PID tuning parameters for the myCobot 280 (for Arduino) robotic arm:
```
                Servo number    J1    J2    J3    J4    J5    J6

              LED alarm (20)    47    47    47    38    38    38
  Proportional gain - P (21)    32    32    10    10    10    10
    Derivative gain - D (22)     8     8     0     0     0     0
      Integral gain - I (23)     0     0     1     1     1     1
Minimum starting torque (24)     0     0     0     0     0     0
                      ? (25)
    CW insensitive zone (26)     3     3     3     3     3     3
   CCW insensitive zone (27)     3     3     3     3     3     3
```

You can print this table using `MyCobot.print_servo_data(sp)` where `sp` is the serial port object.
