The default PID tuning parameters for the myCobot 280 (for Arduino) robotic arm are as follows:

| Parameter |         Meaning         | J1–J3 |  J4   | J5–J6 |
| :-------: | :---------------------: | :---: | :---: | :---: |
|    21     |  P (proportional gain)  |  32   |  25   |  25   |
|    22     |   D (derivative gain)   |   8   |  25   |  25   |
|    23     |    I (integral gain)    |   0   |   1   |   1   |
|    24     | Minimum starting torque |   0   |   0   |   0   |
|    26     |   CW insensitive zone   |   3   |   3   |   3   |
|    27     |  CCW insensitive zone   |   3   |   3   |   3   |

- The PID gains are used to control the position of the robot joints.
  - The proportional gain (P) is the most important parameter and is used to control the stiffness of the joints.
  - The derivative gain (D) is used to control the damping of the joints.
  - The integral gain (I) is used to control the steady-state error of the joints.
- The minimum starting torque is the minimum torque required to move the joints.
- The CW (clockwise) and CCW (counter-clockwise) insensitive zones are used to prevent the robot from moving when the joints are close to their target positions.
