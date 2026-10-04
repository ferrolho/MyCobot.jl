---
title: Stock protocol commands
description: Every command of Elephant's stock ATOM protocol for the 280 for Arduino, cross-checked between the English and Chinese documentation and pymycobot.
---

This page lists the commands of the stock ATOM firmware (the `FE FE` protocol). It
compares three sources:

| Source | Version |
| --- | --- |
| Elephant's GitBook, English: [Communication protocol](https://docs.elephantrobotics.com/docs/mycobot_280_ar_en/3-FunctionsAndApplications/6.developmentGuide/CommunicationProtocolPackage/18-communication.html) | Read on 2026-10-04 |
| Elephant's GitBook, Chinese: [通信协议](https://docs.elephantrobotics.com/docs/mycobot_280_ar_cn/3-FunctionsAndApplications/6.developmentGuide/CommunicationProtocolPackage/18-communication.html) | Read on 2026-10-04 |
| pymycobot source (`pymycobot/common.py`, `generate.py`, `mycobot280.py`) | 3.9.6 in this workspace |

For what the stock firmware does on the servo bus, see
[Stock ATOM protocol](/mycobot-280-lab/comms/stock-atom/).

## How to check again

The script downloads both GitBook pages and compares them table by table. It
compares every number and hex value, checks each frame's length byte, and finds
the pymycobot name of each command code:

```bash
python3 tools/python/gitbook_protocol_check.py             # report
python3 tools/python/gitbook_protocol_check.py --markdown  # also the command table below
```

It exits with 1 if the two languages disagree. Run it again when Elephant
updates the documentation.

## Result (2026-10-04)

- The English and Chinese pages have the same 90 tables and 773 rows. **All
  numbers, hex values and command codes are the same** in both languages.
- The length byte of each of the 85 frames is correct (number of data bytes + 2).
- All 62 command codes are in pymycobot.

The two languages are the same, so most problems are in both. The script finds
differences in numbers. The text differences below were found by reading the two
languages side by side.

## Problems in Elephant's documentation

| Where | Problem | Correct value | Evidence |
| --- | --- | --- | --- |
| Servo registers 22 and 23 (`0x52`/`0x53`) | Named "Position loop **I**" (22) and "Position loop **D**" (23). The descriptions in the same rows say the opposite: 22 is the differential coefficient (微分), 23 the integral coefficient (积分). | **22 = D, 23 = I** | The descriptions, Feetech's STS register table (via [LeRobot](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py)), and our dump: J1–J2 read 32/8/0, so 22 = 8 is D. |
| Servo register 24 | "Minimum starting force, 0–1000". | Registers 24–25, two bytes. | `0x52` and `0x53` carry one data byte, so they cannot write or read values above 255. |
| `0x53` reply | One data byte (length byte `0x03`). | One byte. | pymycobot has an optional `mode` byte for two-byte reads. On this arm the two-byte mode gave wrong values. |
| `0xA1` title | English: "Read base IO **output**". Chinese: "读取底座IO**输入**" (read base IO **input**). | Input | pymycobot: `GET_BASIC_INPUT`. |
| `0x60` data | "`00X00/00X01`" (both languages). | `0x00` / `0x01` | Typing error. |
| `0x31` title | "jod-absolute control" (both languages). | jog | Typing error. |
| `0x32` data | "di" is described as "joint servo direction" in a coordinate jog. | Direction along the axis | Wrong description. |
| Cartesian limits | Maximum speed of rx, ry, rz: "40°". | Probably 40°/s | Unit is missing. |
| Page note | "To use the protocol, burn Transponder in the base and the latest atomMain in the ATOM." | — | Not checked. This arm answers on the base pins 13/14 without it (the base passes the bus through). |

The joint limits on the protocol page (J1 ±168°, J2 ±135°, J3 ±150°, J4 ±145°,
J5 ±165°, J6 ±180°; 150°/s, 200°/s²) agree with the URDF. Elephant's product page
gives other limits for AtomMain ≥ 7.3 (J2 ±140°, J4 ±150°, J5 −155° to +160°). See
[joint limits](/mycobot-280-lab/system/robot/#joint-limits).

## pymycobot

- **One code, two names.** Some codes have a second name in pymycobot for other
  robots. For the 280 for Arduino, use the meaning in the table below. Examples:
  `0x16` is `SET_FRESH_MODE` here, not `CLEAR_ROBOT_ERROR`; `0x65`/`0x66` are
  gripper commands here, not `SET_MASTER_PIN_STATUS`/`GET_MASTER_PIN_STATUS`.
- **Commands that pymycobot sends but the GitBook does not list.** `MyCobot280`
  uses 32 of them:
  `0x02` SOFTWARE_VERSION, `0x07` GET_ERROR_INFO, `0x08` CLEAR_ERROR_INFO,
  `0x09` GET_ATOM_VERSION, `0x15` READ_NEXT_ERROR, `0x17` GET_FRESH_MODE,
  `0x1D` SET_VISION_MODE, `0x34` JOG_INCREMENT_COORD, `0x3E` SET_ENCODERS_DRAG,
  `0x40` GET_SPEED, `0x64` SET_PWM_OUTPUT, `0x6E` GET_GRIPPER_MODE,
  `0x87`/`0x88` SET/GET_MOVEMENT_TYPE, `0x8D` SOLVE_INV_KINEMATICS,
  `0xB0` SET_SSID_PWD, `0xC0` GET_TOF_DISTANCE, `0xC1` GET_BASIC_VERSION,
  `0xC2`/`0xC3` SET/GET_COMMUNICATE_MODE, `0xD5` GET_ANGLES_COORDS,
  `0xD7` SET_FOUR_PIECES_ZERO, `0xD8` GET_REBOOT_COUNT, `0xE1` GET_SERVO_SPEED,
  `0xE3` GET_SERVO_VOLTAGES, `0xE4` GET_SERVO_STATUS, `0xE5` GET_SERVO_TEMPS,
  `0xF1`–`0xF5` drag-teach recording.
  Only `0x02` was used on this arm (reply `0x48` = v7.2).

:::caution[Unlisted commands]
A command that the firmware does not support can stop the stock ATOM. After
`GET_ROBOT_STATUS` (`0x19`), which is in neither list, the ATOM stopped replying
until a power cycle. Do not send a command to the stock firmware only because
pymycobot has it.
:::

## Frame format

```text
FE FE <LEN> <CMD> <data...> FA        LEN = number of data bytes + 2
```

Multi-byte values are big-endian (high byte first). Angles are degrees × 100,
coordinates are 0.1 mm and 0.01° (see [Stock ATOM protocol](/mycobot-280-lab/comms/stock-atom/)).
`(h,l)` in the table is one 16-bit value, high byte first.

## Commands

"This arm" shows what was used on this arm with the stock firmware v7.2. An empty
cell means not tested.

| Code | pymycobot | Elephant (EN / CN) | Request data | Reply data | This arm |
| --- | --- | --- | --- | --- | --- |
| `0x10` | POWER_ON | Robot power on / 机械臂上电 | — |  |  |
| `0x11` | POWER_OFF | Robotic arm power off and disconnect / 机械臂掉电并断开连接 | — |  |  |
| `0x12` | IS_POWER_ON | Atom status query / Atom状态查询 | — | `0X01/0X00` | Verified |
| `0x13` | RELEASE_ALL_SERVOS | Robotic arm only powers off / 机械臂仅掉电 | — |  |  |
| `0x14` | IS_CONTROLLER_CONNECTED | Robot system detection is normal / 机器人系统检测正常 | — | `0X01/0X00` |  |
| `0x16` | SET_FRESH_MODE / CLEAR_ROBOT_ERROR | Command refresh mode switch (set interpolation/refresh motion mode) / 指令刷新模式开关（设置插补/刷新运动模式） | `0X01/0X00` |  |  |
| `0x1A` | SET_FREE_MODE | Robot free mode (turn off all torque output) / 机器人自由模式(关闭所有扭力输出) | `01/00` |  |  |
| `0x1B` | IS_FREE_MODE | Check whether it is free mode / 检查是否是自由模式 | — | `0X01/0X00` |  |
| `0x20` | GET_ANGLES | Read angle (read movement information) / 读取角度（读取走位信息） | — | `Angle1(h,l)`, `Angle2(h,l)`, `Angle3(h,l)`, `Angle4(h,l)`, `Angle5(h,l)`, `Angle6(h,l)` | Verified |
| `0x21` | SEND_ANGLE | Send a single angle / 发送单独角度 | `joint_no`, `angle(h,l)`, `sp` |  | Acknowledged, but nothing written to the bus |
| `0x22` | SEND_ANGLES | Send all angles / 发送全部角度 | `Angle1(h,l)`, `Angle2(h,l)`, `Angle3(h,l)`, `Angle4(h,l)`, `Angle5(h,l)`, `Angle6(h,l)`, `Sp` |  | Verified |
| `0x23` | GET_COORDS / GET_JOINTS_COORD | Read all coordinates / 读取全部坐标 | — | `x(h,l)`, `y(h,l)`, `z(h,l)`, `rx(h,l)`, `ry(h,l)`, `rz(h,l)` | Verified |
| `0x24` | SEND_COORD | Send individual coordinate parameters / 发送单独坐标参数 | `x/y/z/rx/ry/rz`, `xyz/rxryrz(h,l)`, `Sp` |  |  |
| `0x25` | SEND_COORDS | Send all coordinate parameters / 发送全部坐标参数 | `x(h,l)`, `y(h,l)`, `z(h,l)`, `rx(h,l)`, `ry(h,l)`, `rz(h,l)`, `Sp`, `0X01` |  |  |
| `0x26` | PAUSE | Program pause / 程序暂停 | — |  |  |
| `0x27` | IS_PAUSED | Is the program paused? / 程序是否暂停 | — | `0X01/0X00` |  |
| `0x28` | RESUME | Program resume / 程序恢复 | — |  |  |
| `0x29` | STOP | Program stop / 程序停止 | — |  |  |
| `0x2A` | IS_IN_POSITION | Whether the point is reached / 是否达到点位 | `x_high/Angle1_high`, `x_low/Angle1_low`, `y_high/Angle2_high`, `y_low/Angle2_low`, `z_high/Angle3_high`, `z_low/Angle3_low`, `rx_high/Angle4_high`, `rx_low/Angle4_low`, `ry_high/Angle5_high`, `ry_low/Angle5_low`, `rz_high/Angle6_high`, `rz_low/Angle6_low`, `0X01/0X00` | `0X01/0X00` |  |
| `0x2B` | IS_MOVING | Robotic arm motion detection / 机械臂运动检测 | — | `0X01/0X00` |  |
| `0x30` | JOG_ANGLE | jog-Joint direction movement / jog-关节方向运动 | `Joint`, `direction`, `sp` |  |  |
| `0x31` | JOG_ABSOLUTE | jod-absolute control / jod-绝对控制 | `Joint`, `Angle(h,l)`, `sp` |  |  |
| `0x32` | JOG_COORD | jog-coordinate direction movement / jog-坐标方向运动 | `axis`, `di`, `sp` |  |  |
| `0x33` | JOG_INCREMENT | jog-stepping mode / jog-步进模式 | `Joint`, `Angle(h,l)`, `sp` |  |  |
| `0x3A` | SET_ENCODER | Send potential value / 发送电位值 | `Joint`, `Encoder(h,l)`, `sp` |  |  |
| `0x3B` | GET_ENCODER | Get potential value / 获取电位值 | `joint` | `Encoder_high`, `Encoders_low` |  |
| `0x3C` | SET_ENCODERS | Send the potential values ​​of six servos / 发送六个舵机的电位值 | `encoder_1(h,l)`, `encoder_2(h,l)`, `encoder_3(h,l)`, `encoder_4(h,l)`, `encoder_5(h,l)`, `encoder_6(h,l)`, `Sp` |  |  |
| `0x3D` | GET_ENCODERS | Read the potential values ​​of six servos / 读取六个舵机的电位值 | — | `encoder_1(h,l)`, `encoder_2(h,l)`, `encoder_3(h,l)`, `encoder_4(h,l)`, `encoder_5(h,l)`, `encoder_6(h,l)` |  |
| `0x41` | SET_SPEED | Set speed / 设置速度 | `sp` |  |  |
| `0x4A` | GET_JOINT_MIN_ANGLE | Read the minimum angle of the joint / 读取关节最小角度 | `Joint_number` | `Joint_number`, `Angle(h,l)` |  |
| `0x4B` | GET_JOINT_MAX_ANGLE | Read the maximum angle of the joint / 读取关节最大角度 | `joint_number` | `joint_number`, `Angle(h,l)` |  |
| `0x4C` | SET_JOINT_MIN | Set the minimum angle of the joint / 设置关节最小角度 | `Joint_number`, `Angle(h,l)` |  |  |
| `0x4D` | SET_JOINT_MAX | Set the maximum angle of the joint / 设置关节最大角度 | `Joint_number`, `Angle(h,l)` |  |  |
| `0x50` | IS_SERVO_ENABLE | View connection / 查看连接 | `Joint_number` | `Joint_number`, `0X01/0X00` |  |
| `0x51` | IS_ALL_SERVO_ENABLE | Check if all servos are powered on / 查看舵机是否全部上电 | — | `0X01/0X00` |  |
| `0x52` | SET_SERVO_DATA / SET_SERVO_MOTOR_CONFIG | Set servo parameters of steering gear / 设置舵机伺服参数 | `joint_no`, `data_id`, `data` |  | Verified (1 byte) |
| `0x53` | GET_SERVO_DATA / GET_SERVO_MOTOR_CONFIG | Read servo parameters / 读取伺服参数 | `joint_no`, `data_id` | `data` | Verified (1 byte) |
| `0x54` | SET_SERVO_CALIBRATION | Set the servo zero point / 设置舵机零点 | `joint_number` |  |  |
| `0x55` | JOINT_BRAKE | Brake a single motor / 刹车单个电机 | `joint_number` |  |  |
| `0x56` | RELEASE_SERVO | Power off a single motor / 单个电机掉电 | `Servo_no` |  |  |
| `0x57` | FOCUS_SERVO | Power on a single motor / 单个电机上电 | `Servo_no` |  |  |
| `0x60` | SET_PIN_MODE | Set atom pin mode / 设置atom引脚模式 | `pin_no`, `00X00/00X01` |  |  |
| `0x61` | SET_DIGITAL_OUTPUT / SET_ATOM_PIN_STATUS | Set Atom IO (setDigitalOutput) / 设置Atom IO(setDigitalOutput) | `Pin_no`, `0X00/0X01` |  |  |
| `0x62` | GET_DIGITAL_INPUT / GET_ATOM_PIN_STATUS | Read Atom IO (getDigitalInput) / 读取Atom IO(getDigitalInput) | `pin_no` | `pin_no`, `0X00/0X01` |  |
| `0x65` | GET_GRIPPER_VALUE / SET_MASTER_PIN_STATUS | Read the gripper angle / 读取夹爪角度 | — | `value` |  |
| `0x66` | SET_GRIPPER_STATE / GET_MASTER_PIN_STATUS | Set the gripper mode / 设置夹爪模式 | `0X00/0X01`, `Sp` |  |  |
| `0x67` | SET_GRIPPER_VALUE | Set the gripper angle / 设置夹爪角度 | `value`, `Sp` |  |  |
| `0x68` | SET_GRIPPER_CALIBRATION | Set the gripper to zero point / 夹爪设置零点 | — |  |  |
| `0x69` | IS_GRIPPER_MOVING | Detect whether the gripper is moving / 检测夹爪是否运动 | — | `00/01` |  |
| `0x6A` | SET_COLOR / GET_ATOM_LED_COLOR | Set the color of the RGB light on the atom screen / 设定atom屏幕RGB灯的颜色 | `0X00/0XFF`, `0X00/0XFF`, `0X00/0XFF` |  | Verified |
| `0x81` | SET_TOOL_REFERENCE | Set tool coordinate system / 设置工具坐标系 | `x(h,l)`, `y(h,l)`, `z(h,l)`, `rx(h,l)`, `ry(h,l)`, `rz(h,l)` |  |  |
| `0x82` | GET_TOOL_REFERENCE | Get tool coordinate system / 获取工具坐标系 | — | `x(h,l)`, `y(h,l)`, `z(h,l)`, `rx(h,l)`, `ry(h,l)`, `rz(h,l)` |  |
| `0x83` | SET_WORLD_REFERENCE | Set the world coordinate system / 设置世界坐标系 | `x(h,l)`, `y(h,l)`, `z(h,l)`, `rx(h,l)`, `ry(h,l)`, `rz(h,l)` |  |  |
| `0x84` | GET_WORLD_REFERENCE | Get the world coordinate system / 获取世界坐标系 | — | `x(h,l)`, `y(h,l)`, `z(h,l)`, `rx(h,l)`, `ry(h,l)`, `rz(h,l)` |  |
| `0x85` | SET_REFERENCE_FRAME | Set base coordinate system / 设置基坐标系 | `00/01` |  |  |
| `0x86` | GET_REFERENCE_FRAME | Get the base coordinate system / 获取基坐标系 | — | `00/01` |  |
| `0x89` | SET_END_TYPE | Set end coordinate system / 设置末端坐标系 | `00/01` |  |  |
| `0x8A` | GET_END_TYPE | Get the end coordinate system / 获取末端坐标系 | — | `00/01` |  |
| `0xA0` | SET_BASIC_OUTPUT / SET_AUXILIARY_PIN_STATUS | Set the base IO output / 设置底座IO输出 | `Pin_no`, `0X00/0X01` |  |  |
| `0xA1` | GET_BASIC_INPUT / GET_AUXILIARY_PIN_STATUS | Read base IO output / 读取底座IO输入 | `Pin_no` | `Pin_no`, `0X00/0X01` |  |
| `0xB1` | GET_SSID_PWD / TOOL_SERIAL_RESTORE | Get WiFi account & password / 获取WiFi账号&密码 | — |  |  |
| `0xB2` | TOOL_SERIAL_READY / SET_SERVER_PORT | Set port number / 设置端口号 | `port(h,l)` |  |  |

