// atom_controller.ino — onboard trajectory player for the myCobot 280 ATOM (ESP32-PICO-D4).
//
// Core 1 (Arduino loop): owns the servo bus. Holds the pose at power-up, answers bus
//   requests from the network task, and plays uploaded plans at a fixed rate (default
//   500 Hz) with a start-pose check and a tracking-error abort.
// Core 0: IMU sampling (500 Hz), UDP commands, telemetry, status log, OTA updates.
//
// Joints (5.0+): J1-J6, and J7 = the adaptive gripper (bus ID 7) while the ATOM finds it. Every joint
// message carries n joints: 6, or 7 with J7 (see "J7" below).
// On the bus the ATOM is quiet while idle, except: a probe for J7 once a second, and reads of the
// joints while someone subscribes or J7 is found. Drive the servos through the FT232 only with care.
//
// Transports (4.2+): UDP (lab tools) and WebSocket ws://<ATOM>/ws, port 80 (browsers; binary frames =
// the same messages; text frames = the status log). mDNS name: mycobot.local.
// Control (4.2+): one client at a time may send PLAN_*, PLAY, PLAY_SIGNAL, REG_WRITE, MOVE_TO, JOG. If
// nobody has control, such a command takes it for its sender; otherwise another client gets status -2.
// Control ends on release, when its WebSocket closes, or 2 s after its last message (not during its run).
// HOLD and STOP work for every client (during a run they stop it).
//   0x0D CONTROL u8 action (0 release, 1 take, 2 take over) -> ACK 0 / -2 another client / -1 robot moving
//   n below: the number of joints in the message, 6 (J1-J6) or 7 (with J7; 5.0+). The message length
//   tells it. With 6, J7 (if found) holds its goal; 7 without a gripper is refused with -7.
//   0x0E MOVE_TO i16 goal[n] (0.01°), u16 duration_ms (0 = shortest) -> ACK, TELEM (UDP only, 4.3.1+), DONE; state 6 moving
//   0x0F JOG u8 frame (0 = joints), i16 velocity[n] (0.1°/s); ACK only if refused; 200 ms deadman; state 7
//   0x10 TRACK i16 goal[n] (0.01°), u16 vmax (0.1°/s, ≤ 90°/s) (4.4+): the joints go to the goal at up to vmax
//        and their acceleration limits and stop on it (the Control page's Live mode); send it again at least
//        every 200 ms (deadman: brake and hold). ACK only if refused (-1 busy, -10-j goal of joint j outside); state 8
//        JOG and TRACK stopped by a following error of J1-J6 (20°; in TRACK 20° + 0.15 s × recent peak speed)
//        -> DONE result 1, joint, error
//   (GRIPPER 0x11, 4.6-4.7, is gone in 5.0: J7 moves with MOVE_TO, JOG and TRACK.)
// Full API: website/src/content/docs/comms/websocket-api.md.
//
// UDP protocol (little-endian). Clients -> ATOM port 5006. Replies go to the sender of each request
// (its IP and UDP port; 4.0 and older replied to the last sender's IP, port 5007). PLAY telemetry and
// DONE go to the client that started the run.
//   0x01 PING                                   -> 0x81 PONG  u16 version, u8 state, u32 plan_samples, u16 plan_rate, u8 imu_ok,
//                                                             u8 gains_ok, u8 minor, u8 patch
//   0x02 STATE                                  -> 0x82 STATE u8 ok, u8 n (5.0+), u16 pos[n], u16 goal[n] (5.0+),
//                                                             u16 spd[n], u16 load[n], i16 acc[3], i16 gyro[3]
//   0x03 HOLD                                   -> 0x83 ACK   u8 0x03, i8 status
//   0x04 PLAN_BEGIN u32 samples, u16 rate_hz [, u8 interp: 0 linear, 1 cubic (Catmull-Rom; 3.1+)]
//                   [, u8 joints: 6 (default) or 7 with J7 (5.0+)]
//                                               -> 0x83 ACK   u8 0x04, i8 status
//   0x05 PLAN_DATA  u32 offset, u16 count, count × {u16 cmd[joints], u16 ref[joints]}   (servo steps; a
//                   6-joint plan: J7 holds its goal)
//                                               -> 0x83 ACK   u8 0x05, i8 status, u32 offset
//   0x06 PLAN_END   u32 crc32c(all samples)     -> 0x83 ACK   u8 0x06, i8 status
//   0x07 PLAY u16 rate_hz, u16 speed_cap, u16 max_err_steps, u16 start_tol_steps
//                                               -> 0x83 ACK   u8 0x07, i8 status (0 = started; -7 a 7-joint plan
//                                                  and no gripper)
//        during play                            -> 0x84 TELEM u32 first_seq, u8 count, count × Sample (17 + 10 n bytes;
//                                                  77 with 6 joints, 4.0+): u32 t_us, u16 cmd[n], u16 ref[n], u16 pos[n],
//                                                  u16 spd[n], u16 load[n], i16 acc[3], i16 gyro[3], u8 ok. n: the joints
//                                                  of the run's command (the plan, the MOVE_TO; 6 for PLAY_SIGNAL), so
//                                                  a 6-joint client gets 77-byte samples (5.0+). The size tells n.
//        at the end                             -> 0x85 DONE  u8 result, u32 cycles, u32 max_period_us,
//                                                             u32 late_cycles, u32 telem_dropped, u8 joint, i16 error_steps
//   0x08 STOP                                   -> 0x83 ACK   u8 0x08, i8 status
//   0x09 REG_READ  u8 id, u8 addr, u8 len       -> 0x86 REG   u8 id, u8 addr, u8 len, i8 status, data[len]
//   0x0A REG_WRITE u8 id, u8 addr, u8 len, data -> 0x87 REGACK u8 id, u8 addr, i8 status, u8 servo_error
//   0x0B PLAY_SIGNAL (3.1+) u16 rate_hz, u16 speed_cap, u16 max_err_steps, u16 start_tol_steps, sig::Params
//        A test signal computed onboard (test_signal.h): move from the current pose to the base pose,
//        signal on one joint, move back. No plan needed.
//                                               -> 0x83 ACK u8 0x0B, i8 status (0 started, -1 busy, -10-n invalid
//                                                  parameters, -20-n bad start pose), then TELEM and DONE as PLAY
//   0x0C SUBSCRIBE u16 rate_hz (1-100, 0 = stop; 4.1+). Renew at least once a second; a subscription
//        ends 2 s after the last one. Up to 4 subscribers. Also while playing.
//                                               -> 0x88 STREAM (5.0+) u32 t_ms, u8 state, u8 ok, u8 control (0 nobody,
//                                                  1 you, 2 another client), u8 n, u16 pos[n], u16 goal[n], u16 spd[n],
//                                                  u16 load[n], u8 temp[n], u8 volt[n], u8 status[n], i16 acc[3], i16 gyro[3]
//                                                  (21 + 11 n bytes; goal: the goal each joint holds or follows)
//        id 1-7, len 1-32, not while playing. Status: 0 ok, -1 busy or bad request, -4 no reply,
//        -5 write not allowed (registers 0-8, 55 and 80+: ID, baud rate, EEPROM lock, factory),
//        -6 the read-back differs. Writes in the EEPROM area last until the next power cycle.
// Status log (text) once a second: UDP broadcast, port 5005.
//
// Power-up: hold the pose, then write the position-loop gains GAINS and set up the multi-turn joints
// (MULTI_TURN, 4.5+), all verified. PONG's gains_ok is 1 when both took.
//
// LED matrix: blue = starting, green = holding, yellow = plan ready, cyan spiral = playing (progress),
//   red = error (press the button to clear and hold again), magenta = OTA update.
//   Blinks white (4.3+): no WiFi network saved; set it up on the Setup page.
// The button only acknowledges (clear error + hold). It is NOT an emergency stop.
//
// WiFi (4.3+): Improv WiFi over the USB serial port (improv.h; the Setup page's browser installer
// sends the network and password). The network is saved in NVS ("wifi" namespace) and used at
// power-up; without one, a lab build uses the network compiled in from wifi_secrets.h.
// Builds (docs: firmware/build-flash): lab = with ~/.config/mycobot/wifi_secrets.h (WiFi fallback, OTA
// with password); public = -DPUBLIC_BUILD (no secrets, no OTA; the Setup page installs and updates it).

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>

#include "bus.h"
#include "imu.h"
#include "test_signal.h"
#include "motion.h"
#include "improv.h"
#include <Preferences.h>
#if !defined(PUBLIC_BUILD) && __has_include("wifi_secrets.h")
#include "wifi_secrets.h"   // lab build: compiled-in WiFi network and OTA password
#endif
#ifdef PUBLIC_BUILD
#define FW_VARIANT "public"
#else
#define FW_VARIANT "lab"
#define HAS_OTA 1           // public builds have no OTA: without a password anyone on the network could flash them
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif
// After WiFi setup, the browser installer offers this page with the robot's address (Improv "next URL").
#ifndef SETUP_NEXT_URL
#define SETUP_NEXT_URL "https://ferrolho.github.io/mycobot-280-lab/control/?atom="
#endif

// Semantic version: MAJOR for protocol changes that break old clients, MINOR for added commands,
// PATCH for fixes. PING reports MAJOR as its u16 version, then MINOR and PATCH (3.1+).
// History: docs (firmware/changelog). FW_GIT is set by the build (git describe).
#define FW_MAJOR 5
#define FW_MINOR 0
#define FW_PATCH 1
#define FW_VERSION FW_MAJOR
#ifndef FW_GIT
#define FW_GIT "unknown"
#endif
#define STR_(x) #x
#define STR(x) STR_(x)
#define FW_VERSION_STR STR(FW_MAJOR) "." STR(FW_MINOR) "." STR(FW_PATCH)
#define LED_PIN    27
#define BTN_PIN    39
#define CMD_PORT   5006
#define REPLY_PORT 5007
#define LOG_PORT   5005

enum State : uint8_t { BOOTING = 0, HOLDING = 1, READY = 2, PLAYING = 3, ERROR_STATE = 4, OTA = 5, MOVING = 6, JOGGING = 7, TRACKING = 8 };
volatile State state = BOOTING;

Adafruit_NeoPixel matrix(25, LED_PIN, NEO_GRB + NEO_KHZ800);
WiFiUDP cmd_udp, out_udp;
// Reply addresses (4.1+): every reply goes to the sender of its request, and PLAY telemetry to the
// client that started the run. port 0 = nobody (e.g. a HOLD from the button).
struct Sample;   // the Arduino builder puts function prototypes above the first function
struct Addr { uint32_t ip = 0; uint16_t port = 0; uint8_t ws = 0xFF; };   // plain data: it goes through a FreeRTOS queue
// ws != 0xFF: a WebSocket client (connection number); otherwise UDP (ip, port); port 0 and no ws: nobody.
inline bool valid(const Addr& a) { return a.ws != 0xFF || a.port != 0; }
inline bool same(const Addr& a, const Addr& b) {
    return valid(a) && a.ws == b.ws && (a.ws != 0xFF || (a.ip == b.ip && a.port == b.port));
}
WebSocketsServer wss(80);   // ws://<ATOM>/ws (4.2+), network task only
Addr cmd_from;   // sender of the packet that the network task is handling
Addr req_from;   // sender of the request handed to the control loop
Addr play_to;    // client of the running PLAY / PLAY_SIGNAL
volatile bool telem_on = true;   // send TELEM to play_to (not for MOVE_TO over WebSocket, 4.3.1+)

// ---- Plan storage ------------------------------------------------------------------------------
// Each sample: cmd[plan_joints], then ref[plan_joints] (servo steps), as PLAN_DATA sends them.
uint16_t* plan = nullptr;
uint8_t plan_joints = N_ARM;   // 6, or 7 with J7 (5.0+)
uint32_t plan_n = 0, plan_received = 0;
inline const uint16_t* plan_cmd(uint32_t i) { return plan + i * 2 * plan_joints; }
inline const uint16_t* plan_ref(uint32_t i) { return plan + i * 2 * plan_joints + plan_joints; }
uint16_t plan_rate = 250;
bool plan_cubic = false;   // Catmull-Rom between samples (PLAN_BEGIN interp = 1)
volatile bool plan_valid = false;

uint32_t crc32c(uint32_t crc, const uint8_t* p, size_t n) {
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0x82F63B78 & (0 - (crc & 1)));
    }
    return ~crc;
}

// ---- Telemetry ring (control loop -> network task) --------------------------------------------
struct Sample {   // one control cycle; TELEM sends the first n joints of each array (pack_sample)
    uint32_t t_us;
    uint16_t cmd[MAX_JOINTS], ref[MAX_JOINTS];   // goal written and reference, as computed onboard (4.0+)
    uint16_t pos[MAX_JOINTS], spd[MAX_JOINTS], load[MAX_JOINTS];
    int16_t acc[3], gyro[3];
    uint8_t ok, n;
};
const int RING = 512;
const int TELEM_MAX = 1472;   // bytes per UDP packet: 18 samples of 6 joints (77 bytes), 16 of 7 (87 bytes)
Sample ring[RING];

inline size_t sample_size(int n) { return 17 + 10 * n; }
size_t pack_sample(const Sample& s, uint8_t* out) {   // the TELEM layout: arrays of n, little-endian
    const int n = s.n;
    size_t o = 0;
    memcpy(out, &s.t_us, 4); o += 4;
    for (const uint16_t* a : {s.cmd, s.ref, s.pos, s.spd, s.load}) { memcpy(out + o, a, 2 * n); o += 2 * n; }
    memcpy(out + o, s.acc, 6); memcpy(out + o + 6, s.gyro, 6); o += 12;
    out[o++] = s.ok;
    return o;
}
volatile uint32_t ring_head = 0, ring_tail = 0, telem_dropped = 0;

// ---- Requests from the network task to the control loop ---------------------------------------
enum Request : uint8_t { REQ_NONE, REQ_STATE, REQ_HOLD, REQ_PLAY, REQ_REG, REQ_SIGNAL, REQ_MOVE, REQ_JOG, REQ_TRACK };
volatile Request request = REQ_NONE;
volatile bool stop_requested = false;
struct PlayParams { uint16_t rate, speed_cap, max_err, start_tol; } play_params;
uint8_t reg_req[4 + 32];   // REG_READ / REG_WRITE request, copied by the network task
sig::Params signal_params;  // PLAY_SIGNAL request

// Joint angle (°) <-> servo position, as MyCobot.angle_to_position (0° = 2048). JOINT_SIGN and
// GAINS: robot_params.h (generated from mycobot_description/config/mycobot_280_arduino/servos.yaml).
using robot::JOINT_SIGN;
// Past 0-4095 only on multi-turn joints (bus.h).
inline long pos_max(int j) { return robot::MULTI_TURN[j] ? 0xFFFF : 4095; }
inline uint16_t deg_to_pos(int j, float deg) {
    long p = lroundf(2048 + JOINT_SIGN[j] * deg * (4096.0f / 360.0f));
    return (uint16_t)(p < 0 ? 0 : (p > pos_max(j) ? pos_max(j) : p));
}
inline float pos_to_deg(int j, uint16_t p) { return JOINT_SIGN[j] * ((int)p - 2048) * (360.0f / 4096.0f); }

// ---- Servo position-loop gains (P, D, I = registers 21, 22, 23), written at power-up ----------
// The servos store 32/8/0 (no integral action). Integral action on J1-J3 halves the tracking
// error on the circle without more end-effector vibration; a higher P raises the vibration
// (2026-10-04, docs: results/servo-dynamics).
using robot::GAINS;
volatile bool gains_ok = false;

bool write_gains() {
    bool ok = true;
    for (int j = 0; j < N_ARM; j++) {
        ok &= reg_write_verified(j + 1, 21, GAINS[j], 3);   // J7: when found
        // Minimum starting force (24) and dead zone (26/27) (5.0.1): with 20 and 1 on J2-J4, slow motions
        // follow the plan instead of moving in steps of about 1° (the factory 0 and 3).
        uint8_t dz[2] = {robot::DEAD_ZONE[j], robot::DEAD_ZONE[j]};
        ok &= reg_write_verified(j + 1, 24, &robot::START_FORCE[j], 1) && reg_write_verified(j + 1, 26, dz, 2);
    }
    return ok;
}

// Multi-turn joints (4.5+, J6 with the gripper cable): phase bit 4 makes the servo read past one
// turn, and angle limits 0/0 let it take goals past one turn (both tested on J6, 2026-10-06). The
// EEPROM lock stays on, so they last until the servo's next power-off: write them at each power-up.
// The servo then counts from its one-turn reading. The limits span one turn at most, so the reading
// tells the turn: an angle past the middle of the gap between the limits is one turn off. Example
// J6 (−225° to +135°, no gap): a reading of +150° is −210°. Within TURN_BAND of the middle of the
// gap the turn is not known (J6 at ±135° by hand while off): turn_unsure, the joint goes limp and
// the ATOM stays in ERROR until a HOLD finds the turn (move the joint by hand first).
const float TURN_BAND = 1.5f;   // ° (JOG, TRACK and MOVE_TO stop 2° inside the limits)
volatile bool turn_unsure = false;

bool setup_multi_turn() {
    turn_unsure = false;
    bool ok = true;
    for (int j = 0; j < N_ARM; j++) {
        if (!robot::MULTI_TURN[j]) continue;
        uint8_t phase, zero[4] = {0, 0, 0, 0}, d[2];
        if (!reg_read(j + 1, REG_PHASE, 1, &phase)) { ok = false; continue; }
        phase |= 0x10;
        if (!reg_write_verified(j + 1, REG_PHASE, &phase, 1) || !reg_write_verified(j + 1, REG_MIN_ANGLE, zero, 4) ||
            !reg_read(j + 1, REG_PRESENT_POSITION, 2, d)) { ok = false; continue; }
        const float lo = robot::LIMIT_MIN_DEG[j], hi = robot::LIMIT_MAX_DEG[j], gap_mid = hi + (360 - (hi - lo)) / 2;
        float a = JOINT_SIGN[j] * (servo_signed(u16le(d)) - 2048) * (360.0f / 4096.0f);
        int turns = 0;
        while (a > gap_mid) { a -= 360; turns--; }
        while (a < gap_mid - 360) { a += 360; turns++; }
        turn_offset[j] = JOINT_SIGN[j] * turns * 4096;
        if (fabsf(a - gap_mid) < TURN_BAND || fabsf(a - (gap_mid - 360)) < TURN_BAND) {
            turn_unsure = true;
            uint8_t off = 0;
            reg_write(j + 1, REG_TORQUE_ENABLE, &off, 1);
        }
    }
    return ok;
}

bool write_allowed(uint8_t addr, uint8_t len) {
    int end = addr + len;   // exclusive
    return addr >= 9 && end <= 80 && !(addr <= 55 && 55 < end);
}

struct __attribute__((packed)) DoneMsg {
    uint8_t type = 0x85, result;
    uint32_t cycles, max_period_us, late_cycles, telem_dropped;
    uint8_t joint;
    int16_t error_steps;
};
volatile bool done_pending = false;
DoneMsg done_msg;
uint32_t play_progress_permille = 0;

// ---- Network helpers -----------------------------------------------------------------------------
// Only the network task uses the UDP sockets. The control loop posts its replies here.
struct OutMsg { Addr to; uint8_t len; uint8_t data[96]; };   // STATE with 7 joints: 71 bytes
QueueHandle_t outbox;

void post(const void* data, size_t n, const Addr& to) {
    OutMsg m; m.to = to; m.len = min<size_t>(n, sizeof(m.data));
    memcpy(m.data, data, m.len);
    xQueueSend(outbox, &m, 0);
}

void send_to(const Addr& to, const void* data, size_t n) {           // network task only
    if (to.ws != 0xFF) { wss.sendBIN(to.ws, (const uint8_t*)data, n); return; }
    if (to.port == 0) return;
    out_udp.beginPacket(IPAddress(to.ip), to.port);
    out_udp.write(reinterpret_cast<const uint8_t*>(data), n);
    out_udp.endPacket();
}

void send_to_host(const void* data, size_t n) { send_to(cmd_from, data, n); }   // reply to the sender

void ack(uint8_t type, int8_t status, uint32_t value = 0) {        // network task
    uint8_t m[7] = {0x83, type, (uint8_t)status};
    memcpy(m + 3, &value, 4);
    send_to_host(m, sizeof(m));
}

void ack_from_control(uint8_t type, int8_t status) {               // control loop
    uint8_t m[7] = {0x83, type, (uint8_t)status, 0, 0, 0, 0};
    post(m, sizeof(m), req_from);
}

// ---- State stream (SUBSCRIBE 0x0C, 4.1+) ----------------------------------------------------------
const int MAX_SUBS = 4;
struct Sub { Addr a; uint16_t rate = 0; uint32_t last_ms = 0, next_ms = 0; };
Sub subs[MAX_SUBS];                                // network task only
volatile uint16_t stream_rate = 0;                 // highest subscribed rate (0 = none)

struct Latest {                                    // written by the control loop, read by the network task
    uint32_t t_ms; uint8_t ok, n;                  // n: joints in use (5.0+)
    uint16_t pos[MAX_JOINTS], goal[MAX_JOINTS], spd[MAX_JOINTS], load[MAX_JOINTS];
    uint8_t temp[MAX_JOINTS], volt[MAX_JOINTS], status[MAX_JOINTS];
} latest;
portMUX_TYPE latest_mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t goal_steps[MAX_JOINTS];                   // the goal of each joint, as last written (control loop)

void publish_state(bool ok, int n, const uint16_t pos[], const uint16_t spd[], const uint16_t load[]) {
    portENTER_CRITICAL(&latest_mux);
    latest.t_ms = millis(); latest.ok = ok; latest.n = n;
    memcpy(latest.pos, pos, 2 * n); memcpy(latest.goal, goal_steps, 2 * n);
    memcpy(latest.spd, spd, 2 * n); memcpy(latest.load, load, 2 * n);
    portEXIT_CRITICAL(&latest_mux);
}

void subscribe(const Addr& a, uint16_t rate) {
    int free_slot = -1;
    for (int i = 0; i < MAX_SUBS; i++) {
        if (subs[i].rate && same(subs[i].a, a)) {
            subs[i].rate = min<uint16_t>(rate, 100); subs[i].last_ms = millis();
            return;
        }
        if (!subs[i].rate && free_slot < 0) free_slot = i;
    }
    if (rate && free_slot >= 0) { subs[free_slot].a = a; subs[free_slot].rate = min<uint16_t>(rate, 100);
                                  subs[free_slot].last_ms = subs[free_slot].next_ms = millis(); }
}

// ---- Control (4.2+): one client at a time may move the robot or write registers ---------------------
Addr ctrl;                 // the client with control (network task only)
uint32_t ctrl_ms = 0;      // last message from it
inline bool busy() { return state == PLAYING || state == MOVING || state == JOGGING || state == TRACKING; }
inline bool is_ctrl(const Addr& a) { return same(a, ctrl); }
void control_tick() {      // control ends 2 s after the holder's last message, but not during its run
    if (valid(ctrl) && !busy() && millis() - ctrl_ms > 2000) ctrl = Addr();
}

// MOVE_TO and JOG requests (network task -> control loop)
float move_goal[MAX_JOINTS]; uint8_t move_n = N_ARM; uint16_t move_dur_ms = 0;
float jog_target[MAX_JOINTS]; volatile uint32_t jog_ms = 0;   // JOG velocities, or TRACK goal (jog_vmax)
uint8_t jog_n = N_ARM;                                          // joints in the last JOG or TRACK
float jog_vmax = 0;                                             // TRACK speed cap (°/s)
portMUX_TYPE jog_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- J7, the adaptive gripper (5.0+): the seventh joint while it is on the bus ----------------------
// Found by its model number (GRIPPER_MODEL) at power-up, then once a second while the robot is idle (it
// can be plugged in later; not during a run). While found, n_joints is 7: J7 is in every read and write
// of the joints, in MOVE_TO, JOG, TRACK and in the STREAM, with the same units (°). After 5 missed reads
// in a row it counts as removed. Two rules are its own, because it grasps: it has no following-error
// check (on an object it stops short of its goal by design), and a hold keeps its goal and speed cap
// (goal_steps), so a grasp stays closed through HOLD, STOP and the end or abort of a run. Torque:
// GRIPPER_TORQUE when found; thermal derating (4.7+): the torque limit drops to GRIPPER_HOT_TORQUE
// above GRIPPER_HOT_C and comes back below GRIPPER_COOL_C.
const int J7 = N_ARM;                     // index of J7 (bus ID 7)
const uint16_t SPEED_CAP = 2000;          // goal speed (steps/s) while the firmware moves the joints
volatile uint8_t n_joints = N_ARM;        // 6, or 7 while J7 is found (control loop writes)
uint8_t j7_misses = 0;
volatile bool j7_derated = false;

bool write_u16_verified(uint8_t id, uint8_t addr, uint16_t v) {
    uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
    bool ok = reg_write_verified(id, addr, d, 2);
    delayMicroseconds(300);   // no request straight after a write (see bus.h), as on the gripper since 4.6
    return ok;
}

// J7 holds goal_steps[J7]: one write from register 41 (acceleration 0, goal position, goal time 0, goal
// speed), as Feetech's WritePosEx. Two writes back to back (goal speed, then goal position) left the
// servo not moving (2026-10-06). Verified by the goal position read back.
bool j7_hold() {
    const uint16_t st = goal_steps[J7];
    const uint8_t g[7] = {0, (uint8_t)(st & 0xFF), (uint8_t)(st >> 8), 0, 0, (uint8_t)(SPEED_CAP & 0xFF), (uint8_t)(SPEED_CAP >> 8)};
    for (int a = 0; a < 5; a++) {
        if (a) write_retries++;
        uint8_t back[2];
        const bool wrote = reg_write(J7 + 1, REG_ACCELERATION, g, 7) >= 0;   // turns the torque on
        delayMicroseconds(300);   // no request straight after a write (see bus.h)
        if (wrote && reg_read(J7 + 1, REG_GOAL_POSITION, 2, back) && u16le(back) == st) return true;
    }
    return false;
}

void j7_reply(bool ok) {                  // a read of J7: replied or not
    if (ok) j7_misses = 0;
    else if (++j7_misses >= 5) { n_joints = N_ARM; j7_derated = false; }   // removed
}

// The gripper on the bus? Then J7: its gains and torque, and it holds where it is.
bool j7_find() {
    uint8_t m[2], d[2];
    if (!reg_read(J7 + 1, 3, 2, m) || u16le(m) != robot::GRIPPER_MODEL || !reg_read(J7 + 1, REG_PRESENT_POSITION, 2, d)) return false;
    reg_write_verified(J7 + 1, 21, robot::GAINS[J7], 3);
    const uint8_t regs[3] = {REG_MAX_TORQUE, REG_PROTECTION_CURRENT, REG_TORQUE_LIMIT};
    for (uint8_t r : regs) write_u16_verified(J7 + 1, r, robot::GRIPPER_TORQUE);
    goal_steps[J7] = u16le(d);
    j7_hold();
    j7_misses = 0; j7_derated = false;
    n_joints = MAX_JOINTS;
    return true;
}

// Idle only: look for the gripper once a second while it is not found.
void j7_probe_tick() {
    static uint32_t next_ms = 0;
    if (n_joints > N_ARM || (int32_t)(millis() - next_ms) < 0) return;
    next_ms = millis() + 1000;
    j7_find();
}

// Voltage, temperature and status (registers 62-65) of one joint per call, each joint about once a
// second (one read, about 0.3 ms): in the idle loop and in every run. J7's temperature drives its
// thermal derating.
void slow_tick() {
    static uint32_t next_ms = 0;
    static uint8_t j = 0;
    uint32_t now = millis();
    if ((int32_t)(now - next_ms) < 0) return;
    const int n = n_joints;
    next_ms = now + 1000 / n;
    if (j >= n) j = 0;
    uint8_t d[4];   // 62 voltage (0.1 V), 63 temperature (°C), 64, 65 status
    const bool ok = reg_read(j + 1, 62, 4, d);
    if (ok) {
        portENTER_CRITICAL(&latest_mux);
        latest.volt[j] = d[0]; latest.temp[j] = d[1]; latest.status[j] = d[3];
        portEXIT_CRITICAL(&latest_mux);
    }
    if (j == J7) {
        j7_reply(ok);
        if (ok && !j7_derated && d[1] >= robot::GRIPPER_HOT_C) { write_u16_verified(J7 + 1, REG_TORQUE_LIMIT, robot::GRIPPER_HOT_TORQUE); j7_derated = true; }
        else if (ok && j7_derated && d[1] <= robot::GRIPPER_COOL_C) { write_u16_verified(J7 + 1, REG_TORQUE_LIMIT, robot::GRIPPER_TORQUE); j7_derated = false; }
    }
    j++;
}

// Read J1..Jn. True when J1-J6 replied. A missed J7 reply counts toward its removal; then n becomes 6.
bool read_joints(uint16_t pos[], uint16_t spd[], uint16_t load[], int& n) {
    const uint8_t mask = read_state(pos, spd, load, n);
    if (n > N_ARM) { j7_reply(mask & (1 << J7)); n = n_joints; }
    return (mask & all_mask(N_ARM)) == all_mask(N_ARM);
}

// Hold: J1-J6 at their present positions (goal speed 0), J7 at its goal (see above).
bool hold() {
    uint16_t held[N_ARM];
    if (!hold_pose(N_ARM, held)) return false;
    memcpy(goal_steps, held, sizeof(held));
    if (n_joints > N_ARM) j7_reply(j7_hold());
    return true;
}

// The goals that a run starts from: the goals the joints hold (5.0). Restarting from the measured
// position let a joint loaded by gravity sag by its position error at each run (about 1° on J4/J5).
// A goal more than START_GOAL_TOL from the position (stale: the arm moved by hand, or after an error)
// starts from the position. J7 always starts from its goal (a grasp stays closed).
const int START_GOAL_TOL = 34;   // steps (3°)
void start_goals(const uint16_t pos[], int n, uint16_t out[]) {
    for (int j = 0; j < n; j++)
        out[j] = (j == J7 || abs((int)goal_steps[j] - (int)pos[j]) <= START_GOAL_TOL) ? goal_steps[j] : pos[j];
}

// HOLD (5.0): torque on at the goals the joints hold (start_goals), not at the measured position, so a
// HOLD does not let the loaded joints sag. Power-up and errors use hold(): where the arm is.
bool hold_goals() {
    uint16_t zero[MAX_JOINTS] = {0}, pos[MAX_JOINTS], spd[MAX_JOINTS], load[MAX_JOINTS], g[MAX_JOINTS];
    if (!sync_write_u16_verified(REG_GOAL_SPEED, zero, N_ARM)) return false;
    if (read_state(pos, spd, load, N_ARM) != all_mask(N_ARM)) return false;
    start_goals(pos, N_ARM, g);
    if (!sync_write_u16_verified(REG_GOAL_POSITION, g, N_ARM)) return false;
    memcpy(goal_steps, g, 2 * N_ARM);
    if (n_joints > N_ARM) j7_reply(j7_hold());
    return true;
}

void serve_stream() {                              // network task
    uint32_t now = millis();
    uint16_t top = 0;
    uint8_t pkt[21 + 11 * MAX_JOINTS]; size_t len = 0;
    for (int i = 0; i < MAX_SUBS; i++) {
        Sub& s = subs[i];
        if (!s.rate) continue;
        if (now - s.last_ms > 2000) { s.rate = 0; continue; }   // no SUBSCRIBE for 2 s
        top = max(top, s.rate);
        if ((int32_t)(now - s.next_ms) < 0) continue;
        s.next_ms += 1000 / s.rate;
        if ((int32_t)(now - s.next_ms) > 0) s.next_ms = now + 1000 / s.rate;   // late: skip ahead
        if (!len) {   // 5.0 layout: per-joint arrays of n
            Latest l;
            portENTER_CRITICAL(&latest_mux); l = latest; portEXIT_CRITICAL(&latest_mux);
            const int n = l.n ? l.n : N_ARM;   // nothing read yet: 6 joints of zeros
            ImuSample im = imu_get();
            pkt[0] = 0x88;
            memcpy(pkt + 1, &now, 4); pkt[5] = state; pkt[6] = l.ok; pkt[8] = n;
            len = 9;
            for (const uint16_t* a : {l.pos, l.goal, l.spd, l.load}) { memcpy(pkt + len, a, 2 * n); len += 2 * n; }
            for (const uint8_t* a : {l.temp, l.volt, l.status}) { memcpy(pkt + len, a, n); len += n; }
            memcpy(pkt + len, im.acc, 6); memcpy(pkt + len + 6, im.gyro, 6); len += 12;
        }
        pkt[7] = !valid(ctrl) ? 0 : (same(s.a, ctrl) ? 1 : 2);   // control: nobody, you, another client
        send_to(s.a, pkt, len);
    }
    stream_rate = top;
}

bool needs_control(uint8_t c) {
    return c == 0x04 || c == 0x05 || c == 0x06 || c == 0x07 || c == 0x0A || c == 0x0B || c == 0x0E || c == 0x0F || c == 0x10;
}

void handle_command(const uint8_t* b, int n) {
    if (n < 1) return;
    if (is_ctrl(cmd_from)) ctrl_ms = millis();
    if (needs_control(b[0])) {
        if (!valid(ctrl)) { ctrl = cmd_from; ctrl_ms = millis(); }   // implicit take: old clients keep working
        else if (!is_ctrl(cmd_from)) {                                 // another client has control
            if (b[0] == 0x0A) { uint8_t m[5] = {0x87, n > 1 ? b[1] : (uint8_t)0, n > 2 ? b[2] : (uint8_t)0, (uint8_t)-2, 0}; send_to_host(m, 5); }
            else ack(b[0], -2);
            return;
        }
    }
    switch (b[0]) {
    case 0x0D: {   // CONTROL u8 action: 0 release, 1 take, 2 take over
        uint8_t action = n > 1 ? b[1] : 1;
        if (action == 0) { if (is_ctrl(cmd_from)) ctrl = Addr(); ack(0x0D, 0); }
        else if (action == 1) {
            if (!valid(ctrl) || is_ctrl(cmd_from)) { ctrl = cmd_from; ctrl_ms = millis(); ack(0x0D, 0); }
            else ack(0x0D, -2);
        } else {
            if (busy() && !is_ctrl(cmd_from)) ack(0x0D, -1);                // not while the robot moves
            else { ctrl = cmd_from; ctrl_ms = millis(); ack(0x0D, 0); }
        }
        break;
    }
    case 0x0E: {   // MOVE_TO i16 goal[nj] (0.01°), u16 duration_ms (0 = shortest within the limits); nj = 6 or 7
        const int nj = (n - 3) / 2;
        if (n != 15 && n != 17) { ack(0x0E, -1); break; }
        if (nj > n_joints) { ack(0x0E, -7); break; }   // J7 given, no gripper found
        if (busy() || request != REQ_NONE) { ack(0x0E, -1); break; }
        float g[MAX_JOINTS] = {0};
        for (int j = 0; j < nj; j++) { int16_t v; memcpy(&v, b + 1 + 2 * j, 2); g[j] = v * 0.01f; }
        int err = motion::move_validate(g, nj);
        if (err) { ack(0x0E, -10 - err); break; }
        memcpy(move_goal, g, sizeof(g)); move_n = nj; memcpy(&move_dur_ms, b + 1 + 2 * nj, 2);
        play_params = {500, 2000, 227, 34};   // 500 Hz, speed cap, abort at 20° tracking error
        stop_requested = false;
        req_from = play_to = cmd_from;
        telem_on = cmd_from.ws == 0xFF;   // UDP: TELEM as before; WebSocket: the STREAM only (see the network task)
        request = REQ_MOVE;
        break;
    }
    case 0x0F: {   // JOG u8 frame (0 = joints), i16 velocity[nj] (0.1°/s); nj = 6 or 7. ACK only if refused.
        const int nj = (n - 2) / 2;
        if ((n != 14 && n != 16) || b[1] != 0) { ack(0x0F, -1); break; }
        if (nj > n_joints) { ack(0x0F, -7); break; }
        float v[MAX_JOINTS] = {0};
        for (int j = 0; j < nj; j++) { int16_t x; memcpy(&x, b + 2 + 2 * j, 2); v[j] = x * 0.1f; }
        if (state == JOGGING) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, v, sizeof(v)); jog_n = nj; jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
        } else if ((state == HOLDING || state == READY) && request == REQ_NONE) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, v, sizeof(v)); jog_n = nj; jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
            stop_requested = false;
            req_from = play_to = cmd_from; telem_on = true;
            request = REQ_JOG;
        } else ack(0x0F, -1);
        break;
    }
    case 0x10: {   // TRACK i16 goal[nj] (0.01°), u16 vmax (0.1°/s); nj = 6 or 7. ACK only if refused.
        const int nj = (n - 3) / 2;
        if (n != 15 && n != 17) { ack(0x10, -1); break; }
        if (nj > n_joints) { ack(0x10, -7); break; }
        float g[MAX_JOINTS] = {0};
        for (int j = 0; j < nj; j++) { int16_t x; memcpy(&x, b + 1 + 2 * j, 2); g[j] = x * 0.01f; }
        uint16_t vm; memcpy(&vm, b + 1 + 2 * nj, 2);
        int err = motion::move_validate(g, nj);
        if (err) { ack(0x10, -10 - err); break; }
        if (state == TRACKING) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, g, sizeof(g)); jog_n = nj; jog_vmax = vm * 0.1f; jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
        } else if ((state == HOLDING || state == READY) && request == REQ_NONE) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, g, sizeof(g)); jog_n = nj; jog_vmax = vm * 0.1f; jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
            stop_requested = false;
            req_from = play_to = cmd_from; telem_on = true;
            request = REQ_TRACK;
        } else ack(0x10, -1);
        break;
    }
    case 0x01: {   // PING
        uint8_t m[14] = {0x81};
        uint16_t v = FW_VERSION; memcpy(m + 1, &v, 2);
        m[3] = state;
        memcpy(m + 4, &plan_n, 4); memcpy(m + 8, &plan_rate, 2);
        m[10] = imu_ok;
        m[11] = gains_ok;
        m[12] = FW_MINOR;
        m[13] = FW_PATCH;
        send_to_host(m, sizeof(m));
        break;
    }
    case 0x02: if (state != PLAYING && request == REQ_NONE) { req_from = cmd_from; request = REQ_STATE; } break;
    case 0x03:   // HOLD: from any client. During a run it stops the run (the robot holds).
        if (busy()) { stop_requested = true; ack(0x03, 0); }
        else { req_from = cmd_from; request = REQ_HOLD; }
        break;
    case 0x04: {   // PLAN_BEGIN
        if (state == PLAYING || request == REQ_PLAY || n < 7) { ack(0x04, -1); break; }
        uint32_t cnt; uint16_t rate; memcpy(&cnt, b + 1, 4); memcpy(&rate, b + 5, 2);
        plan_valid = false;
        free(plan); plan = nullptr; plan_n = plan_received = 0;
        const uint8_t joints = n >= 9 ? b[8] : N_ARM;
        if (cnt < 2 || rate == 0 || (joints != N_ARM && joints != MAX_JOINTS)) { ack(0x04, -2); break; }
        plan = (uint16_t*)malloc(cnt * 4 * joints);
        if (!plan) { ack(0x04, -3); break; }   // not enough memory
        plan_n = cnt; plan_rate = rate; plan_joints = joints;
        plan_cubic = n >= 8 && b[7] == 1;
        ack(0x04, 0);
        break;
    }
    case 0x05: {   // PLAN_DATA
        uint32_t off; uint16_t cnt; memcpy(&off, b + 1, 4); memcpy(&cnt, b + 5, 2);
        if (!plan || off + cnt > plan_n || n != 7 + cnt * 4 * plan_joints) { ack(0x05, -1, off); break; }
        memcpy(plan + off * 2 * plan_joints, b + 7, cnt * 4 * plan_joints);
        plan_received = max(plan_received, off + cnt);
        ack(0x05, 0, off);
        break;
    }
    case 0x06: {   // PLAN_END
        uint32_t crc; memcpy(&crc, b + 1, 4);
        bool ok = plan && plan_received == plan_n &&
                  crc32c(0, (const uint8_t*)plan, plan_n * 4 * plan_joints) == crc;
        plan_valid = ok;
        if (ok && state == HOLDING) state = READY;
        ack(0x06, ok ? 0 : -1);
        break;
    }
    case 0x07: {   // PLAY
        if (!plan_valid || state == PLAYING || n < 9) { ack(0x07, -1); break; }
        memcpy(&play_params, b + 1, 8);
        stop_requested = false;
        req_from = play_to = cmd_from; telem_on = true;
        request = REQ_PLAY;
        break;
    }
    case 0x08: stop_requested = true; ack(0x08, 0); break;   // STOP: from any client
    case 0x0B: {   // PLAY_SIGNAL
        if (state == PLAYING || request != REQ_NONE || n != 1 + 8 + (int)sizeof(sig::Params)) { ack(0x0B, -1); break; }
        sig::Params p; memcpy(&p, b + 9, sizeof(p));
        int err = sig::validate(p);
        if (err) { ack(0x0B, -10 - err); break; }
        memcpy(&play_params, b + 1, 8);
        signal_params = p;
        stop_requested = false;
        req_from = play_to = cmd_from; telem_on = true;
        request = REQ_SIGNAL;
        break;
    }
    case 0x0C:     // SUBSCRIBE u16 rate_hz (0 = stop); renew at least once a second
        if (n >= 3) { uint16_t rate; memcpy(&rate, b + 1, 2); subscribe(cmd_from, rate); }
        break;
    case 0x09:     // REG_READ
    case 0x0A: {   // REG_WRITE
        bool rd = b[0] == 0x09;
        bool bad = state == PLAYING || request != REQ_NONE || n < 4 || b[1] < 1 || b[1] > 7 || b[3] < 1 || b[3] > 32 ||
                   (!rd && n != 4 + b[3]);
        int8_t status = bad ? -1 : (!rd && !write_allowed(b[2], b[3])) ? -5 : 0;
        if (status) {
            uint8_t m[5] = {(uint8_t)(rd ? 0x86 : 0x87), n > 1 ? b[1] : (uint8_t)0, n > 2 ? b[2] : (uint8_t)0, 0, 0};
            if (rd) m[4] = (uint8_t)status; else m[3] = (uint8_t)status;
            send_to_host(m, sizeof(m));
            break;
        }
        memcpy(reg_req, b, rd ? 4 : n);
        req_from = cmd_from;
        request = REQ_REG;
        break;
    }
    }
}

// ---- WiFi and Improv (4.3+, network task) ----------------------------------------------------------
volatile bool wifi_has_network = false;   // a network is saved or compiled in (else the LED blinks white)
improv::Parser improv_rx;
bool improv_provisioning = false;         // WIFI_SETTINGS received: connecting to the new network
uint32_t improv_deadline = 0;
improv::WifiSettings improv_new;

void wifi_join_saved() {
    Preferences prefs;
    char ssid[improv::MAX_SSID + 1] = "", pass[improv::MAX_PASSWORD + 1] = "";
    if (prefs.begin("wifi", true)) {
        prefs.getString("ssid", ssid, sizeof(ssid));
        prefs.getString("pass", pass, sizeof(pass));
        prefs.end();
    }
    switch (improv::wifi_source(ssid, WIFI_SSID)) {
    case improv::SAVED:    WiFi.begin(ssid, pass); wifi_has_network = true; break;
    case improv::COMPILED: WiFi.begin(WIFI_SSID, WIFI_PASSWORD); wifi_has_network = true; break;
    default:               wifi_has_network = false; break;   // wait for Improv
    }
}

// ---- WiFi watchdog (4.6.1, network task) ------------------------------------------------------------
// After a drop (for example a router restart), join the saved network again every WIFI_RETRY_MS until
// it works. On 2026-10-06 the ESP32's own auto-reconnect gave up after a router restart, and the ATOM
// stayed off the network until a power cycle. mDNS starts again after each reconnect.
const uint32_t WIFI_RETRY_MS = 15000;
bool wifi_up = false, mdns_up = false;
uint32_t wifi_retry_ms = 0;
uint32_t wifi_drops = 0;   // shown in the status log

void wifi_watch() {
    if (!wifi_has_network || improv_provisioning) return;
    uint32_t now = millis();
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifi_up) {   // (re)connected: mycobot.local again
            wifi_up = true;
            if (mdns_up) MDNS.end();
            mdns_up = MDNS.begin("mycobot");
            if (mdns_up) MDNS.addService("http", "tcp", 80);
        }
        return;
    }
    if (wifi_up) {   // just lost
        wifi_up = false;
        wifi_drops++;
        wifi_retry_ms = now;
    }
    if (wifi_retry_ms == 0) wifi_retry_ms = now;   // first join after boot: give it the full interval
    if (now - wifi_retry_ms >= WIFI_RETRY_MS) {
        wifi_retry_ms = now;
        WiFi.disconnect();
        wifi_join_saved();
    }
}

void improv_write(const uint8_t* p, size_t n) { if (n) Serial.write(p, n); }

void improv_send_state(uint8_t st) { uint8_t out[improv::MAX_PACKET]; improv_write(out, improv::build_state(st, out)); }
void improv_send_error(uint8_t e) { uint8_t out[improv::MAX_PACKET]; improv_write(out, improv::build_error(e, out)); }

void improv_send_url(uint8_t command) {   // RPC result: the Control page with this robot's address
    String url = String(SETUP_NEXT_URL) + WiFi.localIP().toString();
    const char* strs[] = {url.c_str()};
    uint8_t out[improv::MAX_PACKET];
    improv_write(out, improv::build_result(command, strs, 1, out));
}

void improv_send_current_state() {
    if (improv_provisioning) { improv_send_state(improv::PROVISIONING); return; }
    if (WiFi.status() == WL_CONNECTED) { improv_send_state(improv::PROVISIONED); improv_send_url(improv::GET_STATE); }
    else improv_send_state(improv::READY);
}

bool robot_busy() { return state == PLAYING || state == MOVING || state == JOGGING || state == TRACKING || state == OTA; }

void improv_handle(const uint8_t* d, size_t len) {
    int command = improv::rpc_command(d, len);
    if (command < 0) { improv_send_error(improv::INVALID_RPC); return; }
    switch (command) {
    case improv::GET_STATE:
        improv_send_error(improv::NO_ERROR);
        improv_send_current_state();
        break;
    case improv::GET_INFO: {
        const char* info[] = {"myCobot 280 controller", FW_VERSION_STR, "ESP32", "mycobot"};
        uint8_t out[improv::MAX_PACKET];
        improv_write(out, improv::build_result(improv::GET_INFO, info, 4, out));
        break;
    }
    case improv::GET_NETWORKS: {
        if (robot_busy() || improv_provisioning) { improv_send_error(improv::UNKNOWN); break; }
        int n = WiFi.scanNetworks();   // blocks for about 2-4 s
        uint8_t out[improv::MAX_PACKET];
        for (int i = 0; i < n; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;   // hidden network
            char rssi[8]; snprintf(rssi, sizeof(rssi), "%d", (int)WiFi.RSSI(i));
            const char* strs[] = {ssid.c_str(), rssi, WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "NO" : "YES"};
            improv_write(out, improv::build_result(improv::GET_NETWORKS, strs, 3, out));
        }
        WiFi.scanDelete();
        improv_write(out, improv::build_result(improv::GET_NETWORKS, nullptr, 0, out));   // end of the list
        break;
    }
    case improv::WIFI_SETTINGS:
        if (robot_busy()) { improv_send_error(improv::UNKNOWN); break; }
        if (!improv::parse_wifi_settings(d, len, improv_new)) { improv_send_error(improv::INVALID_RPC); break; }
        improv_send_error(improv::NO_ERROR);
        improv_provisioning = true;
        improv_deadline = millis() + 20000;
        improv_send_state(improv::PROVISIONING);
        WiFi.disconnect();
        WiFi.begin(improv_new.ssid, improv_new.password);
        break;
    default:
        improv_send_error(improv::UNKNOWN_RPC);
        break;
    }
}

void improv_poll() {
    while (Serial.available()) {
        auto r = improv_rx.feed((uint8_t)Serial.read());
        if (r == improv::Parser::BAD_CHECKSUM) improv_send_error(improv::INVALID_RPC);
        else if (r == improv::Parser::PACKET && improv_rx.type == improv::RPC) improv_handle(improv_rx.data, improv_rx.len);
    }
    if (improv_provisioning) {
        if (WiFi.status() == WL_CONNECTED) {
            Preferences prefs;   // save only a network that works
            if (prefs.begin("wifi", false)) {
                prefs.putString("ssid", improv_new.ssid);
                prefs.putString("pass", improv_new.password);
                prefs.end();
            }
            memset(&improv_new, 0, sizeof(improv_new));
            improv_provisioning = false;
            wifi_has_network = true;
            improv_send_state(improv::PROVISIONED);
            improv_send_url(improv::WIFI_SETTINGS);
        } else if ((int32_t)(millis() - improv_deadline) > 0) {
            memset(&improv_new, 0, sizeof(improv_new));
            improv_provisioning = false;
            improv_send_error(improv::UNABLE_TO_CONNECT);
            improv_send_state(improv::READY);
            WiFi.disconnect();
            wifi_join_saved();   // back to the previous network, if any
        }
    }
}

// ---- LED matrix -------------------------------------------------------------------------------------
// Progress (5.0+): pixels light in the colour, one at a time, in a spiral from the centre out to the
// top-left corner (index = row * 5 + column). Each pixel lights halfway through its 1/25 of the run.
const uint8_t SPIRAL[25] = {12, 7, 8, 13, 18, 17, 16, 11, 6, 1, 2, 3, 4, 9, 14, 19, 24, 23, 22, 21, 20, 15, 10, 5, 0};

void show(uint8_t r, uint8_t g, uint8_t b, int progress_permille = -1) {
    if (progress_permille < 0) {
        for (int i = 0; i < 25; i++) matrix.setPixelColor(i, matrix.Color(r, g, b));
    } else {
        int lit = (progress_permille * 25 + 500) / 1000;
        for (int k = 0; k < 25; k++) matrix.setPixelColor(SPIRAL[k], k < lit ? matrix.Color(r, g, b) : 0);
    }
    matrix.show();
}

void update_led() {
    // No WiFi network saved (4.3+): blink white over the state colour, 1 s period.
    if (!wifi_has_network && !improv_provisioning && (millis() / 500) % 2) { show(30, 30, 30); return; }
    switch (state) {
    case BOOTING: show(0, 0, 40); break;
    case HOLDING: show(0, 40, 0); break;
    case READY:   show(40, 30, 0); break;
    case PLAYING: show(0, 30, 30, play_progress_permille); break;
    case ERROR_STATE: show(50, 0, 0); break;
    case OTA:     show(40, 0, 40); break;
    case MOVING:  show(0, 30, 30, play_progress_permille); break;
    case JOGGING: show(20, 20, 40); break;
    case TRACKING: show(20, 20, 40); break;
    }
}

// ---- Network task (core 0) -----------------------------------------------------------------------
void ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {   // network task (wss.loop)
    if (type == WStype_BIN && len > 0) {
        cmd_from = Addr(); cmd_from.ws = num;
        handle_command(payload, (int)len);
    } else if (type == WStype_DISCONNECTED) {
        for (int i = 0; i < MAX_SUBS; i++) if (subs[i].rate && subs[i].a.ws == num) subs[i].rate = 0;
        if (ctrl.ws == num) {
            ctrl = Addr();
            if (state == JOGGING || state == TRACKING) { portENTER_CRITICAL(&jog_mux); jog_ms = 0; portEXIT_CRITICAL(&jog_mux); }   // deadman now
        }
    }
}

void net_task(void*) {
    uint8_t buf[1500];
    uint32_t last_log = 0;
    for (;;) {
#ifdef HAS_OTA
        if (state != PLAYING) ArduinoOTA.handle();
#endif
        improv_poll();

        int n = cmd_udp.parsePacket();
        if (n > 0) {
            cmd_from = Addr();   // reset all fields: a WebSocket request before must not leave its .ws
            cmd_from.ip = (uint32_t)cmd_udp.remoteIP(); cmd_from.port = cmd_udp.remotePort();
            n = cmd_udp.read(buf, sizeof(buf));
            if (n > 0) handle_command(buf, n);
        }

        wifi_watch();   // rejoin after a drop; mycobot.local after each (re)connect
        wss.loop();
        control_tick();
        OutMsg om;
        while (xQueueReceive(outbox, &om, 0) == pdTRUE) send_to(om.to, om.data, om.len);
        serve_stream();

        static uint32_t last_led = 0;
        if (millis() - last_led >= 100) { last_led = millis(); update_led(); }

        // Telemetry: batches of up to 18 samples (16 with 7 joints), all with the same n. Over a WebSocket
        // (TCP) every send waits for the client: 500 Hz telemetry (28 packets/s) blocked this task during
        // MOVE_TO, so the STREAM paused for 0.1-0.5 s and replies came late (2026-10-05). A WebSocket
        // MOVE_TO sends no TELEM.
        if (!telem_on) { __sync_synchronize(); ring_tail = ring_head; }
        while (ring_tail != ring_head) {
            __sync_synchronize();
            const uint32_t avail = ring_head - ring_tail;
            const int n = ring[ring_tail % RING].n;
            const uint32_t most = (TELEM_MAX - 6) / sample_size(n);
            uint8_t cnt = 0;
            while (cnt < avail && cnt < most && ring[(ring_tail + cnt) % RING].n == n) cnt++;
            uint8_t pkt[TELEM_MAX];
            pkt[0] = 0x84;
            uint32_t seq = ring_tail; memcpy(pkt + 1, &seq, 4); pkt[5] = cnt;
            size_t len = 6;
            for (int i = 0; i < cnt; i++) len += pack_sample(ring[(ring_tail + i) % RING], pkt + len);
            __sync_synchronize();
            ring_tail += cnt;
            send_to(play_to, pkt, len);
        }
        if (done_pending && ring_tail == ring_head) {
            send_to(play_to, &done_msg, sizeof(done_msg));
            done_pending = false;
        }

        uint32_t now = millis();
        if (now - last_log >= 1000) {
            last_log = now;
            char line[256];
            snprintf(line, sizeof(line), "atom_controller v%d.%d.%d (%s, %s) ip=%s rssi=%d state=%d plan=%lu@%uHz valid=%d imu=%d write_retries=%lu heap=%lu up=%lus wifi_drops=%lu joints=%u%s%s",
                     FW_MAJOR, FW_MINOR, FW_PATCH, FW_GIT, FW_VARIANT, WiFi.localIP().toString().c_str(), WiFi.RSSI(), state, (unsigned long)plan_n, plan_rate,
                     plan_valid, imu_ok, (unsigned long)write_retries, (unsigned long)ESP.getFreeHeap(), (unsigned long)(now / 1000), (unsigned long)wifi_drops, (unsigned)n_joints, j7_derated ? " j7_derated" : "",
                     turn_unsure ? " TURN UNKNOWN: move J6 away from ±135° by hand, then HOLD" : "");
            wss.broadcastTXT(line);
            if (WiFi.status() == WL_CONNECTED) {
                out_udp.beginPacket(IPAddress(255, 255, 255, 255), LOG_PORT);
                out_udp.write((const uint8_t*)line, strlen(line));
                out_udp.endPacket();
            }
        }
        vTaskDelay(1);
    }
}

// ---- Control (core 1) ------------------------------------------------------------------------------
// The plan's cmd and ref at sample position s (plan_joints joints).
void interp(uint32_t n, float s, uint16_t cmd[], uint16_t ref[]) {
    const int nj = plan_joints;
    if (s <= 0) { memcpy(cmd, plan_cmd(0), 2 * nj); memcpy(ref, plan_ref(0), 2 * nj); return; }
    if (s >= n - 1) { memcpy(cmd, plan_cmd(n - 1), 2 * nj); memcpy(ref, plan_ref(n - 1), 2 * nj); return; }
    uint32_t i = (uint32_t)s;
    float f = s - i;
    if (plan_cubic) {
        // Catmull-Rom through the samples (C1, passes through every sample).
        uint32_t i0 = i > 0 ? i - 1 : 0, i3 = i + 2 < n ? i + 2 : n - 1;
        float f2 = f * f, f3 = f2 * f;
        auto cr = [&](int j, float p0, float p1, float p2, float p3) {
            float v = 0.5f * (2 * p1 + (p2 - p0) * f + (2 * p0 - 5 * p1 + 4 * p2 - p3) * f2 + (3 * p1 - p0 - 3 * p2 + p3) * f3);
            long r = lroundf(v);
            return (uint16_t)(r < 0 ? 0 : (r > pos_max(j) ? pos_max(j) : r));
        };
        for (int j = 0; j < nj; j++) {
            cmd[j] = cr(j, plan_cmd(i0)[j], plan_cmd(i)[j], plan_cmd(i + 1)[j], plan_cmd(i3)[j]);
            ref[j] = cr(j, plan_ref(i0)[j], plan_ref(i)[j], plan_ref(i + 1)[j], plan_ref(i3)[j]);
        }
        return;
    }
    for (int j = 0; j < nj; j++) {
        cmd[j] = (uint16_t)lroundf(plan_cmd(i)[j] + f * ((int)plan_cmd(i + 1)[j] - (int)plan_cmd(i)[j]));
        ref[j] = (uint16_t)lroundf(plan_ref(i)[j] + f * ((int)plan_ref(i + 1)[j] - (int)plan_ref(i)[j]));
    }
}

void finish(uint8_t result, uint32_t cycles, uint32_t max_period, uint32_t late, uint8_t joint, int16_t err) {
    done_msg.result = result; done_msg.cycles = cycles; done_msg.max_period_us = max_period;
    done_msg.late_cycles = late; done_msg.telem_dropped = telem_dropped; done_msg.joint = joint; done_msg.error_steps = err;
    done_pending = true;
}

// Result codes: 0 done, 1 tracking error, 2 stopped, 3 not at the start pose, 4 bus error
// Runs a motion at a fixed rate with telemetry: src 0 = the uploaded plan (PLAY), 1 = the
// PLAY_SIGNAL test signal, 2 = a MOVE_TO minimum-jerk move (4.2+), all computed onboard. With J7
// (5.0+): MOVE_TO and 7-joint plans move it with the other joints; with 6 goals, a 6-joint plan or a
// test signal (J1-J6) it holds its goal. TELEM has the joints of the command (telem_n).
void play(uint8_t src = 0) {
    const PlayParams pp = play_params;
    const bool use_signal = src == 1, use_move = src == 2;
    const uint8_t type = use_signal ? 0x0B : (use_move ? 0x0E : 0x07);
    if (pp.rate < 50 || pp.rate > 800) { ack_from_control(type, -2); return; }
    int n = n_joints;
    const int cmd_n = use_signal ? N_ARM : use_move ? move_n : plan_joints;   // the joints of the command
    if (cmd_n > n) { ack_from_control(type, -7); return; }   // J7 in the command, no gripper found
    const int telem_n = cmd_n;
    uint16_t pos[MAX_JOINTS], spd[MAX_JOINTS], load[MAX_JOINTS], cmd[MAX_JOINTS], ref[MAX_JOINTS], start[MAX_JOINTS];
    if (!read_joints(pos, spd, load, n)) { ack_from_control(type, -4); finish(4, 0, 0, 0, 0, 0); state = ERROR_STATE; return; }
    start_goals(pos, n, start);
    const sig::Params sp = signal_params;
    float start_deg[MAX_JOINTS], q_deg[N_ARM];
    for (int j = 0; j < n; j++) start_deg[j] = pos_to_deg(j, start[j]);
    if (use_signal) {
        int err = sig::validate_start(sp, start_deg);
        if (err) { ack_from_control(type, -20 - err); finish(3, 0, 0, 0, 0, 0); return; }
    } else if (!use_move) {
        for (int j = 0; j < plan_joints; j++) {   // J7: from its goal
            int e = (int)start[j] - (int)plan_ref(0)[j];
            if (abs(e) > pp.start_tol) { ack_from_control(type, -3); finish(3, 0, 0, 0, j + 1, e); return; }
        }
    }
    float goal[MAX_JOINTS], move_T = 0;
    if (use_move) {
        memcpy(goal, move_goal, sizeof(goal));
        if (move_n < n) goal[J7] = start_deg[J7];   // 6 goals: J7 holds
        float Tmin = motion::move_min_duration(start_deg, goal, n);
        move_T = move_dur_ms ? move_dur_ms * 0.001f : Tmin;
        if (move_T < Tmin * 0.999f) { ack_from_control(type, -1); finish(3, 0, 0, 0, 0, 0); return; }
    }
    ack_from_control(type, 0);
    play_progress_permille = 0;
    state = use_move ? MOVING : PLAYING;
    telem_dropped = 0;

    // Enable motion: hold, no acceleration ramp, speed cap (each write verified)
    uint16_t caps[MAX_JOINTS]; for (int j = 0; j < n; j++) caps[j] = pp.speed_cap;
    if (!sync_write_u16_verified(REG_GOAL_POSITION, start, n) || !sync_write_u8_verified(REG_ACCELERATION, 0, n) ||
        !sync_write_u16_verified(REG_GOAL_SPEED, caps, n)) {
        hold();
        finish(4, 0, 0, 0, 0, 0); state = ERROR_STATE; return;
    }

    const uint32_t period = 1000000UL / pp.rate;
    const float motion_s = use_signal ? sig::total_s(sp) : use_move ? move_T : (plan_n - 1) / (float)plan_rate;
    const float duration_s = motion_s + 0.5f;   // plus 0.5 s settling
    uint32_t t0 = micros(), next = t0, cycles = 0, late = 0, max_period = 0, last = t0;
    uint8_t result = 0, bad_joint = 0; int16_t bad_err = 0;
    for (;;) {
        uint32_t now = micros();
        float t = (now - t0) * 1e-6f;
        if (t >= duration_s) break;
        if (stop_requested) { result = 2; break; }

        if (use_signal) {
            sig::eval(sp, start_deg, t, q_deg);
            for (int j = 0; j < N_ARM; j++) cmd[j] = ref[j] = deg_to_pos(j, q_deg[j]);
            if (n > N_ARM) cmd[J7] = ref[J7] = start[J7];
        } else if (use_move) {
            float s_ = motion::minjerk(t / move_T);
            for (int j = 0; j < n; j++) cmd[j] = ref[j] = deg_to_pos(j, start_deg[j] + s_ * (goal[j] - start_deg[j]));
        } else {
            interp(plan_n, t * plan_rate, cmd, ref);
            if (n > plan_joints) cmd[J7] = ref[J7] = start[J7];   // a 6-joint plan: J7 holds
        }
        sync_write_u16(REG_GOAL_POSITION, cmd, n);
        memcpy(goal_steps, cmd, 2 * n);
        bool ok = read_joints(pos, spd, load, n);
        if (stream_rate) publish_state(ok, n, pos, spd, load);
        slow_tick();

        Sample& s = ring[ring_head % RING];
        if (ring_head - ring_tail >= RING) { telem_dropped++; }
        else {
            s.t_us = now - t0; s.n = telem_n;
            memcpy(s.cmd, cmd, sizeof(s.cmd)); memcpy(s.ref, ref, sizeof(s.ref));
            memcpy(s.pos, pos, sizeof(s.pos)); memcpy(s.spd, spd, sizeof(s.spd)); memcpy(s.load, load, sizeof(s.load));
            ImuSample im = imu_get();
            memcpy(s.acc, im.acc, sizeof(s.acc)); memcpy(s.gyro, im.gyro, sizeof(s.gyro));
            s.ok = ok;
            __sync_synchronize();
            ring_head++;
        }

        if (ok) {
            for (int j = 0; j < N_ARM; j++) {   // not J7: on an object it stops short of its goal
                int e = (int)pos[j] - (int)ref[j];
                if (abs(e) > pp.max_err) { result = 1; bad_joint = j + 1; bad_err = e; break; }
            }
            if (result) break;
        }

        cycles++;
        play_progress_permille = t >= motion_s ? 1000 : (uint32_t)(1000 * t / motion_s);   // full while settling
        uint32_t dt = now - last; last = now;
        if (cycles > 1) max_period = max(max_period, dt);
        next += period;
        if ((int32_t)(micros() - next) > 0) { late++; next = micros(); }   // late: don't try to catch up
        while ((int32_t)(micros() - next) < 0) {}
    }

    if (result) hold();                              // abort: stay where we are
    else delay(300);
    uint16_t zero[N_ARM] = {0};
    sync_write_u16_verified(REG_GOAL_SPEED, zero, N_ARM);   // back to "don't move"; J7 keeps its goal
    finish(result, cycles, max_period, late, bad_joint, bad_err);
    state = result == 0 || result == 2 ? (plan_valid ? READY : HOLDING) : ERROR_STATE;
}

void setup() {
    Serial.begin(115200);
    pinMode(BTN_PIN, INPUT);
    matrix.begin();
    matrix.setBrightness(20);
    show(0, 0, 40);

    bus_begin();
    imu_ok = imu_init();

    WiFi.persistent(false);                        // the saved network is in our own NVS namespace
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("mycobot-atom");
    WiFi.setSleep(false);                          // lower, steadier latency
    wifi_join_saved();                             // saved over Improv, else compiled in, else none
#ifdef HAS_OTA
    ArduinoOTA.setHostname("mycobot-atom");
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.onStart([]() { state = OTA; show(40, 0, 40); Bus.end(); });
    ArduinoOTA.begin();
#endif
    cmd_udp.begin(CMD_PORT);
    wss.begin();
    wss.onEvent(ws_event);


    outbox = xQueueCreate(16, sizeof(OutMsg));
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(net_task, "net", 8192, nullptr, 1, nullptr, 0);

    // Power-up: J7 if the gripper is there, then hold the pose (torque on, goal speed 0). Retry until all
    // servos answer.
    delay(300);
    j7_find();
    for (int k = 0; k < 20 && !hold(); k++) delay(100);
    for (int k = 0; k < 5 && !gains_ok; k++) { gains_ok = write_gains() && setup_multi_turn(); if (!gains_ok) delay(100); }
    if (!turn_unsure) hold();   // goals in our steps (turn_offset)
    int n = n_joints;
    uint16_t p[MAX_JOINTS], s[MAX_JOINTS], l[MAX_JOINTS];
    state = read_joints(p, s, l, n) && !turn_unsure ? HOLDING : ERROR_STATE;
}

// JOG (4.2+): the client streams joint velocities; the goals integrate them at 500 Hz within the
// speed, acceleration and joint limits (motion.h). No JOG for 200 ms (deadman), STOP, HOLD or a
// zero velocity: ramp down at the acceleration limit, then hold. The state stream shows the motion.
// TRACK (4.4+, track = true): the client streams a goal pose; the joints go there (motion::track_step)
// and stay there while TRACK keeps coming. Deadman, STOP or HOLD: brake to zero, then hold.
// J7 (5.0+) starts from its goal; with 6 values it holds (JOG: no velocity, TRACK: its last goal).
void jog_run(bool track = false) {
    int n = n_joints;
    uint16_t pos[MAX_JOINTS], spd[MAX_JOINTS], load[MAX_JOINTS], cmd[MAX_JOINTS], start[MAX_JOINTS];
    if (!read_joints(pos, spd, load, n)) { ack_from_control(track ? 0x10 : 0x0F, -4); return; }
    start_goals(pos, n, start);
    uint16_t caps[MAX_JOINTS]; for (int j = 0; j < n; j++) caps[j] = SPEED_CAP;
    if (!sync_write_u16_verified(REG_GOAL_POSITION, start, n) || !sync_write_u8_verified(REG_ACCELERATION, 0, n) ||
        !sync_write_u16_verified(REG_GOAL_SPEED, caps, n)) { hold(); state = ERROR_STATE; return; }
    motion::Jog js = {};
    for (int j = 0; j < n; j++) js.q[j] = pos_to_deg(j, start[j]);
    float j7_goal = js.q[J7];   // TRACK: J7's goal while the messages have 6 values
    state = track ? TRACKING : JOGGING;
    const float dt = 0.002f;
    const int max_err = 227;   // 20° in steps: abort and hold
    uint32_t next = micros();
    bool fault = false;
    uint8_t fault_joint = 0; int16_t fault_err = 0;
    float vpeak[N_ARM] = {0};
    const float vdecay = expf(-dt / 0.3f);
    for (;;) {
        float target[MAX_JOINTS], vmax;
        portENTER_CRITICAL(&jog_mux);
        memcpy(target, jog_target, sizeof(target));
        vmax = jog_vmax;
        uint32_t last = jog_ms;
        const int tn = jog_n;
        portEXIT_CRITICAL(&jog_mux);
        if (track) { if (tn > J7) j7_goal = target[J7]; target[J7] = j7_goal; }
        else if (tn <= J7) target[J7] = 0;
        bool any = false;
        const bool stop = stop_requested || millis() - last > (uint32_t)(lim::JOG_DEADMAN_S * 1000);
        if (track) {
            motion::track_step(js, target, n, vmax, stop, dt);
            any = !stop;
        } else {
            if (stop) memset(target, 0, sizeof(target));
            for (int j = 0; j < n; j++) any |= target[j] != 0;
            motion::jog_step(js, target, n, dt);
        }
        for (int j = 0; j < n; j++) cmd[j] = deg_to_pos(j, js.q[j]);
        sync_write_u16(REG_GOAL_POSITION, cmd, n);
        memcpy(goal_steps, cmd, 2 * n);
        bool ok = read_joints(pos, spd, load, n);
        publish_state(ok, n, pos, spd, load);
        slow_tick();
        // TRACK at speed: the servos lag their goal by about 0.11 s, and up to twice that with the integral
        // gain on J1-J3 in fast reversals (2026-10-05: J1 at 90 °/s went past 20°). So in TRACK the allowed
        // following error grows with the recent peak speed (it decays over 0.3 s, so a reversal through zero
        // speed keeps it): 20° + 0.15 s × speed, about 33° at 90 °/s. A blocked joint still stops the arm.
        // Not J7: on an object it stops short of its goal.
        for (int j = 0; j < N_ARM; j++) vpeak[j] = fmaxf(fabsf(js.v[j]), vpeak[j] * vdecay);
        if (ok) for (int j = 0; j < N_ARM; j++) {
            const int allowed = track ? max_err + (int)(0.15f * vpeak[j] * (4096.0f / 360.0f)) : max_err;
            const int e = (int)pos[j] - (int)cmd[j];
            if (abs(e) > allowed && !fault) { fault = true; fault_joint = j + 1; fault_err = e; }
        }
        if (fault) break;
        if (!any && motion::jog_stopped(js, n)) break;
        next += 2000;
        while ((int32_t)(micros() - next) < 0) {}
        if ((int32_t)(micros() - next) > 2000) next = micros();
    }
    // Stop: keep the last goals (goal_steps), so the loaded joints do not sag (5.0). After a tracking
    // error: hold where the arm is.
    if (fault) hold();
    uint16_t zero[N_ARM] = {0};
    sync_write_u16_verified(REG_GOAL_SPEED, zero, N_ARM);
    if (fault) finish(1, 0, 0, 0, fault_joint, fault_err);   // DONE "tracking error" to the client (4.4+)
    state = fault ? ERROR_STATE : (plan_valid ? READY : HOLDING);
}

// Idle: look for J7 once a second while it is not found; while someone subscribes, read the joints at
// the stream rate (≤ 100 Hz); while someone subscribes or J7 is found (thermal derating), the slow reads
// (slow_tick). During a run the run publishes instead.
void idle_reads() {
    static uint32_t next_state = 0;
    static uint16_t pos[MAX_JOINTS], spd[MAX_JOINTS], load[MAX_JOINTS];   // kept: a missed J7 reply keeps its last values
    if (state == PLAYING || state == OTA) return;
    j7_probe_tick();
    uint16_t rate = stream_rate;
    if (rate || n_joints > N_ARM) slow_tick();
    if (!rate) return;
    uint32_t now = millis();
    if ((int32_t)(now - next_state) >= 0) {
        next_state = now + 1000 / rate;
        int n = n_joints;
        bool ok = read_joints(pos, spd, load, n);
        publish_state(ok, n, pos, spd, load);
    }
}

void loop() {
    static bool btn_prev = false;
    bool btn = digitalRead(BTN_PIN) == LOW;
    if (btn && !btn_prev && state != PLAYING) { req_from = Addr(); request = REQ_HOLD; }   // acknowledge / clear error
    btn_prev = btn;

    Request r = request;
    if (r != REQ_NONE) {
        request = REQ_NONE;
        if (r == REQ_STATE) {
            uint8_t m[3 + 8 * MAX_JOINTS + 12] = {0x82};   // u8 ok, u8 n (5.0+), pos[n], goal[n], spd[n], load[n], acc, gyro
            uint16_t pos[MAX_JOINTS] = {0}, spd[MAX_JOINTS] = {0}, load[MAX_JOINTS] = {0};
            int n = n_joints;
            m[1] = read_joints(pos, spd, load, n);
            m[2] = n;
            memcpy(m + 3, pos, 2 * n); memcpy(m + 3 + 2 * n, goal_steps, 2 * n);
            memcpy(m + 3 + 4 * n, spd, 2 * n); memcpy(m + 3 + 6 * n, load, 2 * n);
            ImuSample im = imu_get();
            memcpy(m + 3 + 8 * n, im.acc, 6); memcpy(m + 9 + 8 * n, im.gyro, 6);
            post(m, 15 + 8 * n, req_from);
        } else if (r == REQ_HOLD) {
            if (turn_unsure) setup_multi_turn();   // the joint may have been moved by hand: find the turn again
            bool ok = !turn_unsure && hold_goals();
            state = ok ? (plan_valid ? READY : HOLDING) : ERROR_STATE;
            ack_from_control(0x03, ok ? 0 : -4);
        } else if (r == REQ_PLAY) {
            play();
        } else if (r == REQ_SIGNAL) {
            play(1);
        } else if (r == REQ_MOVE) {
            play(2);
        } else if (r == REQ_JOG) {
            jog_run();
        } else if (r == REQ_TRACK) {
            jog_run(true);
        } else if (r == REQ_REG) {
            uint8_t id = reg_req[1], addr = reg_req[2], len = reg_req[3];
            if (reg_req[0] == 0x09) {
                uint8_t m[5 + 32] = {0x86, id, addr, len, 0};
                if (!reg_read(id, addr, len, m + 5)) m[4] = (uint8_t)-4;
                post(m, m[4] == 0 ? 5 + len : 5, req_from);
            } else {
                int err = reg_write(id, addr, reg_req + 4, len);
                uint8_t back[32];
                int8_t status = err < 0 ? -4 : (!reg_read(id, addr, len, back) || memcmp(back, reg_req + 4, len)) ? -6 : 0;
                uint8_t m[5] = {0x87, id, addr, (uint8_t)status, (uint8_t)(err < 0 ? 0 : err)};
                post(m, sizeof(m), req_from);
            }
        }
    }
    idle_reads();
}
