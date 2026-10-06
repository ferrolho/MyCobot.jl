// atom_controller.ino — onboard trajectory player for the myCobot 280 ATOM (ESP32-PICO-D4).
//
// Core 1 (Arduino loop): owns the servo bus. Holds the pose at power-up, answers bus
//   requests from the network task, and plays uploaded plans at a fixed rate (default
//   500 Hz) with a start-pose check and a tracking-error abort.
// Core 0: IMU sampling (500 Hz), UDP commands, telemetry, status log, OTA updates.
//
// On the bus the ATOM is silent except at power-up and when commanded, so the laptop can
// still drive the servos through the FT232 while the ATOM is idle (never at the same time).
//
// Transports (4.2+): UDP (lab tools) and WebSocket ws://<ATOM>/ws, port 80 (browsers; binary frames =
// the same messages; text frames = the status log). mDNS name: mycobot.local.
// Control (4.2+): one client at a time may send PLAN_*, PLAY, PLAY_SIGNAL, REG_WRITE, MOVE_TO, JOG. If
// nobody has control, such a command takes it for its sender; otherwise another client gets status -2.
// Control ends on release, when its WebSocket closes, or 2 s after its last message (not during its run).
// HOLD and STOP work for every client (during a run they stop it).
//   0x0D CONTROL u8 action (0 release, 1 take, 2 take over) -> ACK 0 / -2 another client / -1 robot moving
//   0x0E MOVE_TO i16 goal[6] (0.01°), u16 duration_ms (0 = shortest) -> ACK, TELEM (UDP only, 4.3.1+), DONE; state 6 moving
//   0x0F JOG u8 frame (0 = joints), i16 velocity[6] (0.1°/s); ACK only if refused; 200 ms deadman; state 7
//   0x10 TRACK i16 goal[6] (0.01°), u16 vmax (0.1°/s, ≤ 90°/s) (4.4+): the joints go to the goal at up to vmax
//        and their acceleration limits and stop on it (the Control page's Live mode); send it again at least
//        every 200 ms (deadman: brake and hold). ACK only if refused (-1 busy, -10-j goal of joint j outside); state 8
//        JOG and TRACK stopped by a following error (20°; in TRACK 20° + 0.15 s × recent peak speed) -> DONE result 1, joint, error
//   STREAM (4.2+) has one more byte at offset 73: control 0 nobody, 1 you, 2 another client (74 bytes).
// Full API: website/src/content/docs/comms/websocket-api.md.
//
// UDP protocol (little-endian). Clients -> ATOM port 5006. Replies go to the sender of each request
// (its IP and UDP port; 4.0 and older replied to the last sender's IP, port 5007). PLAY telemetry and
// DONE go to the client that started the run.
//   0x01 PING                                   -> 0x81 PONG  u16 version, u8 state, u32 plan_samples, u16 plan_rate, u8 imu_ok,
//                                                             u8 gains_ok, u8 minor, u8 patch
//   0x02 STATE                                  -> 0x82 STATE u8 ok, u16 pos[6], u16 spd[6], u16 load[6], i16 acc[3], i16 gyro[3]
//   0x03 HOLD                                   -> 0x83 ACK   u8 0x03, i8 status
//   0x04 PLAN_BEGIN u32 n, u16 rate_hz [, u8 interp: 0 linear, 1 cubic (Catmull-Rom; 3.1+)]
//                                               -> 0x83 ACK   u8 0x04, i8 status
//   0x05 PLAN_DATA  u32 offset, u16 count, count × {u16 cmd[6], u16 ref[6]}   (servo steps)
//                                               -> 0x83 ACK   u8 0x05, i8 status, u32 offset
//   0x06 PLAN_END   u32 crc32c(all samples)     -> 0x83 ACK   u8 0x06, i8 status
//   0x07 PLAY u16 rate_hz, u16 speed_cap, u16 max_err_steps, u16 start_tol_steps
//                                               -> 0x83 ACK   u8 0x07, i8 status (0 = started)
//        during play                            -> 0x84 TELEM u32 first_seq, u8 n (≤ 18), n × Sample (77 bytes, 4.0+):
//                                                  u32 t_us, u16 cmd[6], u16 ref[6], u16 pos[6], u16 spd[6],
//                                                  u16 load[6], i16 acc[3], i16 gyro[3], u8 ok
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
//                                               -> 0x88 STREAM u32 t_ms, u8 state, u8 ok, u16 pos[6], u16 spd[6],
//                                                  u16 load[6], u8 temp[6], u8 volt[6], u8 status[6], i16 acc[3], i16 gyro[3]
//        id 1-7, len 1-32, not while playing. Status: 0 ok, -1 busy or bad request, -4 no reply,
//        -5 write not allowed (registers 0-8, 55 and 80+: ID, baud rate, EEPROM lock, factory),
//        -6 the read-back differs. Writes in the EEPROM area last until the next power cycle.
// Status log (text) once a second: UDP broadcast, port 5005.
//
// Power-up: hold the pose, then write the position-loop gains GAINS and set up the multi-turn joints
// (MULTI_TURN, 4.5+), all verified. PONG's gains_ok is 1 when both took.
//
// LED matrix: blue = starting, green = holding, yellow = plan ready, cyan + progress = playing,
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
#define FW_MAJOR 4
#define FW_MINOR 5
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
struct PlanSample;   // the Arduino builder puts function prototypes above the first function
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
struct PlanSample { uint16_t cmd[N_SERVOS]; uint16_t ref[N_SERVOS]; };
PlanSample* plan = nullptr;
uint32_t plan_n = 0, plan_received = 0;
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
struct __attribute__((packed)) Sample {
    uint32_t t_us;
    uint16_t cmd[N_SERVOS], ref[N_SERVOS];   // goal written and reference, as computed onboard (4.0+)
    uint16_t pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS];
    int16_t acc[3], gyro[3];
    uint8_t ok;
};
const int RING = 512;
const int TELEM_BATCH = 18;   // samples per UDP packet (18 × 77 bytes + 6 ≤ 1472)
Sample ring[RING];
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
    for (int j = 0; j < N_SERVOS; j++) ok &= reg_write_verified(j + 1, 21, GAINS[j], 3);
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
    for (int j = 0; j < N_SERVOS; j++) {
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
struct OutMsg { Addr to; uint8_t len; uint8_t data[63]; };
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
    uint32_t t_ms; uint8_t ok;
    uint16_t pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS];
    uint8_t temp[N_SERVOS], volt[N_SERVOS], status[N_SERVOS];
} latest;
portMUX_TYPE latest_mux = portMUX_INITIALIZER_UNLOCKED;

void publish_state(bool ok, const uint16_t pos[], const uint16_t spd[], const uint16_t load[]) {
    portENTER_CRITICAL(&latest_mux);
    latest.t_ms = millis(); latest.ok = ok;
    memcpy(latest.pos, pos, sizeof(latest.pos)); memcpy(latest.spd, spd, sizeof(latest.spd));
    memcpy(latest.load, load, sizeof(latest.load));
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
float move_goal[N_SERVOS]; uint16_t move_dur_ms = 0;
float jog_target[N_SERVOS]; volatile uint32_t jog_ms = 0;   // JOG velocities, or TRACK goal (jog_vmax)
float jog_vmax = 0;                                           // TRACK speed cap (°/s)
portMUX_TYPE jog_mux = portMUX_INITIALIZER_UNLOCKED;

void serve_stream() {                              // network task
    uint32_t now = millis();
    uint16_t top = 0;
    uint8_t pkt[74]; bool built = false;
    for (int i = 0; i < MAX_SUBS; i++) {
        Sub& s = subs[i];
        if (!s.rate) continue;
        if (now - s.last_ms > 2000) { s.rate = 0; continue; }   // no SUBSCRIBE for 2 s
        top = max(top, s.rate);
        if ((int32_t)(now - s.next_ms) < 0) continue;
        s.next_ms += 1000 / s.rate;
        if ((int32_t)(now - s.next_ms) > 0) s.next_ms = now + 1000 / s.rate;   // late: skip ahead
        if (!built) {
            Latest l;
            portENTER_CRITICAL(&latest_mux); l = latest; portEXIT_CRITICAL(&latest_mux);
            ImuSample im = imu_get();
            pkt[0] = 0x88;
            memcpy(pkt + 1, &now, 4); pkt[5] = state; pkt[6] = l.ok;
            memcpy(pkt + 7, l.pos, 12); memcpy(pkt + 19, l.spd, 12); memcpy(pkt + 31, l.load, 12);
            memcpy(pkt + 43, l.temp, 6); memcpy(pkt + 49, l.volt, 6); memcpy(pkt + 55, l.status, 6);
            memcpy(pkt + 61, im.acc, 6); memcpy(pkt + 67, im.gyro, 6);
            built = true;
        }
        pkt[73] = !valid(ctrl) ? 0 : (same(s.a, ctrl) ? 1 : 2);   // control: nobody, you, another client
        send_to(s.a, pkt, sizeof(pkt));
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
    case 0x0E: {   // MOVE_TO i16 goal[6] (0.01°), u16 duration_ms (0 = shortest within the limits)
        if (n != 15) { ack(0x0E, -1); break; }
        if (busy() || request != REQ_NONE) { ack(0x0E, -1); break; }
        float g[N_SERVOS];
        for (int j = 0; j < N_SERVOS; j++) { int16_t v; memcpy(&v, b + 1 + 2 * j, 2); g[j] = v * 0.01f; }
        int err = motion::move_validate(g);
        if (err) { ack(0x0E, -10 - err); break; }
        memcpy(move_goal, g, sizeof(g)); memcpy(&move_dur_ms, b + 13, 2);
        play_params = {500, 2000, 227, 34};   // 500 Hz, speed cap, abort at 20° tracking error
        stop_requested = false;
        req_from = play_to = cmd_from;
        telem_on = cmd_from.ws == 0xFF;   // UDP: TELEM as before; WebSocket: the STREAM only (see the network task)
        request = REQ_MOVE;
        break;
    }
    case 0x0F: {   // JOG u8 frame (0 = joints), i16 velocity[6] (0.1°/s). ACK only if refused.
        if (n != 14 || b[1] != 0) { ack(0x0F, -1); break; }
        float v[N_SERVOS];
        for (int j = 0; j < N_SERVOS; j++) { int16_t x; memcpy(&x, b + 2 + 2 * j, 2); v[j] = x * 0.1f; }
        if (state == JOGGING) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, v, sizeof(v)); jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
        } else if ((state == HOLDING || state == READY) && request == REQ_NONE) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, v, sizeof(v)); jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
            stop_requested = false;
            req_from = play_to = cmd_from; telem_on = true;
            request = REQ_JOG;
        } else ack(0x0F, -1);
        break;
    }
    case 0x10: {   // TRACK i16 goal[6] (0.01°), u16 vmax (0.1°/s). ACK only if refused.
        if (n != 15) { ack(0x10, -1); break; }
        float g[N_SERVOS];
        for (int j = 0; j < N_SERVOS; j++) { int16_t x; memcpy(&x, b + 1 + 2 * j, 2); g[j] = x * 0.01f; }
        uint16_t vm; memcpy(&vm, b + 13, 2);
        int err = motion::move_validate(g);
        if (err) { ack(0x10, -10 - err); break; }
        if (state == TRACKING) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, g, sizeof(g)); jog_vmax = vm * 0.1f; jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
        } else if ((state == HOLDING || state == READY) && request == REQ_NONE) {
            portENTER_CRITICAL(&jog_mux); memcpy(jog_target, g, sizeof(g)); jog_vmax = vm * 0.1f; jog_ms = millis(); portEXIT_CRITICAL(&jog_mux);
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
        if (cnt < 2 || rate == 0) { ack(0x04, -2); break; }
        plan = (PlanSample*)malloc(cnt * sizeof(PlanSample));
        if (!plan) { ack(0x04, -3); break; }   // not enough memory
        plan_n = cnt; plan_rate = rate;
        plan_cubic = n >= 8 && b[7] == 1;
        ack(0x04, 0);
        break;
    }
    case 0x05: {   // PLAN_DATA
        uint32_t off; uint16_t cnt; memcpy(&off, b + 1, 4); memcpy(&cnt, b + 5, 2);
        if (!plan || off + cnt > plan_n || n != 7 + cnt * (int)sizeof(PlanSample)) { ack(0x05, -1, off); break; }
        memcpy(plan + off, b + 7, cnt * sizeof(PlanSample));
        plan_received = max(plan_received, off + cnt);
        ack(0x05, 0, off);
        break;
    }
    case 0x06: {   // PLAN_END
        uint32_t crc; memcpy(&crc, b + 1, 4);
        bool ok = plan && plan_received == plan_n &&
                  crc32c(0, (const uint8_t*)plan, plan_n * sizeof(PlanSample)) == crc;
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
void show(uint8_t r, uint8_t g, uint8_t b, int progress_permille = -1) {
    for (int i = 0; i < 25; i++) matrix.setPixelColor(i, matrix.Color(r, g, b));
    if (progress_permille >= 0) {
        int lit = progress_permille * 5 / 1000;
        for (int x = 0; x < 5; x++) matrix.setPixelColor(20 + x, x < lit ? matrix.Color(40, 40, 40) : 0);
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

        static bool mdns_up = false;
        if (!mdns_up && WiFi.status() == WL_CONNECTED) {   // mycobot.local, once WiFi is up
            mdns_up = MDNS.begin("mycobot");
            if (mdns_up) MDNS.addService("http", "tcp", 80);
        }
        wss.loop();
        control_tick();
        OutMsg om;
        while (xQueueReceive(outbox, &om, 0) == pdTRUE) send_to(om.to, om.data, om.len);
        serve_stream();

        static uint32_t last_led = 0;
        if (millis() - last_led >= 100) { last_led = millis(); update_led(); }

        // Telemetry: batches of up to 18 samples. Over a WebSocket (TCP) every send waits for the
        // client: 500 Hz telemetry (28 packets/s) blocked this task during MOVE_TO, so the STREAM paused
        // for 0.1-0.5 s and replies came late (2026-10-05). A WebSocket MOVE_TO sends no TELEM.
        if (!telem_on) { __sync_synchronize(); ring_tail = ring_head; }
        while (ring_tail != ring_head) {
            __sync_synchronize();
            uint32_t avail = ring_head - ring_tail;
            uint8_t cnt = min<uint32_t>(avail, TELEM_BATCH);
            uint8_t pkt[6 + TELEM_BATCH * sizeof(Sample)];
            pkt[0] = 0x84;
            uint32_t seq = ring_tail; memcpy(pkt + 1, &seq, 4); pkt[5] = cnt;
            for (int i = 0; i < cnt; i++) memcpy(pkt + 6 + i * sizeof(Sample), &ring[(ring_tail + i) % RING], sizeof(Sample));
            __sync_synchronize();
            ring_tail += cnt;
            send_to(play_to, pkt, 6 + cnt * sizeof(Sample));
        }
        if (done_pending && ring_tail == ring_head) {
            send_to(play_to, &done_msg, sizeof(done_msg));
            done_pending = false;
        }

        uint32_t now = millis();
        if (now - last_log >= 1000) {
            last_log = now;
            char line[256];
            snprintf(line, sizeof(line), "atom_controller v%d.%d.%d (%s, %s) ip=%s rssi=%d state=%d plan=%lu@%uHz valid=%d imu=%d write_retries=%lu heap=%lu up=%lus%s",
                     FW_MAJOR, FW_MINOR, FW_PATCH, FW_GIT, FW_VARIANT, WiFi.localIP().toString().c_str(), WiFi.RSSI(), state, (unsigned long)plan_n, plan_rate,
                     plan_valid, imu_ok, (unsigned long)write_retries, (unsigned long)ESP.getFreeHeap(), (unsigned long)(now / 1000),
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
void interp(const PlanSample* p, uint32_t n, float s, uint16_t cmd[N_SERVOS], uint16_t ref[N_SERVOS]) {
    if (s <= 0) { memcpy(cmd, p[0].cmd, sizeof(p[0].cmd)); memcpy(ref, p[0].ref, sizeof(p[0].ref)); return; }
    if (s >= n - 1) { memcpy(cmd, p[n - 1].cmd, sizeof(p[0].cmd)); memcpy(ref, p[n - 1].ref, sizeof(p[0].ref)); return; }
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
        for (int j = 0; j < N_SERVOS; j++) {
            cmd[j] = cr(j, p[i0].cmd[j], p[i].cmd[j], p[i + 1].cmd[j], p[i3].cmd[j]);
            ref[j] = cr(j, p[i0].ref[j], p[i].ref[j], p[i + 1].ref[j], p[i3].ref[j]);
        }
        return;
    }
    for (int j = 0; j < N_SERVOS; j++) {
        cmd[j] = (uint16_t)lroundf(p[i].cmd[j] + f * ((int)p[i + 1].cmd[j] - (int)p[i].cmd[j]));
        ref[j] = (uint16_t)lroundf(p[i].ref[j] + f * ((int)p[i + 1].ref[j] - (int)p[i].ref[j]));
    }
}

void finish(uint8_t result, uint32_t cycles, uint32_t max_period, uint32_t late, uint8_t joint, int16_t err) {
    done_msg.result = result; done_msg.cycles = cycles; done_msg.max_period_us = max_period;
    done_msg.late_cycles = late; done_msg.telem_dropped = telem_dropped; done_msg.joint = joint; done_msg.error_steps = err;
    done_pending = true;
}

// Result codes: 0 done, 1 tracking error, 2 stopped, 3 not at the start pose, 4 bus error
// Runs a motion at a fixed rate with telemetry: src 0 = the uploaded plan (PLAY), 1 = the
// PLAY_SIGNAL test signal, 2 = a MOVE_TO minimum-jerk move (4.2+), all computed onboard.
void play(uint8_t src = 0) {
    const PlayParams pp = play_params;
    const bool use_signal = src == 1, use_move = src == 2;
    const uint8_t type = use_signal ? 0x0B : (use_move ? 0x0E : 0x07);
    if (pp.rate < 50 || pp.rate > 800) { ack_from_control(type, -2); return; }
    uint16_t pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS], cmd[N_SERVOS], ref[N_SERVOS];
    if (!read_state(pos, spd, load)) { ack_from_control(type, -4); finish(4, 0, 0, 0, 0, 0); state = ERROR_STATE; return; }
    const sig::Params sp = signal_params;
    float start_deg[N_SERVOS], q_deg[N_SERVOS];
    for (int j = 0; j < N_SERVOS; j++) start_deg[j] = pos_to_deg(j, pos[j]);
    if (use_signal) {
        int err = sig::validate_start(sp, start_deg);
        if (err) { ack_from_control(type, -20 - err); finish(3, 0, 0, 0, 0, 0); return; }
    } else if (!use_move) {
        for (int j = 0; j < N_SERVOS; j++) {
            int e = (int)pos[j] - (int)plan[0].ref[j];
            if (abs(e) > pp.start_tol) { ack_from_control(type, -3); finish(3, 0, 0, 0, j + 1, e); return; }
        }
    }
    float goal[N_SERVOS], move_T = 0;
    if (use_move) {
        memcpy(goal, move_goal, sizeof(goal));
        float Tmin = motion::move_min_duration(start_deg, goal);
        move_T = move_dur_ms ? move_dur_ms * 0.001f : Tmin;
        if (move_T < Tmin * 0.999f) { ack_from_control(type, -1); finish(3, 0, 0, 0, 0, 0); return; }
    }
    ack_from_control(type, 0);
    state = use_move ? MOVING : PLAYING;
    telem_dropped = 0;

    // Enable motion: hold, no acceleration ramp, speed cap (each write verified)
    uint16_t caps[N_SERVOS]; for (int j = 0; j < N_SERVOS; j++) caps[j] = pp.speed_cap;
    if (!sync_write_u16_verified(REG_GOAL_POSITION, pos) || !sync_write_u8_verified(REG_ACCELERATION, 0) ||
        !sync_write_u16_verified(REG_GOAL_SPEED, caps)) {
        hold_pose();
        finish(4, 0, 0, 0, 0, 0); state = ERROR_STATE; return;
    }

    const uint32_t period = 1000000UL / pp.rate;
    const float duration_s = (use_signal ? sig::total_s(sp) : use_move ? move_T : (plan_n - 1) / (float)plan_rate) + 0.5f;   // plus 0.5 s settling
    uint32_t t0 = micros(), next = t0, cycles = 0, late = 0, max_period = 0, last = t0;
    uint8_t result = 0, bad_joint = 0; int16_t bad_err = 0;
    for (;;) {
        uint32_t now = micros();
        float t = (now - t0) * 1e-6f;
        if (t >= duration_s) break;
        if (stop_requested) { result = 2; break; }

        if (use_signal) {
            sig::eval(sp, start_deg, t, q_deg);
            for (int j = 0; j < N_SERVOS; j++) cmd[j] = ref[j] = deg_to_pos(j, q_deg[j]);
        } else if (use_move) {
            float s_ = motion::minjerk(t / move_T);
            for (int j = 0; j < N_SERVOS; j++) cmd[j] = ref[j] = deg_to_pos(j, start_deg[j] + s_ * (goal[j] - start_deg[j]));
        } else {
            interp(plan, plan_n, t * plan_rate, cmd, ref);
        }
        sync_write_u16(REG_GOAL_POSITION, cmd);
        bool ok = read_state(pos, spd, load);
        if (stream_rate) publish_state(ok, pos, spd, load);

        Sample& s = ring[ring_head % RING];
        if (ring_head - ring_tail >= RING) { telem_dropped++; }
        else {
            s.t_us = now - t0;
            memcpy(s.cmd, cmd, sizeof(cmd)); memcpy(s.ref, ref, sizeof(ref));
            memcpy(s.pos, pos, sizeof(pos)); memcpy(s.spd, spd, sizeof(spd)); memcpy(s.load, load, sizeof(load));
            ImuSample im = imu_get();
            memcpy(s.acc, im.acc, sizeof(s.acc)); memcpy(s.gyro, im.gyro, sizeof(s.gyro));
            s.ok = ok;
            __sync_synchronize();
            ring_head++;
        }

        if (ok) {
            for (int j = 0; j < N_SERVOS; j++) {
                int e = (int)pos[j] - (int)ref[j];
                if (abs(e) > pp.max_err) { result = 1; bad_joint = j + 1; bad_err = e; break; }
            }
            if (result) break;
        }

        cycles++;
        play_progress_permille = (uint32_t)(1000 * t / duration_s);
        uint32_t dt = now - last; last = now;
        if (cycles > 1) max_period = max(max_period, dt);
        next += period;
        if ((int32_t)(micros() - next) > 0) { late++; next = micros(); }   // late: don't try to catch up
        while ((int32_t)(micros() - next) < 0) {}
    }

    if (result) hold_pose();                         // abort: stay where we are
    else delay(300);
    uint16_t zero[N_SERVOS] = {0};
    sync_write_u16_verified(REG_GOAL_SPEED, zero);   // back to "don't move"
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

    // Power-up: hold the pose (torque on, goal speed 0). Retry until all servos answer.
    delay(300);
    for (int k = 0; k < 20 && !hold_pose(); k++) delay(100);
    for (int k = 0; k < 5 && !gains_ok; k++) { gains_ok = write_gains() && setup_multi_turn(); if (!gains_ok) delay(100); }
    if (!turn_unsure) hold_pose();   // goals in our steps (turn_offset)
    uint16_t p[N_SERVOS], s[N_SERVOS], l[N_SERVOS];
    state = read_state(p, s, l) && !turn_unsure ? HOLDING : ERROR_STATE;
}

// JOG (4.2+): the client streams joint velocities; the goals integrate them at 500 Hz within the
// speed, acceleration and joint limits (motion.h). No JOG for 200 ms (deadman), STOP, HOLD or a
// zero velocity: ramp down at the acceleration limit, then hold. The state stream shows the motion.
// TRACK (4.4+, track = true): the client streams a goal pose; the joints go there (motion::track_step)
// and stay there while TRACK keeps coming. Deadman, STOP or HOLD: brake to zero, then hold.
void jog_run(bool track = false) {
    uint16_t pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS], cmd[N_SERVOS];
    if (!read_state(pos, spd, load)) { ack_from_control(track ? 0x10 : 0x0F, -4); return; }
    uint16_t caps[N_SERVOS]; for (int j = 0; j < N_SERVOS; j++) caps[j] = 2000;
    if (!sync_write_u16_verified(REG_GOAL_POSITION, pos) || !sync_write_u8_verified(REG_ACCELERATION, 0) ||
        !sync_write_u16_verified(REG_GOAL_SPEED, caps)) { hold_pose(); state = ERROR_STATE; return; }
    motion::Jog js = {};
    for (int j = 0; j < N_SERVOS; j++) js.q[j] = pos_to_deg(j, pos[j]);
    state = track ? TRACKING : JOGGING;
    const float dt = 0.002f;
    const int max_err = 227;   // 20° in steps: abort and hold
    uint32_t next = micros();
    bool fault = false;
    uint8_t fault_joint = 0; int16_t fault_err = 0;
    float vpeak[N_SERVOS] = {0};
    const float vdecay = expf(-dt / 0.3f);
    for (;;) {
        float target[N_SERVOS], vmax;
        portENTER_CRITICAL(&jog_mux);
        memcpy(target, jog_target, sizeof(target));
        vmax = jog_vmax;
        uint32_t last = jog_ms;
        portEXIT_CRITICAL(&jog_mux);
        bool any = false;
        const bool stop = stop_requested || millis() - last > (uint32_t)(lim::JOG_DEADMAN_S * 1000);
        if (track) {
            motion::track_step(js, target, vmax, stop, dt);
            any = !stop;
        } else {
            if (stop) memset(target, 0, sizeof(target));
            for (int j = 0; j < N_SERVOS; j++) any |= target[j] != 0;
            motion::jog_step(js, target, dt);
        }
        for (int j = 0; j < N_SERVOS; j++) cmd[j] = deg_to_pos(j, js.q[j]);
        sync_write_u16(REG_GOAL_POSITION, cmd);
        bool ok = read_state(pos, spd, load);
        publish_state(ok, pos, spd, load);
        // TRACK at speed: the servos lag their goal by about 0.11 s, and up to twice that with the integral
        // gain on J1-J3 in fast reversals (2026-10-05: J1 at 90 °/s went past 20°). So in TRACK the allowed
        // following error grows with the recent peak speed (it decays over 0.3 s, so a reversal through zero
        // speed keeps it): 20° + 0.15 s × speed, about 33° at 90 °/s. A blocked joint still stops the arm.
        for (int j = 0; j < N_SERVOS; j++) vpeak[j] = fmaxf(fabsf(js.v[j]), vpeak[j] * vdecay);
        if (ok) for (int j = 0; j < N_SERVOS; j++) {
            const int allowed = track ? max_err + (int)(0.15f * vpeak[j] * (4096.0f / 360.0f)) : max_err;
            const int e = (int)pos[j] - (int)cmd[j];
            if (abs(e) > allowed && !fault) { fault = true; fault_joint = j + 1; fault_err = e; }
        }
        if (fault) break;
        if (!any && motion::jog_stopped(js)) break;
        next += 2000;
        while ((int32_t)(micros() - next) < 0) {}
        if ((int32_t)(micros() - next) > 2000) next = micros();
    }
    hold_pose();
    uint16_t zero[N_SERVOS] = {0};
    sync_write_u16_verified(REG_GOAL_SPEED, zero);
    if (fault) finish(1, 0, 0, 0, fault_joint, fault_err);   // DONE "tracking error" to the client (4.4+)
    state = fault ? ERROR_STATE : (plan_valid ? READY : HOLDING);
}

// While idle and someone subscribes: read the state at the stream rate (≤ 100 Hz), and the
// temperatures, voltages and status once a second. During PLAY the control loop publishes instead.
void idle_stream_reads() {
    static uint32_t next_state = 0, next_slow = 0;
    uint16_t rate = stream_rate;
    if (!rate || state == PLAYING || state == OTA) return;
    uint32_t now = millis();
    if ((int32_t)(now - next_state) >= 0) {
        next_state = now + 1000 / rate;
        uint16_t pos[N_SERVOS] = {0}, spd[N_SERVOS] = {0}, load[N_SERVOS] = {0};
        bool ok = read_state(pos, spd, load);
        publish_state(ok, pos, spd, load);
    }
    if ((int32_t)(now - next_slow) >= 0) {
        next_slow = now + 1000;
        uint8_t d[N_SERVOS][16], st[N_SERVOS][16];
        uint8_t m1 = sync_read(62, 2, d), m2 = sync_read(65, 1, st);
        portENTER_CRITICAL(&latest_mux);
        for (int j = 0; j < N_SERVOS; j++) {
            if (m1 & (1 << j)) { latest.volt[j] = d[j][0]; latest.temp[j] = d[j][1]; }
            if (m2 & (1 << j)) latest.status[j] = st[j][0];
        }
        portEXIT_CRITICAL(&latest_mux);
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
            uint8_t m[1 + 1 + 36 + 12] = {0x82};
            uint16_t pos[N_SERVOS] = {0}, spd[N_SERVOS] = {0}, load[N_SERVOS] = {0};
            m[1] = read_state(pos, spd, load);
            memcpy(m + 2, pos, 12); memcpy(m + 14, spd, 12); memcpy(m + 26, load, 12);
            ImuSample im = imu_get();
            memcpy(m + 38, im.acc, 6); memcpy(m + 44, im.gyro, 6);
            post(m, sizeof(m), req_from);
        } else if (r == REQ_HOLD) {
            if (turn_unsure) setup_multi_turn();   // the joint may have been moved by hand: find the turn again
            bool ok = !turn_unsure && hold_pose();
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
    idle_stream_reads();
}
