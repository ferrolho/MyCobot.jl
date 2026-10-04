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
// UDP protocol (little-endian). Laptop -> ATOM port 5006; replies go to the sender's IP, port 5007.
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
//        during play                            -> 0x84 TELEM u32 first_seq, u8 n, n × Sample
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
//        id 1-7, len 1-32, not while playing. Status: 0 ok, -1 busy or bad request, -4 no reply,
//        -5 write not allowed (registers 0-8, 55 and 80+: ID, baud rate, EEPROM lock, factory),
//        -6 the read-back differs. Writes in the EEPROM area last until the next power cycle.
// Status log (text) once a second: UDP broadcast, port 5005.
//
// Power-up: hold the pose, then write the position-loop gains GAINS (verified).
//
// LED matrix: blue = starting, green = holding, yellow = plan ready, cyan + progress = playing,
//   red = error (press the button to clear and hold again), magenta = OTA update.
// The button only acknowledges (clear error + hold). It is NOT an emergency stop.
//
// Build: see firmware/README.md (needs ~/.config/mycobot/wifi_secrets.h).

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>

#include "bus.h"
#include "imu.h"
#include "test_signal.h"
#include "wifi_secrets.h"

// Semantic version: MAJOR for protocol changes that break old clients, MINOR for added commands,
// PATCH for fixes. PING reports MAJOR as its u16 version, then MINOR and PATCH (3.1+).
// History: docs (firmware/changelog). FW_GIT is set by the build (git describe).
#define FW_MAJOR 3
#define FW_MINOR 1
#define FW_PATCH 0
#define FW_VERSION FW_MAJOR
#ifndef FW_GIT
#define FW_GIT "unknown"
#endif
#define LED_PIN    27
#define BTN_PIN    39
#define CMD_PORT   5006
#define REPLY_PORT 5007
#define LOG_PORT   5005

enum State : uint8_t { BOOTING = 0, HOLDING = 1, READY = 2, PLAYING = 3, ERROR_STATE = 4, OTA = 5 };
volatile State state = BOOTING;

Adafruit_NeoPixel matrix(25, LED_PIN, NEO_GRB + NEO_KHZ800);
WiFiUDP cmd_udp, out_udp;
IPAddress host_ip;
volatile bool have_host = false;

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
    uint16_t pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS];
    int16_t acc[3], gyro[3];
    uint8_t ok;
};
const int RING = 512;
Sample ring[RING];
volatile uint32_t ring_head = 0, ring_tail = 0, telem_dropped = 0;

// ---- Requests from the network task to the control loop ---------------------------------------
enum Request : uint8_t { REQ_NONE, REQ_STATE, REQ_HOLD, REQ_PLAY, REQ_REG, REQ_SIGNAL };
volatile Request request = REQ_NONE;
volatile bool stop_requested = false;
struct PlayParams { uint16_t rate, speed_cap, max_err, start_tol; } play_params;
uint8_t reg_req[4 + 32];   // REG_READ / REG_WRITE request, copied by the network task
sig::Params signal_params;  // PLAY_SIGNAL request

// Joint angle (°) <-> servo position, as MyCobot.angle_to_position (0° = 2048).
const int8_t JOINT_SIGN[N_SERVOS] = {-1, -1, +1, -1, -1, -1};
inline uint16_t deg_to_pos(int j, float deg) {
    long p = lroundf(2048 + JOINT_SIGN[j] * deg * (4096.0f / 360.0f));
    return (uint16_t)(p < 0 ? 0 : (p > 4095 ? 4095 : p));
}
inline float pos_to_deg(int j, uint16_t p) { return JOINT_SIGN[j] * ((int)p - 2048) * (360.0f / 4096.0f); }

// ---- Servo position-loop gains (P, D, I = registers 21, 22, 23), written at power-up ----------
// The servos store 32/8/0 (no integral action). Integral action on J1-J3 halves the tracking
// error on the circle without more end-effector vibration; a higher P raises the vibration
// (2026-10-04, docs: results/servo-dynamics). Keep in sync with MyCobot.GAINS.
const uint8_t GAINS[N_SERVOS][3] = {{32, 4, 16}, {32, 4, 16}, {32, 4, 16}, {32, 8, 0}, {32, 8, 0}, {32, 8, 0}};
volatile bool gains_ok = false;

bool write_gains() {
    bool ok = true;
    for (int j = 0; j < N_SERVOS; j++) ok &= reg_write_verified(j + 1, 21, GAINS[j], 3);
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
struct OutMsg { uint8_t len; uint8_t data[63]; };
QueueHandle_t outbox;

void post(const void* data, size_t n) {
    OutMsg m; m.len = min<size_t>(n, sizeof(m.data));
    memcpy(m.data, data, m.len);
    xQueueSend(outbox, &m, 0);
}

void send_to_host(const void* data, size_t n) {
    if (!have_host) return;
    out_udp.beginPacket(host_ip, REPLY_PORT);
    out_udp.write(reinterpret_cast<const uint8_t*>(data), n);
    out_udp.endPacket();
}

void ack(uint8_t type, int8_t status, uint32_t value = 0) {        // network task
    uint8_t m[7] = {0x83, type, (uint8_t)status};
    memcpy(m + 3, &value, 4);
    send_to_host(m, sizeof(m));
}

void ack_from_control(uint8_t type, int8_t status) {               // control loop
    uint8_t m[7] = {0x83, type, (uint8_t)status, 0, 0, 0, 0};
    post(m, sizeof(m));
}

void handle_command(const uint8_t* b, int n) {
    switch (b[0]) {
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
    case 0x02: if (state != PLAYING) request = REQ_STATE; break;
    case 0x03: if (state != PLAYING) request = REQ_HOLD; else ack(0x03, -1); break;
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
        request = REQ_PLAY;
        break;
    }
    case 0x08: stop_requested = true; ack(0x08, 0); break;
    case 0x0B: {   // PLAY_SIGNAL
        if (state == PLAYING || request != REQ_NONE || n != 1 + 8 + (int)sizeof(sig::Params)) { ack(0x0B, -1); break; }
        sig::Params p; memcpy(&p, b + 9, sizeof(p));
        int err = sig::validate(p);
        if (err) { ack(0x0B, -10 - err); break; }
        memcpy(&play_params, b + 1, 8);
        signal_params = p;
        stop_requested = false;
        request = REQ_SIGNAL;
        break;
    }
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
        request = REQ_REG;
        break;
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
    switch (state) {
    case BOOTING: show(0, 0, 40); break;
    case HOLDING: show(0, 40, 0); break;
    case READY:   show(40, 30, 0); break;
    case PLAYING: show(0, 30, 30, play_progress_permille); break;
    case ERROR_STATE: show(50, 0, 0); break;
    case OTA:     show(40, 0, 40); break;
    }
}

// ---- Network task (core 0) -----------------------------------------------------------------------
void net_task(void*) {
    uint8_t buf[1500];
    uint32_t last_log = 0;
    for (;;) {
        if (state != PLAYING) ArduinoOTA.handle();

        int n = cmd_udp.parsePacket();
        if (n > 0) {
            host_ip = cmd_udp.remoteIP(); have_host = true;
            n = cmd_udp.read(buf, sizeof(buf));
            if (n > 0) handle_command(buf, n);
        }

        OutMsg om;
        while (xQueueReceive(outbox, &om, 0) == pdTRUE) send_to_host(om.data, om.len);

        static uint32_t last_led = 0;
        if (millis() - last_led >= 100) { last_led = millis(); update_led(); }

        // Telemetry: batches of up to 20 samples
        while (ring_tail != ring_head) {
            __sync_synchronize();
            uint32_t avail = ring_head - ring_tail;
            uint8_t cnt = min<uint32_t>(avail, 20);
            uint8_t pkt[6 + 20 * sizeof(Sample)];
            pkt[0] = 0x84;
            uint32_t seq = ring_tail; memcpy(pkt + 1, &seq, 4); pkt[5] = cnt;
            for (int i = 0; i < cnt; i++) memcpy(pkt + 6 + i * sizeof(Sample), &ring[(ring_tail + i) % RING], sizeof(Sample));
            __sync_synchronize();
            ring_tail += cnt;
            send_to_host(pkt, 6 + cnt * sizeof(Sample));
        }
        if (done_pending && ring_tail == ring_head) {
            send_to_host(&done_msg, sizeof(done_msg));
            done_pending = false;
        }

        uint32_t now = millis();
        if (now - last_log >= 1000) {
            last_log = now;
            char line[256];
            snprintf(line, sizeof(line), "atom_controller v%d.%d.%d (%s) ip=%s rssi=%d state=%d plan=%lu@%uHz valid=%d imu=%d write_retries=%lu heap=%lu up=%lus",
                     FW_MAJOR, FW_MINOR, FW_PATCH, FW_GIT, WiFi.localIP().toString().c_str(), WiFi.RSSI(), state, (unsigned long)plan_n, plan_rate,
                     plan_valid, imu_ok, (unsigned long)write_retries, (unsigned long)ESP.getFreeHeap(), (unsigned long)(now / 1000));
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
        auto cr = [&](float p0, float p1, float p2, float p3) {
            float v = 0.5f * (2 * p1 + (p2 - p0) * f + (2 * p0 - 5 * p1 + 4 * p2 - p3) * f2 + (3 * p1 - p0 - 3 * p2 + p3) * f3);
            long r = lroundf(v);
            return (uint16_t)(r < 0 ? 0 : (r > 4095 ? 4095 : r));
        };
        for (int j = 0; j < N_SERVOS; j++) {
            cmd[j] = cr(p[i0].cmd[j], p[i].cmd[j], p[i + 1].cmd[j], p[i3].cmd[j]);
            ref[j] = cr(p[i0].ref[j], p[i].ref[j], p[i + 1].ref[j], p[i3].ref[j]);
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
// Plays the uploaded plan, or with `use_signal` the PLAY_SIGNAL test signal computed onboard.
void play(bool use_signal = false) {
    const PlayParams pp = play_params;
    const uint8_t type = use_signal ? 0x0B : 0x07;
    if (pp.rate < 50 || pp.rate > 800) { ack_from_control(type, -2); return; }
    uint16_t pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS], cmd[N_SERVOS], ref[N_SERVOS];
    if (!read_state(pos, spd, load)) { ack_from_control(type, -4); finish(4, 0, 0, 0, 0, 0); state = ERROR_STATE; return; }
    const sig::Params sp = signal_params;
    float start_deg[N_SERVOS], q_deg[N_SERVOS];
    for (int j = 0; j < N_SERVOS; j++) start_deg[j] = pos_to_deg(j, pos[j]);
    if (use_signal) {
        int err = sig::validate_start(sp, start_deg);
        if (err) { ack_from_control(type, -20 - err); finish(3, 0, 0, 0, 0, 0); return; }
    } else {
        for (int j = 0; j < N_SERVOS; j++) {
            int e = (int)pos[j] - (int)plan[0].ref[j];
            if (abs(e) > pp.start_tol) { ack_from_control(type, -3); finish(3, 0, 0, 0, j + 1, e); return; }
        }
    }
    ack_from_control(type, 0);
    state = PLAYING;
    telem_dropped = 0;

    // Enable motion: hold, no acceleration ramp, speed cap (each write verified)
    uint16_t caps[N_SERVOS]; for (int j = 0; j < N_SERVOS; j++) caps[j] = pp.speed_cap;
    if (!sync_write_u16_verified(REG_GOAL_POSITION, pos) || !sync_write_u8_verified(REG_ACCELERATION, 0) ||
        !sync_write_u16_verified(REG_GOAL_SPEED, caps)) {
        hold_pose();
        finish(4, 0, 0, 0, 0, 0); state = ERROR_STATE; return;
    }

    const uint32_t period = 1000000UL / pp.rate;
    const float duration_s = (use_signal ? sig::total_s(sp) : (plan_n - 1) / (float)plan_rate) + 0.5f;   // plus 0.5 s settling
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
        } else {
            interp(plan, plan_n, t * plan_rate, cmd, ref);
        }
        sync_write_u16(REG_GOAL_POSITION, cmd);
        bool ok = read_state(pos, spd, load);

        Sample& s = ring[ring_head % RING];
        if (ring_head - ring_tail >= RING) { telem_dropped++; }
        else {
            s.t_us = now - t0;
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

    WiFi.mode(WIFI_STA);
    WiFi.setHostname("mycobot-atom");
    WiFi.setSleep(false);                          // lower, steadier latency
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    ArduinoOTA.setHostname("mycobot-atom");
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.onStart([]() { state = OTA; show(40, 0, 40); Bus.end(); });
    ArduinoOTA.begin();
    cmd_udp.begin(CMD_PORT);

    outbox = xQueueCreate(16, sizeof(OutMsg));
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(net_task, "net", 8192, nullptr, 1, nullptr, 0);

    // Power-up: hold the pose (torque on, goal speed 0). Retry until all servos answer.
    delay(300);
    for (int k = 0; k < 20 && !hold_pose(); k++) delay(100);
    for (int k = 0; k < 5 && !gains_ok; k++) { gains_ok = write_gains(); if (!gains_ok) delay(100); }
    uint16_t p[N_SERVOS], s[N_SERVOS], l[N_SERVOS];
    state = read_state(p, s, l) ? HOLDING : ERROR_STATE;
}

void loop() {
    static bool btn_prev = false;
    bool btn = digitalRead(BTN_PIN) == LOW;
    if (btn && !btn_prev && state != PLAYING) request = REQ_HOLD;   // acknowledge / clear error
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
            post(m, sizeof(m));
        } else if (r == REQ_HOLD) {
            bool ok = hold_pose();
            state = ok ? (plan_valid ? READY : HOLDING) : ERROR_STATE;
            ack_from_control(0x03, ok ? 0 : -4);
        } else if (r == REQ_PLAY) {
            play();
        } else if (r == REQ_SIGNAL) {
            play(true);
        } else if (r == REQ_REG) {
            uint8_t id = reg_req[1], addr = reg_req[2], len = reg_req[3];
            if (reg_req[0] == 0x09) {
                uint8_t m[5 + 32] = {0x86, id, addr, len, 0};
                if (!reg_read(id, addr, len, m + 5)) m[4] = (uint8_t)-4;
                post(m, m[4] == 0 ? 5 + len : 5);
            } else {
                int err = reg_write(id, addr, reg_req + 4, len);
                uint8_t back[32];
                int8_t status = err < 0 ? -4 : (!reg_read(id, addr, len, back) || memcmp(back, reg_req + 4, len)) ? -6 : 0;
                uint8_t m[5] = {0x87, id, addr, (uint8_t)status, (uint8_t)(err < 0 ? 0 : err)};
                post(m, sizeof(m));
            }
        }
    }
}
