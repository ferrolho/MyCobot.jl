// Feetech STS servo bus on the ATOM (G19 RX, G22 TX, 1 Mbaud, half-duplex handled by the arm).
// Measured on the robot: a SYNC READ of 6 bytes from 6 servos takes 1.26 ms, and a SYNC WRITE
// followed immediately by a SYNC READ takes 1.54 ms with no dropped writes (no gap needed here,
// unlike the laptop path through the FT232 and the base).
#pragma once
#include <Arduino.h>
#include "robot_params.h"

#define BUS_RX    19
#define BUS_TX    22
#define BUS_BAUD  1000000
#define N_SERVOS  6

#define REG_MIN_ANGLE          9      // min (9-10) and max (11-12) angle limit
#define REG_PHASE             18
#define REG_TORQUE_ENABLE     40
#define REG_ACCELERATION      41
#define REG_GOAL_POSITION     42
#define REG_GOAL_SPEED        46
#define REG_PRESENT_POSITION  56
#define REG_MAX_TORQUE        16     // EEPROM area: with the lock on, a write lasts to the next power cycle
#define REG_PROTECTION_CURRENT 28    // caps the torque limit (48): seen on the gripper, 2026-10-06
#define REG_TORQUE_LIMIT      48
#define REG_PRESENT_TEMPERATURE 63

HardwareSerial Bus(1);

inline uint8_t ft_checksum(const uint8_t* body, int n) {
    uint32_t s = 0;
    for (int i = 0; i < n; i++) s += body[i];
    return ~s & 0xFF;
}

// Multi-turn joints (robot::MULTI_TURN, firmware 4.5+). The rest of the firmware uses one step value
// per joint: 2048 + sign × angle × 4096 / 360, which goes past 0-4095 beyond ±180°. A multi-turn
// servo reads and takes positions in sign-magnitude (bit 15 = sign), and counts from its one-turn
// reading at power-up, so it can be one turn off: turn_offset = our steps − servo steps (a multiple
// of 4096), set by setup_multi_turn(). REG_READ / REG_WRITE stay raw.
int32_t turn_offset[N_SERVOS] = {0};

inline int32_t servo_signed(uint16_t raw) { return (raw & 0x8000) ? -(int32_t)(raw & 0x7FFF) : raw; }

inline uint16_t pos_from_servo(int j, uint16_t raw) {
    if (!robot::MULTI_TURN[j]) return raw;
    int32_t p = servo_signed(raw) + turn_offset[j];
    return (uint16_t)(p < 0 ? 0 : (p > 0xFFFF ? 0xFFFF : p));
}

inline uint16_t pos_to_servo(int j, uint16_t pos) {
    if (!robot::MULTI_TURN[j]) return pos;
    int32_t s = (int32_t)pos - turn_offset[j];
    return s < 0 ? (uint16_t)(0x8000 | (-s & 0x7FFF)) : (uint16_t)s;
}

// Values as the servos take them: goal positions of multi-turn joints in servo steps.
inline void to_servo(uint8_t addr, const uint16_t v[N_SERVOS], uint16_t out[N_SERVOS]) {
    for (int j = 0; j < N_SERVOS; j++) out[j] = addr == REG_GOAL_POSITION ? pos_to_servo(j, v[j]) : v[j];
}

void bus_begin() {
    Bus.setRxBufferSize(2048);
    Bus.begin(BUS_BAUD, SERIAL_8N1, BUS_RX, BUS_TX);
    Bus.setRxFIFOFull(16);   // hand bytes to the driver early
}

void bus_send(uint8_t id, uint8_t instr, const uint8_t* params, int n) {
    uint8_t pkt[80];
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = id; pkt[3] = n + 2; pkt[4] = instr;
    memcpy(pkt + 5, params, n);
    pkt[5 + n] = ft_checksum(pkt + 2, 3 + n);
    while (Bus.available()) Bus.read();     // drop stale bytes
    Bus.write(pkt, 6 + n);
    Bus.flush();                            // returns once the last byte is on the wire
}

int bus_recv(uint8_t* out, int n, uint32_t timeout_us) {
    int got = 0;
    uint32_t t0 = micros();
    while (got < n && micros() - t0 < timeout_us) {
        int a = Bus.available();
        if (a > 0) got += Bus.read(out + got, min(a, n - got));
    }
    return got;
}

// SYNC READ `len` bytes at `addr` from servos 1..6 into data[j][0..len). Returns a bitmask of
// the servos that replied with a valid packet.
uint8_t sync_read(uint8_t addr, uint8_t len, uint8_t data[N_SERVOS][16]) {
    uint8_t params[2 + N_SERVOS] = {addr, len, 1, 2, 3, 4, 5, 6};
    bus_send(0xFE, 0x82, params, sizeof(params));
    uint8_t rx[N_SERVOS * 22];
    int got = bus_recv(rx, N_SERVOS * (6 + len), 3000);
    uint8_t mask = 0;
    for (int i = 0; i + 6 + len <= got;) {
        if (rx[i] == 0xFF && rx[i + 1] == 0xFF && rx[i + 3] == len + 2) {
            uint8_t id = rx[i + 2];
            if (id >= 1 && id <= N_SERVOS && ft_checksum(rx + i + 2, 3 + len) == rx[i + 5 + len]) {
                memcpy(data[id - 1], rx + i + 5, len);
                mask |= 1 << (id - 1);
            }
            i += 6 + len;
        } else {
            i++;
        }
    }
    return mask;
}

void sync_write_u16(uint8_t addr, const uint16_t v[N_SERVOS]) {
    uint16_t s[N_SERVOS];
    to_servo(addr, v, s);
    uint8_t params[2 + N_SERVOS * 3] = {addr, 2};
    for (int j = 0; j < N_SERVOS; j++) {
        params[2 + 3 * j] = j + 1;
        params[3 + 3 * j] = s[j] & 0xFF;
        params[4 + 3 * j] = s[j] >> 8;
    }
    bus_send(0xFE, 0x83, params, sizeof(params));
}

void sync_write_u8(uint8_t addr, uint8_t v) {
    uint8_t params[2 + N_SERVOS * 2] = {addr, 1};
    for (int j = 0; j < N_SERVOS; j++) { params[2 + 2 * j] = j + 1; params[3 + 2 * j] = v; }
    bus_send(0xFE, 0x83, params, sizeof(params));
}

inline uint16_t u16le(const uint8_t* p) { return p[0] | (p[1] << 8); }

// Setup writes are verified: a SYNC WRITE straight after another SYNC WRITE can be lost by some
// servos (seen 2026-10-04: the speed cap didn't reach J1-J4/J6, so they ignored their goals).
// Write, read the register back, retry. Returns false if it never took.
volatile uint32_t write_retries = 0;

bool sync_write_u16_verified(uint8_t addr, const uint16_t v[N_SERVOS], int attempts = 5) {
    uint16_t s[N_SERVOS];
    to_servo(addr, v, s);
    for (int a = 0; a < attempts; a++) {
        if (a) write_retries++;
        sync_write_u16(addr, v);
        delayMicroseconds(300);
        uint8_t d[N_SERVOS][16];
        if (sync_read(addr, 2, d) != (1 << N_SERVOS) - 1) continue;
        bool ok = true;
        for (int j = 0; j < N_SERVOS; j++) ok &= u16le(d[j]) == s[j];
        if (ok) return true;
    }
    return false;
}

bool sync_write_u8_verified(uint8_t addr, uint8_t v, int attempts = 5) {
    for (int a = 0; a < attempts; a++) {
        if (a) write_retries++;
        sync_write_u8(addr, v);
        delayMicroseconds(300);
        uint8_t d[N_SERVOS][16];
        if (sync_read(addr, 1, d) != (1 << N_SERVOS) - 1) continue;
        bool ok = true;
        for (int j = 0; j < N_SERVOS; j++) ok &= d[j][0] == v;
        if (ok) return true;
    }
    return false;
}

// Present position/speed/load (raw registers) of all servos. Returns true if all replied.
bool read_state(uint16_t pos[N_SERVOS], uint16_t spd[N_SERVOS], uint16_t load[N_SERVOS]) {
    uint8_t d[N_SERVOS][16];
    uint8_t mask = sync_read(REG_PRESENT_POSITION, 6, d);
    for (int j = 0; j < N_SERVOS; j++) {
        if (mask & (1 << j)) {
            pos[j] = pos_from_servo(j, u16le(d[j])); spd[j] = u16le(d[j] + 2); load[j] = u16le(d[j] + 4);
        }
    }
    return mask == (1 << N_SERVOS) - 1;
}

// Goals = present positions and goal speed 0: torque on, nothing can move.
bool hold_pose() {
    uint16_t zero[N_SERVOS] = {0}, pos[N_SERVOS], spd[N_SERVOS], load[N_SERVOS];
    if (!sync_write_u16_verified(REG_GOAL_SPEED, zero)) return false;
    if (!read_state(pos, spd, load)) return false;
    return sync_write_u16_verified(REG_GOAL_POSITION, pos);
}

// READ `len` bytes at `addr` from one servo into `out`. Returns true on a valid reply.
bool reg_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t* out) {
    if (len < 1 || len > 32) return false;
    uint8_t params[2] = {addr, len};
    bus_send(id, 0x02, params, 2);
    uint8_t rx[64];
    int want = 6 + len;
    int got = bus_recv(rx, want, 3000);
    for (int i = 0; i + want <= got; i++) {
        if (rx[i] == 0xFF && rx[i + 1] == 0xFF && rx[i + 2] == id && rx[i + 3] == len + 2 &&
            ft_checksum(rx + i + 2, 3 + len) == rx[i + 5 + len]) {
            memcpy(out, rx + i + 5, len);
            return true;
        }
    }
    return false;
}

// WRITE `n` bytes at `addr` on one servo. Returns the servo's error byte, or -1 without a reply.
int reg_write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n) {
    if (n < 1 || n > 32) return -1;
    uint8_t params[33];
    params[0] = addr;
    memcpy(params + 1, data, n);
    bus_send(id, 0x03, params, n + 1);
    uint8_t rx[16];
    int got = bus_recv(rx, 6, 3000);
    for (int i = 0; i + 6 <= got; i++)
        if (rx[i] == 0xFF && rx[i + 1] == 0xFF && rx[i + 2] == id && rx[i + 3] == 2 && ft_checksum(rx + i + 2, 3) == rx[i + 5])
            return rx[i + 4];
    return -1;
}

// WRITE and read back until the registers hold `data` (or `attempts` run out).
bool reg_write_verified(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t n, int attempts = 5) {
    uint8_t back[32];
    for (int a = 0; a < attempts; a++) {
        reg_write(id, addr, data, n);
        if (reg_read(id, addr, n, back) && memcmp(back, data, n) == 0) return true;
        write_retries++;
    }
    return false;
}

