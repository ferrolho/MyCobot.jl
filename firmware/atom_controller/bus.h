// Feetech STS servo bus on the ATOM (G19 RX, G22 TX, 1 Mbaud, half-duplex handled by the arm).
// Measured on the robot: a SYNC READ of 6 bytes from 6 servos takes 1.26 ms, and a SYNC WRITE
// followed immediately by a SYNC READ takes 1.54 ms with no dropped writes (no gap needed here,
// unlike the laptop path through the FT232 and the base).
#pragma once
#include <Arduino.h>

#define BUS_RX    19
#define BUS_TX    22
#define BUS_BAUD  1000000
#define N_SERVOS  6

#define REG_ACCELERATION      41
#define REG_GOAL_POSITION     42
#define REG_GOAL_SPEED        46
#define REG_PRESENT_POSITION  56

HardwareSerial Bus(1);

inline uint8_t ft_checksum(const uint8_t* body, int n) {
    uint32_t s = 0;
    for (int i = 0; i < n; i++) s += body[i];
    return ~s & 0xFF;
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
    uint8_t params[2 + N_SERVOS * 3] = {addr, 2};
    for (int j = 0; j < N_SERVOS; j++) {
        params[2 + 3 * j] = j + 1;
        params[3 + 3 * j] = v[j] & 0xFF;
        params[4 + 3 * j] = v[j] >> 8;
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
    for (int a = 0; a < attempts; a++) {
        if (a) write_retries++;
        sync_write_u16(addr, v);
        delayMicroseconds(300);
        uint8_t d[N_SERVOS][16];
        if (sync_read(addr, 2, d) != (1 << N_SERVOS) - 1) continue;
        bool ok = true;
        for (int j = 0; j < N_SERVOS; j++) ok &= u16le(d[j]) == v[j];
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
            pos[j] = u16le(d[j]); spd[j] = u16le(d[j] + 2); load[j] = u16le(d[j] + 4);
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
