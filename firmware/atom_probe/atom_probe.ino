// atom_probe.ino — passive servo-bus probe for the myCobot 280 ATOM (M5Stack ATOM Matrix, ESP32-PICO-D4).
//
// Purpose: confirm which ATOM pins reach the Feetech servo bus, and that WiFi, OTA updates,
// the IMU, the button and the LED matrix all work, before writing the real controller.
//
// It NEVER commands servos 1–6. It only listens, and replies when addressed as ID 7:
//   PING (0x01)               -> empty status packet
//   READ (0x02) addr 0, n ≤ 32 -> probe counters (see `counters` below, little-endian)
//
// LED matrix rows (dim):
//   0  WiFi      blue = connecting, green = connected, red = no credentials/failed
//   1  bus bytes white while bytes arrive on BUS_RX
//   2  packets   green = valid Feetech packets seen in the last second, red = only bad checksums
//   3  ID 7      yellow flash when we answer a request
//   4  IMU       green = MPU6886 found (WHO_AM_I 0x19), red = not found; whole matrix blue while the button is held
//
// Status is broadcast once a second over UDP to port 5005. OTA hostname: mycobot-atom.
//
// WiFi credentials come from wifi_secrets.h, kept OUTSIDE the repository:
//   ~/.config/mycobot/wifi_secrets.h   with   #define WIFI_SSID "..."  /  #define WIFI_PASSWORD "..."
//   (optional) #define OTA_PASSWORD "..."
// Build with: arduino-cli compile --fqbn esp32:esp32:m5stack_atom \
//   --build-property "compiler.cpp.extra_flags=-I$HOME/.config/mycobot" firmware/atom_probe

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#include "wifi_secrets.h"

// ---- Pins (ATOM Matrix) --------------------------------------------------------------
#define LED_PIN   27   // 5x5 SK6812 matrix
#define BTN_PIN   39   // button under the matrix (active low)
#define BUS_RX    19   // servo bus, per github.com/lewpar/myCobot280 (to be confirmed here)
#define BUS_TX    22
#define IMU_SDA   25   // MPU6886
#define IMU_SCL   21

#define PROBE_VERSION 3
#define BUS_BAUD  1000000
#define OUR_ID    7
#define LOG_PORT  5005
#define CMD_PORT  5006   // UDP commands: "bench"
#define MPU_ADDR  0x68

HardwareSerial Bus(1);
Adafruit_NeoPixel matrix(25, LED_PIN, NEO_GRB + NEO_KHZ800);
WiFiUDP udp;
WiFiUDP cmd_udp;

// ---- Counters (also readable over the bus as ID 7, READ addr 0) -----------------------
struct __attribute__((packed)) Counters {
    uint32_t bytes_rx;
    uint32_t packets_valid;
    uint32_t packets_bad_checksum;
    uint32_t requests_to_us;
    uint16_t packets_by_id[8];   // ids 0..6, and [7] = everything else (e.g. broadcast 0xFE)
} counters = {};

uint32_t last_byte_ms = 0, last_valid_ms = 0, last_bad_ms = 0, last_reply_ms = 0;
bool imu_ok = false;
int16_t imu_raw[7] = {0};   // ax ay az temp gx gy gz

// ---- Feetech helpers -----------------------------------------------------------------------
uint8_t checksum(const uint8_t* body, int n) {
    uint32_t s = 0;
    for (int i = 0; i < n; i++) s += body[i];
    return ~s & 0xFF;
}

void reply(const uint8_t* data, int n) {
    uint8_t pkt[48];
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = OUR_ID; pkt[3] = n + 2; pkt[4] = 0x00;   // error = 0
    memcpy(pkt + 5, data, n);
    pkt[5 + n] = checksum(pkt + 2, 3 + n);
    Bus.write(pkt, 6 + n);
    Bus.flush();
    last_reply_ms = millis();
}

void handle_packet(const uint8_t* p, int total) {
    uint8_t id = p[2], len = p[3], instr = p[4];
    counters.packets_valid++;
    counters.packets_by_id[id < 7 ? id : 7]++;
    last_valid_ms = millis();
    if (id != OUR_ID) return;
    if (instr == 0x01 && len == 2) {                        // PING
        counters.requests_to_us++;
        reply(nullptr, 0);
    } else if (instr == 0x02 && len == 4 && p[5] == 0) {    // READ addr 0, n bytes
        counters.requests_to_us++;
        uint8_t n = min<uint8_t>(p[6], 32);
        reply(reinterpret_cast<const uint8_t*>(&counters), min<int>(n, sizeof(counters)));
    }
    // Our own replies (instr byte = error 0x00) echo back here and are ignored.
}

// Byte-by-byte parser: FF FF ID LEN (LEN bytes ...) CHK
uint8_t buf[80];
int pos = 0;

void feed(uint8_t b) {
    counters.bytes_rx++;
    last_byte_ms = millis();
    if (pos < 2) {
        if (b == 0xFF) buf[pos++] = b; else pos = 0;
        return;
    }
    if (pos == 2 && b == 0xFF) return;                      // tolerate FF FF FF ...
    buf[pos++] = b;
    if (pos >= 4) {
        int total = buf[3] + 4;
        if (buf[3] < 2 || total > (int)sizeof(buf)) { pos = 0; return; }
        if (pos == total) {
            if (checksum(buf + 2, total - 3) == buf[total - 1]) handle_packet(buf, total);
            else { counters.packets_bad_checksum++; last_bad_ms = millis(); }
            pos = 0;
        }
    }
}

// ---- IMU (MPU6886) ---------------------------------------------------------------------------
void imu_write(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}

bool imu_init() {
    Wire.begin(IMU_SDA, IMU_SCL, 400000);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDR, 1) != 1 || Wire.read() != 0x19) return false;
    imu_write(0x6B, 0x00); delay(10);    // wake
    imu_write(0x6B, 0x01); delay(10);    // auto clock
    imu_write(0x1C, 0x10);               // accel ±8 g
    imu_write(0x1B, 0x18);               // gyro ±2000 °/s
    return true;
}

void imu_read() {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDR, 14) != 14) return;
    for (int i = 0; i < 7; i++) imu_raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());
}

// ---- LED matrix --------------------------------------------------------------------------------
void row(int r, uint32_t c) { for (int x = 0; x < 5; x++) matrix.setPixelColor(r * 5 + x, c); }

void show_status() {
    uint32_t now = millis(), off = 0;
    if (digitalRead(BTN_PIN) == LOW) {
        for (int i = 0; i < 25; i++) matrix.setPixelColor(i, matrix.Color(0, 0, 60));
        matrix.show();
        return;
    }
    wl_status_t ws = WiFi.status();
    row(0, ws == WL_CONNECTED ? matrix.Color(0, 40, 0) : (strlen(WIFI_SSID) ? matrix.Color(0, 0, 40) : matrix.Color(40, 0, 0)));
    row(1, now - last_byte_ms < 200 ? matrix.Color(30, 30, 30) : off);
    row(2, now - last_valid_ms < 1000 ? matrix.Color(0, 40, 0) : (now - last_bad_ms < 1000 ? matrix.Color(40, 0, 0) : off));
    row(3, now - last_reply_ms < 300 ? matrix.Color(40, 30, 0) : off);
    row(4, imu_ok ? matrix.Color(0, 40, 0) : matrix.Color(40, 0, 0));
    matrix.show();
}

// ---- Bus benchmark (triggered by UDP "bench") -------------------------------------------
// Never moves anything: goal speed is set to 0 on all servos first (and checked), so new
// goals are accepted but not followed. Writing goals turns torque on (the arm stiffens).

void broadcast(const char* line) {
    Serial.println(line);
    if (WiFi.status() != WL_CONNECTED) return;
    udp.beginPacket(IPAddress(255, 255, 255, 255), LOG_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
    udp.endPacket();
}

void bus_send(uint8_t id, uint8_t instr, const uint8_t* params, int n) {
    uint8_t pkt[64];
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = id; pkt[3] = n + 2; pkt[4] = instr;
    memcpy(pkt + 5, params, n);
    pkt[5 + n] = checksum(pkt + 2, 3 + n);
    while (Bus.available()) Bus.read();     // discard stale bytes
    Bus.write(pkt, 6 + n);
    Bus.flush();                            // returns once the last byte is on the wire
}

// Read exactly n bytes or give up after timeout_us; returns bytes read
int bus_recv(uint8_t* out, int n, uint32_t timeout_us) {
    int got = 0;
    uint32_t t0 = micros();
    while (got < n && micros() - t0 < timeout_us) {
        int a = Bus.available();
        if (a > 0) got += Bus.read(out + got, min(a, n - got));
    }
    return got;
}

// SYNC READ `len` bytes at `addr` from servos 1..6; fills data[id-1][0..len); returns servos that replied
int sync_read(uint8_t addr, uint8_t len, uint8_t data[6][16]) {
    uint8_t params[8] = {addr, len, 1, 2, 3, 4, 5, 6};
    bus_send(0xFE, 0x82, params, 8);
    uint8_t rx[6 * 22];
    int want = 6 * (6 + len);
    int got = bus_recv(rx, want, 3000);
    int ok = 0;
    for (int i = 0; i + 6 + len <= got; ) {
        if (rx[i] == 0xFF && rx[i + 1] == 0xFF && rx[i + 3] == len + 2) {
            uint8_t id = rx[i + 2];
            if (id >= 1 && id <= 6 && checksum(rx + i + 2, 3 + len) == rx[i + 5 + len]) {
                memcpy(data[id - 1], rx + i + 5, len); ok++;
            }
            i += 6 + len;
        } else i++;
    }
    return ok;
}

void sync_write_u16(uint8_t addr, const uint16_t v[6]) {
    uint8_t params[2 + 6 * 3] = {addr, 2};
    for (int j = 0; j < 6; j++) { params[2 + 3 * j] = j + 1; params[3 + 3 * j] = v[j] & 0xFF; params[4 + 3 * j] = v[j] >> 8; }
    bus_send(0xFE, 0x83, params, sizeof(params));
}

void run_bench() {
    char line[300];
    uint8_t d[6][16];
    broadcast("bench: start");

    // Safety: goal speed 0 everywhere, and verify
    uint16_t zero[6] = {0, 0, 0, 0, 0, 0};
    sync_write_u16(46, zero); delay(5);
    if (sync_read(46, 2, d) != 6) { broadcast("bench: ABORT, not all servos replied"); return; }
    for (int j = 0; j < 6; j++) if (d[j][0] | d[j][1]) { broadcast("bench: ABORT, goal speed not 0"); return; }

    // 1) state reads (6 bytes: position, speed, load)
    const int N = 2000;
    int fails = 0; uint32_t tmin = 1e9, tmax = 0, t_all = micros();
    for (int k = 0; k < N; k++) {
        uint32_t t = micros();
        if (sync_read(56, 6, d) != 6) fails++;
        t = micros() - t; tmin = min(tmin, t); tmax = max(tmax, t);
    }
    t_all = micros() - t_all;
    snprintf(line, sizeof(line), "bench: state read x%d: mean %.3f ms (min %.3f, max %.3f), failures %d",
             N, t_all / 1000.0 / N, tmin / 1000.0, tmax / 1000.0, fails);
    broadcast(line);

    // 2) write + gap + state read cycles; verify every goal write by reading the goals back
    if (sync_read(56, 2, d) != 6) { broadcast("bench: ABORT, no positions"); return; }
    uint16_t base[6];
    for (int j = 0; j < 6; j++) base[j] = d[j][0] | (d[j][1] << 8);
    const uint32_t gaps[] = {0, 100, 200, 300, 500, 1000};
    for (uint32_t gap : gaps) {
        const int M = 1000;
        int drops = 0, rfail = 0; uint32_t cycle_sum = 0;
        for (int k = 0; k < M; k++) {
            uint16_t v[6];
            for (int j = 0; j < 6; j++) v[j] = base[j] + (k & 1);
            uint32_t t = micros();
            sync_write_u16(42, v);
            uint32_t tg = micros(); while (micros() - tg < gap) {}
            if (sync_read(56, 6, d) != 6) rfail++;
            cycle_sum += micros() - t;
            if (sync_read(42, 2, d) != 6) { drops++; continue; }    // verification (not timed)
            for (int j = 0; j < 6; j++) if ((d[j][0] | (d[j][1] << 8)) != v[j]) { drops++; break; }
        }
        snprintf(line, sizeof(line), "bench: gap %4lu us: write+read cycle %.3f ms (~%d Hz), dropped writes %d/%d, read failures %d",
                 (unsigned long)gap, cycle_sum / 1000.0 / M, (int)(1e6 * M / cycle_sum), drops, M, rfail);
        broadcast(line);
    }
    sync_write_u16(42, base);   // goals back to where the joints are
    broadcast("bench: done (goal speed 0, goals = present)");
}

// ---- Setup / loop ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(BTN_PIN, INPUT);
    matrix.begin();
    matrix.setBrightness(20);            // keep the 25 LEDs (and the ATOM) cool
    matrix.clear(); matrix.show();

    Bus.setRxBufferSize(2048);
    Bus.begin(BUS_BAUD, SERIAL_8N1, BUS_RX, BUS_TX);

    imu_ok = imu_init();

    WiFi.mode(WIFI_STA);
    WiFi.setHostname("mycobot-atom");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    ArduinoOTA.setHostname("mycobot-atom");
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
    ArduinoOTA.onStart([]() { Bus.end(); });   // stay off the bus while flashing
    ArduinoOTA.begin();
    cmd_udp.begin(CMD_PORT);

    Serial.printf("atom_probe: bus RX=%d TX=%d @ %d, IMU %s\n", BUS_RX, BUS_TX, BUS_BAUD, imu_ok ? "ok" : "not found");
}

void loop() {
    while (Bus.available()) feed(Bus.read());
    ArduinoOTA.handle();

    if (cmd_udp.parsePacket()) {
        char cmd[32] = {0};
        cmd_udp.read(cmd, sizeof(cmd) - 1);
        if (strncmp(cmd, "bench", 5) == 0) run_bench();
    }

    static uint32_t last_ui = 0, last_log = 0, last_imu = 0;
    uint32_t now = millis();
    if (imu_ok && now - last_imu >= 10) { last_imu = now; imu_read(); }
    if (now - last_ui >= 50) { last_ui = now; show_status(); }
    if (now - last_log >= 1000) {
        last_log = now;
        char line[320];
        snprintf(line, sizeof(line),
                 "atom_probe v%d ip=%s rssi=%d bus_rx=%lu valid=%lu bad=%lu to_us=%lu by_id=%u,%u,%u,%u,%u,%u,%u,%u "
                 "imu=%d acc=%d,%d,%d gyro=%d,%d,%d btn=%d up=%lus",
                 PROBE_VERSION, WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                 (unsigned long)counters.bytes_rx, (unsigned long)counters.packets_valid,
                 (unsigned long)counters.packets_bad_checksum, (unsigned long)counters.requests_to_us,
                 counters.packets_by_id[0], counters.packets_by_id[1], counters.packets_by_id[2], counters.packets_by_id[3],
                 counters.packets_by_id[4], counters.packets_by_id[5], counters.packets_by_id[6], counters.packets_by_id[7],
                 imu_ok, imu_raw[0], imu_raw[1], imu_raw[2], imu_raw[4], imu_raw[5], imu_raw[6],
                 digitalRead(BTN_PIN) == LOW, (unsigned long)(now / 1000));
        Serial.println(line);
        if (WiFi.status() == WL_CONNECTED) {
            udp.beginPacket(IPAddress(255, 255, 255, 255), LOG_PORT);
            udp.write(reinterpret_cast<const uint8_t*>(line), strlen(line));
            udp.endPacket();
        }
    }
}
