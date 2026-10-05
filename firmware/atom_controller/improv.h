// improv.h — Improv WiFi over serial (https://www.improv-wifi.com/serial/), firmware 4.3+.
// The browser installer on the Setup page (ESP Web Tools) uses it to send the WiFi network and
// password over USB. Plain C++ without Arduino calls, so tools/firmware-tests/test_improv_check.cpp
// can test it on the host.
//
// Packet: "IMPROV", u8 version (1), u8 type, u8 length, data[length], u8 checksum (sum of all
// earlier bytes, mod 256). The device ends each packet with '\n' (the browser skips other lines).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace improv {

const uint8_t VERSION = 1;
const size_t MAX_DATA = 255;
const size_t MAX_PACKET = 9 + MAX_DATA + 2;   // header, version, type, length, data, checksum, '\n'

enum Type : uint8_t { CURRENT_STATE = 0x01, ERROR_STATE = 0x02, RPC = 0x03, RPC_RESULT = 0x04 };
enum State : uint8_t { STOPPED = 0x00, READY = 0x02, PROVISIONING = 0x03, PROVISIONED = 0x04 };
enum Error : uint8_t { NO_ERROR = 0x00, INVALID_RPC = 0x01, UNKNOWN_RPC = 0x02, UNABLE_TO_CONNECT = 0x03, UNKNOWN = 0xFF };
enum Command : uint8_t { WIFI_SETTINGS = 0x01, GET_STATE = 0x02, GET_INFO = 0x03, GET_NETWORKS = 0x04 };

const size_t MAX_SSID = 32, MAX_PASSWORD = 64;   // 802.11 SSID; WPA2 passphrase (63) or hex key (64)

inline uint8_t checksum(const uint8_t* p, size_t n) {
    uint8_t s = 0;
    while (n--) s += *p++;
    return s;
}

// Reads a byte stream and finds Improv packets. Other bytes (log lines, noise) are skipped.
struct Parser {
    uint8_t buf[9 + MAX_DATA + 1];
    size_t n = 0;
    uint8_t type = 0, len = 0;
    const uint8_t* data = nullptr;   // valid after feed() returns PACKET, until the next feed()

    enum Result { NONE, PACKET, BAD_CHECKSUM };

    Result feed(uint8_t b) {
        static const char HDR[] = "IMPROV";
        if (n < 6) {
            if (b == (uint8_t)HDR[n]) buf[n++] = b;
            else n = b == 'I' ? (buf[0] = b, 1) : 0;   // a new header can start at this byte
            return NONE;
        }
        if (n == 6 && b != VERSION) { n = b == 'I' ? (buf[0] = b, 1) : 0; return NONE; }
        buf[n++] = b;
        if (n < 9) return NONE;
        size_t total = 9 + (size_t)buf[8] + 1;
        if (n < total) return NONE;
        n = 0;
        if (checksum(buf, total - 1) != buf[total - 1]) return BAD_CHECKSUM;
        type = buf[7]; len = buf[8]; data = buf + 9;
        return PACKET;
    }
};

struct WifiSettings { char ssid[MAX_SSID + 1]; char password[MAX_PASSWORD + 1]; };

// Checks an RPC packet's frame: data[0] = command, data[1] = length of the rest.
// Returns the command, or -1 if the frame is invalid.
inline int rpc_command(const uint8_t* d, size_t len) {
    if (len < 2 || (size_t)d[1] != len - 2) return -1;
    return d[0];
}

// WIFI_SETTINGS: u8 ssid_len, ssid, u8 password_len, password. Returns false if malformed.
inline bool parse_wifi_settings(const uint8_t* d, size_t len, WifiSettings& out) {
    if (rpc_command(d, len) != WIFI_SETTINGS) return false;
    const uint8_t* p = d + 2;
    size_t rest = len - 2;
    if (rest < 1) return false;
    size_t sl = p[0];
    if (sl < 1 || sl > MAX_SSID || rest < 1 + sl + 1) return false;
    size_t pl = p[1 + sl];
    if (pl > MAX_PASSWORD || rest != 1 + sl + 1 + pl) return false;
    memcpy(out.ssid, p + 1, sl); out.ssid[sl] = 0;
    memcpy(out.password, p + 2 + sl, pl); out.password[pl] = 0;
    if (strlen(out.ssid) != sl || strlen(out.password) != pl) return false;   // no NUL bytes inside
    return true;
}

// Writes a packet (with checksum and '\n') to out[MAX_PACKET]. Returns its size, 0 if too long.
inline size_t build(uint8_t type, const uint8_t* data, size_t len, uint8_t* out) {
    if (len > MAX_DATA) return 0;
    memcpy(out, "IMPROV", 6);
    out[6] = VERSION; out[7] = type; out[8] = (uint8_t)len;
    if (len) memcpy(out + 9, data, len);
    out[9 + len] = checksum(out, 9 + len);
    out[10 + len] = '\n';
    return 11 + len;
}

inline size_t build_state(uint8_t state, uint8_t* out) { return build(CURRENT_STATE, &state, 1, out); }
inline size_t build_error(uint8_t error, uint8_t* out) { return build(ERROR_STATE, &error, 1, out); }

// RPC result: u8 command, u8 length, then each string as u8 length + bytes. n = 0 gives the empty
// result that ends a list of scanned networks. Returns the packet size, 0 if it does not fit.
inline size_t build_result(uint8_t command, const char* const* strings, int n, uint8_t* out) {
    uint8_t d[MAX_DATA];
    size_t k = 2;
    for (int i = 0; i < n; i++) {
        size_t sl = strlen(strings[i]);
        if (sl > 255 || k + 1 + sl > MAX_DATA) return 0;
        d[k++] = (uint8_t)sl;
        memcpy(d + k, strings[i], sl); k += sl;
    }
    d[0] = command; d[1] = (uint8_t)(k - 2);
    return build(RPC_RESULT, d, k, out);
}

// Which WiFi network to join at power-up: the one saved over Improv first, then the one compiled
// in (lab builds with wifi_secrets.h), else none (wait for Improv).
enum Source { NONE_SAVED = 0, SAVED = 1, COMPILED = 2 };
inline Source wifi_source(const char* saved_ssid, const char* compiled_ssid) {
    if (saved_ssid && saved_ssid[0]) return SAVED;
    if (compiled_ssid && compiled_ssid[0]) return COMPILED;
    return NONE_SAVED;
}

}  // namespace improv
