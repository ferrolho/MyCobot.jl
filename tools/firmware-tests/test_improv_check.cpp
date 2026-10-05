// Tests for firmware/atom_controller/improv.h (Improv WiFi over serial, firmware 4.3+).
// Run by test/test_firmware.jl, or by hand:
//   c++ -std=c++17 -O1 -I firmware/atom_controller tools/firmware-tests/test_improv_check.cpp -o t && ./t
#include <stdio.h>
#include <string>
#include <vector>
#include "improv.h"

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// A packet as the browser sends it (improv-wifi-serial-sdk): checksum over all earlier bytes, then '\n'.
static std::vector<uint8_t> client_packet(uint8_t type, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> p = {'I', 'M', 'P', 'R', 'O', 'V', 1, type, (uint8_t)data.size()};
    p.insert(p.end(), data.begin(), data.end());
    uint8_t s = 0;
    for (uint8_t b : p) s += b;
    p.push_back(s);
    p.push_back('\n');
    return p;
}

static std::vector<uint8_t> wifi_rpc(const std::string& ssid, const std::string& pass) {
    std::vector<uint8_t> d = {improv::WIFI_SETTINGS, 0, (uint8_t)ssid.size()};
    d.insert(d.end(), ssid.begin(), ssid.end());
    d.push_back((uint8_t)pass.size());
    d.insert(d.end(), pass.begin(), pass.end());
    d[1] = (uint8_t)(d.size() - 2);
    return d;
}

// Feeds bytes; returns the number of packets found and keeps the last one.
static int feed_all(improv::Parser& p, const std::vector<uint8_t>& bytes, std::vector<uint8_t>* last = nullptr, int* bad = nullptr) {
    int found = 0;
    for (uint8_t b : bytes) {
        auto r = p.feed(b);
        if (r == improv::Parser::PACKET) {
            found++;
            if (last) last->assign(p.data, p.data + p.len);
        } else if (r == improv::Parser::BAD_CHECKSUM && bad) (*bad)++;
    }
    return found;
}

int main() {
    // 1. Parse a WiFi-settings RPC, also after noise, log lines and a false start of the header.
    {
        improv::Parser p;
        auto pkt = client_packet(improv::RPC, wifi_rpc("MyWirelessAP", "mysecurepassword"));
        std::vector<uint8_t> stream = {'b', 'o', 'o', 't', '\n', 'I', 'M', 'P', 'R', 'I', 'M', 'P', 0xFE, 'I', 'I'};
        stream.insert(stream.end(), pkt.begin(), pkt.end());
        std::vector<uint8_t> data;
        CHECK(feed_all(p, stream, &data) == 1, "one packet in a noisy stream");
        CHECK(p.type == improv::RPC, "type RPC");
        improv::WifiSettings ws;
        CHECK(improv::rpc_command(data.data(), data.size()) == improv::WIFI_SETTINGS, "command 1");
        CHECK(improv::parse_wifi_settings(data.data(), data.size(), ws), "settings parse");
        CHECK(std::string(ws.ssid) == "MyWirelessAP" && std::string(ws.password) == "mysecurepassword", "ssid and password");
        // the spec's example: 01 1E 0C {MyWirelessAP} 10 {mysecurepassword}
        CHECK(data[0] == 0x01 && data[1] == 0x1E && data[2] == 0x0C && data[15] == 0x10, "matches the spec example");
    }
    // 2. Two packets back to back, then one with a bad checksum, then a good one.
    {
        improv::Parser p;
        auto a = client_packet(improv::RPC, {improv::GET_STATE, 0});
        auto b = client_packet(improv::RPC, {improv::GET_INFO, 0});
        auto c = client_packet(improv::RPC, {improv::GET_NETWORKS, 0});
        c[c.size() - 2] ^= 0x55;   // corrupt the checksum
        std::vector<uint8_t> s;
        for (auto* v : {&a, &b, &c, &a}) s.insert(s.end(), v->begin(), v->end());
        int bad = 0;
        CHECK(feed_all(p, s, nullptr, &bad) == 3, "three good packets");
        CHECK(bad == 1, "one bad checksum");
    }
    // 3. A wrong version is not a packet.
    {
        improv::Parser p;
        auto a = client_packet(improv::RPC, {improv::GET_STATE, 0});
        a[6] = 2;
        CHECK(feed_all(p, a) == 0, "version 2 skipped");
    }
    // 4. Malformed WiFi settings are refused.
    {
        improv::WifiSettings ws;
        auto ok = wifi_rpc("net", "");
        CHECK(improv::parse_wifi_settings(ok.data(), ok.size(), ws) && ws.password[0] == 0, "open network (no password)");
        auto empty = wifi_rpc("", "x");
        CHECK(!improv::parse_wifi_settings(empty.data(), empty.size(), ws), "empty SSID refused");
        auto longssid = wifi_rpc(std::string(33, 'a'), "x");
        CHECK(!improv::parse_wifi_settings(longssid.data(), longssid.size(), ws), "33-byte SSID refused");
        auto maxssid = wifi_rpc(std::string(32, 'a'), std::string(64, 'b'));
        CHECK(improv::parse_wifi_settings(maxssid.data(), maxssid.size(), ws) && strlen(ws.ssid) == 32 && strlen(ws.password) == 64, "32-byte SSID, 64-byte key");
        auto longpass = wifi_rpc("net", std::string(65, 'b'));
        CHECK(!improv::parse_wifi_settings(longpass.data(), longpass.size(), ws), "65-byte password refused");
        auto badlen = ok; badlen[1]++;
        CHECK(!improv::parse_wifi_settings(badlen.data(), badlen.size(), ws), "wrong RPC length refused");
        auto trunc = wifi_rpc("network", "password"); trunc.resize(trunc.size() - 3); trunc[1] = (uint8_t)(trunc.size() - 2);
        CHECK(!improv::parse_wifi_settings(trunc.data(), trunc.size(), ws), "truncated password refused");
        auto nul = wifi_rpc(std::string("ne\0t", 4), "x");
        CHECK(!improv::parse_wifi_settings(nul.data(), nul.size(), ws), "NUL inside the SSID refused");
        uint8_t one[1] = {1};
        CHECK(improv::rpc_command(one, 1) == -1, "one-byte RPC invalid");
    }
    // 5. Packets the device sends parse back (round trip), with the checksum and '\n'.
    {
        uint8_t out[improv::MAX_PACKET];
        const char* info[] = {"myCobot 280 controller", "4.3.0", "ESP32", "mycobot"};
        size_t n = improv::build_result(improv::GET_INFO, info, 4, out);
        CHECK(n > 0 && out[n - 1] == '\n', "result ends with a newline");
        improv::Parser p;
        std::vector<uint8_t> data;
        CHECK(feed_all(p, std::vector<uint8_t>(out, out + n), &data) == 1 && p.type == improv::RPC_RESULT, "result parses");
        CHECK(data[0] == improv::GET_INFO && data[1] == data.size() - 2, "result header");
        size_t k = 2; int strings = 0; bool same = true;
        while (k < data.size()) { size_t l = data[k]; same &= std::string((char*)&data[k + 1], l) == info[strings]; k += 1 + l; strings++; }
        CHECK(strings == 4 && same && k == data.size(), "four strings back");
        CHECK(improv::build_result(improv::GET_NETWORKS, nullptr, 0, out) == 11 + 2 && out[9] == improv::GET_NETWORKS && out[10] == 0, "empty result ends a scan");
        n = improv::build_state(improv::PROVISIONED, out);
        CHECK(n == 12 && out[7] == improv::CURRENT_STATE && out[9] == improv::PROVISIONED, "state packet");
        CHECK(improv::checksum(out, 10) == out[10], "state checksum");
        std::string big(300, 'x'); const char* bigs[] = {big.c_str()};
        CHECK(improv::build_result(improv::GET_STATE, bigs, 1, out) == 0, "too long a result refused");
    }
    // 6. WiFi source at power-up.
    {
        CHECK(improv::wifi_source("home", "lab") == improv::SAVED, "saved network first");
        CHECK(improv::wifi_source("", "lab") == improv::COMPILED, "then the compiled one");
        CHECK(improv::wifi_source("", nullptr) == improv::NONE_SAVED && improv::wifi_source(nullptr, "") == improv::NONE_SAVED, "else none");
    }
    // 7. Fuzz: random bytes never crash the parser, and a packet after them still parses.
    {
        improv::Parser p;
        uint32_t x = 12345;
        std::vector<uint8_t> s;
        for (int i = 0; i < 200000; i++) { x = x * 1664525u + 1013904223u; s.push_back((uint8_t)(x >> 24)); }
        feed_all(p, s);
        auto a = client_packet(improv::RPC, {improv::GET_STATE, 0});
        // the random tail may hold a partial packet: a line break and a full packet resynchronise
        std::vector<uint8_t> t(300, '\n');
        t.insert(t.end(), a.begin(), a.end());
        std::vector<uint8_t> data;
        CHECK(feed_all(p, t, &data) >= 1 && data.size() == 2 && data[0] == improv::GET_STATE, "parses after random bytes");
    }
    printf(failures ? "%d FAILURES\n" : "all checks passed\n", failures);
    return failures ? 1 : 0;
}
