// MPU6886 on the ATOM Matrix (I2C 0x68, SDA G25, SCL G21), at the end effector.
// Sampled by its own task; the control loop copies the latest sample into telemetry.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define IMU_SDA   25
#define IMU_SCL   21
#define MPU_ADDR  0x68

struct ImuSample {
    int16_t acc[3];    // ±8 g: 4096 LSB/g
    int16_t gyro[3];   // ±2000 °/s: 16.4 LSB/(°/s)
    uint32_t t_us;
};

volatile bool imu_ok = false;
ImuSample imu_latest = {};
portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;

static void imu_write(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}

bool imu_init() {
    Wire.begin(IMU_SDA, IMU_SCL, 400000);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x75); Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDR, 1) != 1 || Wire.read() != 0x19) return false;
    imu_write(0x6B, 0x00); delay(10);    // wake
    imu_write(0x6B, 0x01); delay(10);    // auto clock
    imu_write(0x19, 0x00);               // sample rate divider 0 (1 kHz with DLPF)
    imu_write(0x1A, 0x01);               // gyro DLPF ~176 Hz
    imu_write(0x1D, 0x01);               // accel DLPF ~218 Hz
    imu_write(0x1C, 0x10);               // accel ±8 g
    imu_write(0x1B, 0x18);               // gyro ±2000 °/s
    return true;
}

bool imu_read(ImuSample& s) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
    if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
    uint8_t b[14];
    for (int i = 0; i < 14; i++) b[i] = Wire.read();
    for (int k = 0; k < 3; k++) {
        s.acc[k] = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
        s.gyro[k] = (int16_t)((b[8 + 2 * k] << 8) | b[9 + 2 * k]);
    }
    s.t_us = micros();
    return true;
}

void imu_task(void*) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        ImuSample s;
        if (imu_ok && imu_read(s)) {
            portENTER_CRITICAL(&imu_mux);
            imu_latest = s;
            portEXIT_CRITICAL(&imu_mux);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2));   // ~500 Hz
    }
}

ImuSample imu_get() {
    portENTER_CRITICAL(&imu_mux);
    ImuSample s = imu_latest;
    portEXIT_CRITICAL(&imu_mux);
    return s;
}
