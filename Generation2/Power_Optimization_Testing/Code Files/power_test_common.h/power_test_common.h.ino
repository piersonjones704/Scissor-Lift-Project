/**
 * power_test_common.h
 * ---------------------------------------------------------------------------
 * Shared definitions, helpers and logging macros for the VL53L8CX
 * structured power-characterisation test suite.
 *
 * Include this file in every test sketch.
 *
 * Hardware assumptions
 *   - ESP32-S3-DevKitC-1 (WROOM-2)
 *   - VL53L8CX connected via I2C (SDA=GPIO8, SCL=GPIO9)
 *   - Current measurement: external shunt / INA219 on the 3.3 V rail OR
 *     USB power-meter read manually and logged in the table below.
 *
 * Power-logging table template (copy into your spreadsheet for each test)
 * -----------------------------------------------------------------------
 *  Test ID | Parameter changed        | Value | I_min(mA) | I_avg(mA) | I_max(mA) | Notes
 *  T01     | Baseline (default)       | –     |           |           |           |
 *  T02     | Ranging frequency        | 10 Hz |           |           |           |
 *  ...
 * -----------------------------------------------------------------------
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

/* ── I2C pin assignment ─────────────────────────────────────────────────── */
#define SDA_PIN   8
#define SCL_PIN   9
#define I2C_FREQ  100000   /* 400 kHz Fast-mode */

/* ── Serial baud rate ───────────────────────────────────────────────────── */
#define SERIAL_BAUD 115200

/* ── Warm-up period before logging (allows current to stabilise) ─────────── */
#define WARMUP_READINGS  10   /* discard this many readings at startup */

/* ── Number of ranging frames to capture per test window ─────────────────── */
#define CAPTURE_FRAMES   60   /* ~60 seconds at 1 Hz, scale as needed */

/* ── Logging helpers ────────────────────────────────────────────────────── */
#define LOG(msg)         Serial.println(F(msg))
#define LOGF(fmt, ...)   Serial.printf(fmt, __VA_ARGS__)

/**
 * @brief Print a human-readable ranging result summary.
 *        Only the first zone is printed to keep output concise.
 */
static inline void print_result_summary(VL53L8CX_ResultsData *pRes, uint8_t nb_zones)
{
    Serial.printf("[RANGE] zones=%u  z0_dist=%d mm  z0_status=%u  z0_signal=%.1f kcps/spad\n",
        nb_zones,
        pRes->distance_mm[0],
        pRes->target_status[0],
        (float)pRes->signal_per_spad[0] / 2048.0f);
}

/**
 * @brief Block until a valid ranging result is ready.
 *        Returns false on timeout (5 s).
 */
static inline bool wait_for_data(VL53L8CX_Configuration *pDev)
{
    uint8_t ready = 0;
    uint32_t t0 = millis();
    while (!ready) {
        vl53l8cx_check_data_ready(pDev, &ready);
        if (millis() - t0 > 5000) {
            LOG("[ERROR] Timeout waiting for ranging data");
            return false;
        }
        delay(5);
    }
    return true;
}

/**
 * @brief Unified sensor initialisation.
 *        Call at the top of every test's setup().
 * @param pDev   Pointer to a caller-allocated VL53L8CX_Configuration.
 * @return true on success, false on failure.
 */
static inline bool sensor_init(VL53L8CX_Configuration *pDev)
{
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
    Serial.begin(SERIAL_BAUD);
    delay(200);

    /* Populate the platform handle that the ULD needs */
    pDev->platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

    uint8_t isAlive = 0;
    if (vl53l8cx_is_alive(pDev, &isAlive) || !isAlive) {
        LOG("[ERROR] Sensor not detected. Check wiring.");
        return false;
    }
    LOG("[INFO]  Sensor alive");

    if (vl53l8cx_init(pDev)) {
        LOG("[ERROR] vl53l8cx_init() failed");
        return false;
    }
    LOG("[INFO]  Sensor initialised");
    return true;
}