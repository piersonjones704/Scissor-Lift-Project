/**
 * T10_six_sensor_stagger.ino
 * ===========================================================================
 * TEST 10 — 6-SENSOR STAGGERED RANGING ARCHITECTURE (1-sensor scaffold)
 * ===========================================================================
 * Purpose
 *   Simulate the timing and I2C address management strategy that will be
 *   used when all 6 sensors are installed.  Right now only Sensor 0 is
 *   physical; the remaining 5 are placeholders.
 *
 * Architecture
 *   Each sensor gets a unique I2C address assigned at boot by toggling its
 *   XSHUT pin (one sensor woken at a time).  Sensors are then ranged in a
 *   round-robin stagger so only ONE sensor is active at any given moment,
 *   preventing peak current spikes that could brown-out the power rail.
 *
 * I2C address assignment (must be done before any sensor is allowed to
 * respond at the default 0x52 address):
 *   1. Assert all XSHUT pins LOW (all sensors off).
 *   2. For each sensor i:
 *       a. Set XSHUT[i] HIGH (wake sensor i alone).
 *       b. Call vl53l8cx_set_i2c_address(&Dev[i], BASE_ADDR + 2*i).
 *       c. Sensor now responds at its new address.
 *   3. All sensors awake, all at unique addresses.
 *
 * Stagger strategy
 *   With 6 sensors at 1 Hz each we have 1000 ms per frame.
 *   Divide the frame into 6 × 166 ms slots.
 *   In each slot: wake one sensor, take one burst, put it back to SLEEP.
 *   Peak current = I_active (1 sensor) rather than 6 × I_active.
 *
 * Expected peak current impact
 *   Without stagger: 6 × ~45 mA = ~270 mA peak
 *   With stagger   :  1 × ~45 mA = ~45 mA peak (same as single sensor)
 *   Average current: 6 × I_avg_per_sensor (same total energy, lower peak)
 *
 * THIS SKETCH runs with 1 physical sensor (Sensor 0).
 * Uncomment the additional Dev[] entries as sensors are added.
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Pin assignments (XSHUT lines) ─────────────────────────────────────── */
/* Assign one GPIO per sensor.  Adjust for your wiring. */
static const uint8_t XSHUT_PINS[6] = {4, 5, 6, 7, 15, 16};

/* ── I2C address table ──────────────────────────────────────────────────── */
/* Addresses must be even (LSB=0).  Spread them away from 0x52 default. */
#define BASE_I2C_ADDR 0x54   /* 0x54, 0x56, 0x58, 0x5A, 0x5C, 0x5E */
#define NUM_SENSORS   1      /* ← change to 6 when all sensors installed */

/* ── Stagger timing ─────────────────────────────────────────────────────── */
#define FRAME_PERIOD_MS  1000              /* one complete round-robin cycle */
#define SLOT_MS          (FRAME_PERIOD_MS / 6)  /* 166 ms per sensor slot */
#define BURST_MS          50               /* active ranging within each slot */
#define TEST_CYCLES       30

/* ── Sensor array ───────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev[NUM_SENSORS];
static VL53L8CX_ResultsData   Results;

/* ── Helper: configure one sensor ──────────────────────────────────────── */
static bool configure_sensor(uint8_t idx, uint8_t i2c_addr)
{
    Dev[idx].platform.address = i2c_addr;

    uint8_t alive = 0;
    if (vl53l8cx_is_alive(&Dev[idx], &alive) || !alive) {
        LOGF("[ERROR] Sensor %u not alive at 0x%02X\n", idx, i2c_addr);
        return false;
    }
    if (vl53l8cx_init(&Dev[idx])) {
        LOGF("[ERROR] init failed for sensor %u\n", idx);
        return false;
    }

    vl53l8cx_set_resolution(&Dev[idx], VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_mode(&Dev[idx], VL53L8CX_RANGING_MODE_AUTONOMOUS);
    vl53l8cx_set_ranging_frequency_hz(&Dev[idx], 10);
    vl53l8cx_set_integration_time_ms(&Dev[idx], 10);
    vl53l8cx_set_VHV_repeat_count(&Dev[idx], 10);

    LOGF("[INFO]  Sensor %u OK at 0x%02X\n", idx, i2c_addr);
    return true;
}

/* ── Address assignment (sequential XSHUT toggling) ────────────────────── */
static void assign_addresses()
{
    /* All off */
    for (uint8_t i = 0; i < 6; i++) {
        pinMode(XSHUT_PINS[i], OUTPUT);
        digitalWrite(XSHUT_PINS[i], LOW);
    }
    delay(10);

    /* Wake each sensor alone, assign unique address */
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(XSHUT_PINS[i], HIGH);
        delay(10);   /* boot time */

        uint8_t newAddr = BASE_I2C_ADDR + (uint8_t)(2 * i);
        /* Sensor is at 0x52 (default) when first woken */
        Dev[i].platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
        vl53l8cx_set_i2c_address(&Dev[i], newAddr);
        Dev[i].platform.address = newAddr;
        LOGF("[INFO]  Sensor %u assigned 0x%02X\n", i, newAddr);
    }
}

/* ── One stagger slot ───────────────────────────────────────────────────── */
static void range_sensor_slot(uint8_t idx)
{
    /* Wake from sleep */
    vl53l8cx_set_power_mode(&Dev[idx], VL53L8CX_POWER_MODE_WAKEUP);
    delay(3);   /* settling time */

    vl53l8cx_start_ranging(&Dev[idx]);
    uint32_t t0 = millis();

    while (millis() - t0 < BURST_MS) {
        uint8_t ready = 0;
        vl53l8cx_check_data_ready(&Dev[idx], &ready);
        if (ready) {
            vl53l8cx_get_ranging_data(&Dev[idx], &Results);
            LOGF("[S%u]  dist=%d mm  status=%u\n",
                 idx,
                 Results.distance_mm[0],
                 Results.target_status[0]);
        }
        delay(5);
    }

    vl53l8cx_stop_ranging(&Dev[idx]);
    /* Sleep for remainder of slot */
    vl53l8cx_set_power_mode(&Dev[idx], VL53L8CX_POWER_MODE_SLEEP);
}

/* ── Arduino entry points ───────────────────────────────────────────────── */

void setup()
{
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
    Serial.begin(SERIAL_BAUD);
    delay(200);

    LOG("=== T10 6-SENSOR STAGGER SCAFFOLD ===");
    LOGF("[INFO]  NUM_SENSORS = %u\n", NUM_SENSORS);

    assign_addresses();

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (!configure_sensor(i, BASE_I2C_ADDR + 2 * i)) {
            while (1) delay(1000);
        }
        /* Start in sleep; stagger loop will wake them */
        vl53l8cx_set_power_mode(&Dev[i], VL53L8CX_POWER_MODE_SLEEP);
    }

    LOG("[INFO]  All sensors configured. Starting stagger loop.");
    LOG("[INFO]  Meter should average over the full 1000 ms frame period.");

    for (uint8_t c = 0; c < TEST_CYCLES; c++) {
        LOGF("[CYCLE] %u/%u\n", c + 1, TEST_CYCLES);
        uint32_t frame_start = millis();

        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            range_sensor_slot(i);
            /* Pad slot to SLOT_MS regardless of how long the burst took */
            uint32_t elapsed = millis() - frame_start - (uint32_t)(i * SLOT_MS);
            if (elapsed < SLOT_MS) delay(SLOT_MS - elapsed);
        }
        /* Idle for remaining slots (when NUM_SENSORS < 6) */
        uint32_t remaining = FRAME_PERIOD_MS - (millis() - frame_start);
        if (remaining < FRAME_PERIOD_MS) delay(remaining);
    }

    LOG("\n=== T10 COMPLETE ===");
    LOG("[INFO]  Record I_min/I_avg/I_max across the full stagger period.");
    while (1) delay(1000);
}

void loop() {}