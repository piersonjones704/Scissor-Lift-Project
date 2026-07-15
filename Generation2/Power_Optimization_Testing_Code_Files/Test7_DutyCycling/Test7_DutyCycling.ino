/**
 * T07_duty_cycling.ino
 * ===========================================================================
 * TEST 07 — MANUAL SOFTWARE DUTY CYCLING (sleep between measurements)
 * ===========================================================================
 * Variable changed : on/off ratio of start_ranging / SLEEP power mode
 * All other params : resolution = 4×4, frequency = 10 Hz during active window,
 *                    mode = AUTONOMOUS, integration = 10 ms
 *
 * Concept
 *   Instead of running the sensor continuously at a low frequency, we
 *   burst-range at a higher frequency for a short active window, put the
 *   sensor into SLEEP, then wake it for the next burst.
 *   This can achieve lower average current than continuous 1 Hz, because:
 *     - SLEEP current is much lower than idle WAKEUP current
 *     - The burst captures the same number of samples in less wall-clock time
 *
 * Test steps (run all three, compare average current)
 *   Duty  10 %  → active 1 s,  sleep 9 s   (effective update rate ~1/10 s)
 *   Duty  20 %  → active 2 s,  sleep 8 s
 *   Duty  50 %  → active 5 s,  sleep 5 s
 *
 * Expected power impact
 *   Average I ≈ (I_active × duty) + (I_sleep × (1 − duty))
 *   With I_active ≈ 45 mA and I_sleep ≈ 1 mA:
 *     10 % duty → ~5.4 mA average   (LARGE reduction from T01)
 *     20 % duty → ~9.8 mA average
 *     50 % duty → ~23 mA average
 *
 * Trade-off
 *   Wake-up from SLEEP takes ~1–5 ms (firmware retained).
 *   Confirm that wakeup latency is acceptable for your use-case.
 *
 * Log I_min/I_avg/I_max across a full duty cycle (active + sleep).
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Duty cycle definitions ─────────────────────────────────────────────── */
struct DutyCycle { uint32_t active_ms; uint32_t sleep_ms; };
static const DutyCycle DUTY_STEPS[] = {
    {1000,  9000},   /* 10 % */
    {2000,  8000},   /* 20 % */
    {5000,  5000},   /* 50 % */
};
static const uint8_t NUM_STEPS = sizeof(DUTY_STEPS) / sizeof(DutyCycle);

#define BURST_FREQ_HZ        10
#define BURST_INTEG_MS        5   /* short integration during burst */
#define CYCLES_PER_STEP       5   /* full on/off cycles before advancing */

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

/* ── Helper: run one burst window ──────────────────────────────────────── */
static void burst_window(uint32_t duration_ms)
{
    vl53l8cx_start_ranging(&Dev);
    uint32_t t0 = millis();
    while (millis() - t0 < duration_ms) {
        uint8_t ready = 0;
        vl53l8cx_check_data_ready(&Dev, &ready);
        if (ready) {
            vl53l8cx_get_ranging_data(&Dev, &Results);
            LOGF("[BURST]  ");
            print_result_summary(&Results, 16);
        }
        delay(5);
    }
    vl53l8cx_stop_ranging(&Dev);
}

/* ── Helper: sleep window ───────────────────────────────────────────────── */
static void sleep_window(uint32_t duration_ms)
{
    vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_SLEEP);
    LOGF("[SLEEP]  sleeping %lu ms\n", duration_ms);
    delay(duration_ms);
    vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_WAKEUP);
    delay(5);   /* allow analog circuitry to settle after wake */
}

/* ── Arduino entry points ───────────────────────────────────────────────── */

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    vl53l8cx_set_ranging_frequency_hz(&Dev, BURST_FREQ_HZ);
    vl53l8cx_set_integration_time_ms(&Dev, BURST_INTEG_MS);

    LOG("=== T07 DUTY CYCLING ===");

    for (uint8_t s = 0; s < NUM_STEPS; s++) {
        uint32_t ton  = DUTY_STEPS[s].active_ms;
        uint32_t toff = DUTY_STEPS[s].sleep_ms;
        uint8_t  pct  = (uint8_t)(100 * ton / (ton + toff));

        LOGF("\n--- Duty cycle %u%%  (active=%lu ms, sleep=%lu ms) ---\n",
             pct, ton, toff);
        LOG("[INFO]  Set your power meter to average over full on+off cycle");

        for (uint8_t c = 0; c < CYCLES_PER_STEP; c++) {
            LOGF("[CYCLE] %u/%u\n", c + 1, CYCLES_PER_STEP);
            burst_window(ton);
            sleep_window(toff);
        }
        LOGF("[DONE]  Duty %u%% complete. Record I_min/I_avg/I_max.\n", pct);
    }

    LOG("\n=== T07 COMPLETE ===");
    while (1) delay(1000);
}

void loop() {}