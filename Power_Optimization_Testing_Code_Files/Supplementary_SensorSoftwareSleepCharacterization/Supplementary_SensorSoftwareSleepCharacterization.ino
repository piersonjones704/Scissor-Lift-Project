/**
 * T06_power_modes.ino
 * ===========================================================================
 * TEST 06 — SENSOR POWER MODES: WAKEUP vs SLEEP vs DEEP_SLEEP
 * ===========================================================================
 * Variable changed : vl53l8cx_set_power_mode()
 * All other params : N/A (ranging is stopped during sleep phases)
 *
 * Power mode definitions (from vl53l8cx_api.h)
 *   WAKEUP     (1) : Normal operating mode.
 *   SLEEP      (0) : Retains firmware + configuration. Fast wake-up.
 *                    Recommended for inter-measurement gaps < ~10 s.
 *   DEEP_SLEEP (2) : Clears all memory. Very low Iq.
 *                    Requires full vl53l8cx_init() on wake-up.
 *                    Use when sleeping > 10 s.
 *
 * Test procedure
 *   Phase A: Sensor ranging (1 Hz, 4×4, AUTONOMOUS 10 ms) — note I_active
 *   Phase B: stop_ranging() + WAKEUP (idle, not sleeping) — note I_idle
 *   Phase C: set_power_mode(SLEEP)      — note I_sleep
 *   Phase D: set_power_mode(DEEP_SLEEP) — note I_deep_sleep
 *   Phase E: wake from SLEEP, resume ranging — verify recovery time
 *
 * Expected currents (3.3 V rail, approximate)
 *   Active ranging    : ~35–50 mA
 *   Stopped / WAKEUP  : ~5–10 mA  (clocks still running)
 *   SLEEP             : ~0.5–2 mA
 *   DEEP_SLEEP        : < 0.1 mA  (near leakage)
 *
 * IMPORTANT: After DEEP_SLEEP you MUST call vl53l8cx_init() again.
 *            Firmware and config are lost.
 *
 * Log four rows: T06a (active), T06b (stopped), T06c (sleep), T06d (deep sleep)
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Test parameters ────────────────────────────────────────────────────── */
#define ACTIVE_FRAMES        30
#define HOLD_DURATION_MS  15000   /* 15 s to settle meter in each idle phase */

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

/* ── Helper: blocking hold with countdown ───────────────────────────────── */
static void hold_and_log(const char *label, uint32_t ms)
{
    LOGF("[HOLD]  %s — measuring for %lu ms\n", label, ms);
    uint32_t t0 = millis();
    uint32_t tick = 0;
    while (millis() - t0 < ms) {
        if (millis() - t0 > tick * 3000) {
            LOGF("[HOLD]  %s — %lu s elapsed\n", label, tick * 3);
            tick++;
        }
        delay(100);
    }
    LOGF("[DONE]  %s phase complete. Record I_min/I_avg/I_max now.\n\n", label);
}

/* ── Arduino entry points ───────────────────────────────────────────────── */

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 1);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    vl53l8cx_set_integration_time_ms(&Dev, 10);

    LOG("=== T06 POWER MODES ===");

    /* ── Phase A: Active ranging ── */
    LOG("\n[PHASE A] Active ranging (autonomous, 1 Hz, 10 ms integ)");
    vl53l8cx_start_ranging(&Dev);
    for (uint32_t f = 0; f < ACTIVE_FRAMES; f++) {
        wait_for_data(&Dev);
        vl53l8cx_get_ranging_data(&Dev, &Results);
        LOGF("[DATA]   frame %lu  ", f + 1);
        print_result_summary(&Results, 16);
    }
    vl53l8cx_stop_ranging(&Dev);
    LOG("[INFO]  stop_ranging() called. Record I_active first, then I_stopped.");
    hold_and_log("T06b-STOPPED/WAKEUP", HOLD_DURATION_MS);

    /* ── Phase C: SLEEP ── */
    LOG("[PHASE C] Entering SLEEP mode");
    uint8_t status = vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_SLEEP);
    if (status) LOGF("[WARN]  set_power_mode(SLEEP) → status %u\n", status);
    hold_and_log("T06c-SLEEP", HOLD_DURATION_MS);

    /* Wake from sleep — firmware + config retained */
    LOG("[INFO]  Waking from SLEEP…");
    vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_WAKEUP);
    LOG("[INFO]  Wake from SLEEP complete. Verifying ranging…");
    vl53l8cx_start_ranging(&Dev);
    for (uint32_t f = 0; f < 5; f++) {
        wait_for_data(&Dev);
        vl53l8cx_get_ranging_data(&Dev, &Results);
        LOGF("[VERIFY] frame %lu  ", f + 1);
        print_result_summary(&Results, 16);
    }
    vl53l8cx_stop_ranging(&Dev);

    /* ── Phase D: DEEP_SLEEP ── */
    LOG("\n[PHASE D] Entering DEEP_SLEEP mode (firmware will be erased)");
    status = vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_DEEP_SLEEP);
    if (status) LOGF("[WARN]  set_power_mode(DEEP_SLEEP) → status %u\n", status);
    hold_and_log("T06d-DEEP_SLEEP", HOLD_DURATION_MS);

    /* Wake from DEEP_SLEEP — must re-init */
    LOG("[INFO]  Waking from DEEP_SLEEP — re-initialising sensor…");
    vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_WAKEUP);
    delay(10);
    if (vl53l8cx_init(&Dev)) {
        LOG("[ERROR] Re-init after DEEP_SLEEP failed!");
    } else {
        LOG("[INFO]  Re-init OK. Sensor fully operational.");
    }

    LOG("\n=== T06 COMPLETE ===");
    while (1) delay(1000);
}

void loop() {}