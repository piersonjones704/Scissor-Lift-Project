/**
 * T02_frequency_sweep.ino
 * ===========================================================================
 * TEST 02 — RANGING FREQUENCY SWEEP
 * ===========================================================================
 * Variable changed : vl53l8cx_set_ranging_frequency_hz()
 * All other params : identical to T01 baseline
 *
 * API limits (from ULD docs)
 *   4×4 resolution : 1 – 60 Hz
 *   8×8 resolution : 1 – 15 Hz
 *   This test uses 8×8 (MY default) so 1–15 Hz is valid.
 *   The default resolution for the ST's API is 4x4
 *
 * Test sequence
 *   The sketch cycles through TEST_FREQUENCIES automatically.
 *   Each frequency runs for FRAMES_PER_FREQ frames then advances.
 *   Record I_min/I_avg/I_max during the CAPTURE window printed on Serial.
 *
 * Expected power trend
 *   Higher frequency → sensor stays active more of the time → higher average
 *   current.  Expect roughly linear scaling between 1 Hz and ~10 Hz.
 *   Beyond ~10 Hz most of the duty-cycle is already active; gains diminish.
 *
 *   Rough guide (4×4, autonomous mode, 3.3 V rail):
 *     1  Hz  ≈ baseline (T01)
 *     2  Hz  ≈ +5–10 mA vs T01
 *     5  Hz  ≈ +15–25 mA vs T01
 *     10 Hz  ≈ +25–40 mA vs T01
 *
 * Log row per frequency in your power table.
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include <power_test_common.h>

/* ── Test parameters ────────────────────────────────────────────────────── */
/* ── Test parameters ────────────────────────────────────────────────────── */
#define TEST_FREQUENCY_HZ  1   // change this value and reflash for each test
                               // Valid values: 1, 2, 5, 10, 20

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration  Dev;
static VL53L8CX_ResultsData    Results;
static uint8_t  freqIdx   = 0;
static uint32_t frameCount = 0;

/* ── Helpers ────────────────────────────────────────────────────────────── */
static void apply_frequency(uint8_t hz)
{
    vl53l8cx_stop_ranging(&Dev);
    uint8_t status = vl53l8cx_set_ranging_frequency_hz(&Dev, hz);
    if (status) {
        LOGF("[ERROR] set_ranging_frequency_hz(%u) → status %u\n", hz, status);
    }
    frameCount = 0;
    LOGF("\n=== Frequency step: %u Hz ===\n", hz);
    LOG("[INFO]  Settle current meter, then log I_min/I_avg/I_max");
    vl53l8cx_start_ranging(&Dev);
}

/* ── Arduino entry points ───────────────────────────────────────────────── */

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8);
    LOG("=== T02 FREQUENCY SWEEP ===");
    vl53l8cx_set_ranging_frequency_hz(&Dev, TEST_FREQUENCY_HZ);
    vl53l8cx_start_ranging(&Dev);
}

void loop()
{
    if (!wait_for_data(&Dev)) return;
    vl53l8cx_get_ranging_data(&Dev, &Results);
    LOGF("[DATA] freq=%u Hz  ", TEST_FREQUENCY_HZ);
    print_result_summary(&Results, 64);
}