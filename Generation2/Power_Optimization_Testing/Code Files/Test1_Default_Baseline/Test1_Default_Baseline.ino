/**
 * T01_baseline.ino
Profile picture

 * ===========================================================================
 * TEST 01 — BASELINE (default ULD configuration)
 * ===========================================================================
 * Purpose
 *   Establish the reference current draw with zero modifications.
 *   Every subsequent test is compared against this baseline.
 *
 * What the ULD defaults to after vl53l8cx_init()
 *   - Resolution  : 8×8  (64 zones) ULD Defaults it to 4x4, but for testing I 
 *                   have default at 8x8.
 *      - HOWEVER, I am defaulting to 8x8 (64 zones)
 *   - Frequency   : 1 Hz
 *   - Integration : autonomous maximum for the given frequency
 *   - Ranging mode: CONTINUOUS
 *   - Power mode  : WAKEUP (fully active)
 *   - Sharpener   : 0 %
 *   - Target order: STRONGEST
 *
 * Expected current (3.3 V rail, approximate)
 *   Active ranging at 1 Hz : ~35–50 mA average
 *   (Exact value depends on PCB layout, decoupling, I2C pull-ups.)
 *
 * Log these metrics
 *   I_min, I_avg, I_max over 60 frames (~60 s at 1 Hz)
 *
 * How to use
 *   1. Flash this sketch.
 *   2. Open Serial Monitor (115200 baud).
 *   3. Allow 10 warm-up frames (logged as [WARMUP]).
 *   4. Record I_min / I_avg / I_max from your power meter.
 *   5. Enter values in row T01 of your power table.
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration  Dev;
static VL53L8CX_ResultsData    Results;

/* ── Arduino entry points ───────────────────────────────────────────────── */

void setup()
{
    Serial.begin(115200);
    delay(10000);
    while(!Serial);
    Serial.println("BOOT");
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    /* ------------------------------------------------------------------ */
    /* NO CONFIGURATION CHANGES — pure default after init (BESIDES 8x8 resolution) */
    /* ------------------------------------------------------------------ */
    /* Print active configuration for the log */

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_8X8); // Added to default to 8x8 resolution instead of ST's default of 4x4
    uint8_t  resolution, frequency, mode, sharpener, target_order;
    uint32_t integration_ms;

    vl53l8cx_get_resolution(&Dev, &resolution);
    vl53l8cx_get_ranging_frequency_hz(&Dev, &frequency);
    vl53l8cx_get_integration_time_ms(&Dev, &integration_ms);
    vl53l8cx_get_ranging_mode(&Dev, &mode);
    vl53l8cx_get_sharpener_percent(&Dev, &sharpener);
    vl53l8cx_get_target_order(&Dev, &target_order);

    LOG("=== T01 BASELINE ===");
    LOGF("  Resolution  : %u zones\n",  resolution);
    LOGF("  Frequency   : %u Hz\n",     frequency);
    LOGF("  Integration : %lu ms\n",    integration_ms);
    LOGF("  Ranging mode: %u  (1=CONTINUOUS, 3=AUTONOMOUS)\n", mode);
    LOGF("  Sharpener   : %u %%\n",     sharpener);
    LOGF("  Target order: %u  (1=CLOSEST, 2=STRONGEST)\n",     target_order);
    LOG("====================");
    LOG("[INFO]  Starting ranging…");

    vl53l8cx_start_ranging(&Dev);
    delay(2000);
}

void loop()
{
    static uint32_t frame = 0;

    if (!wait_for_data(&Dev)) return;

    vl53l8cx_get_ranging_data(&Dev, &Results);
    frame++;

    if (frame <= WARMUP_READINGS) {
        LOGF("[WARMUP] frame %lu\n", frame);
    } else {
        LOGF("[DATA]   frame %lu  ", frame);
        print_result_summary(&Results, 64);

        if (frame >= WARMUP_READINGS + CAPTURE_FRAMES) {
            LOG("[INFO]  Capture complete. Record I_min/I_avg/I_max now.");
            vl53l8cx_stop_ranging(&Dev);
            while (1) delay(1000);   /* hold — keep the meter on the rail */
        }
    }
}
