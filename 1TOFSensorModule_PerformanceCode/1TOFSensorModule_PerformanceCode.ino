/**
 * AdaptiveProximity_1Sensor.ino
 * ===========================================================================
 * ADAPTIVE PROXIMITY DETECTION — SINGLE VL53L8CX
 * ===========================================================================
 *
 * Behavior:
 *   FAR mode  (no object within 300 cm):
 *     - 4x4 resolution (fixed)
 *     - 1 Hz ranging frequency
 *     - Buzzer silent
 *
 *   NEAR mode (object detected within 300 cm):
 *     - 4x4 resolution (fixed)
 *     - 6 Hz ranging frequency
 *     - Buzzer beeps with proximity-based profiles
 *
 *   Mode switching:
 *     - Upscale to 6 Hz: 2 consecutive readings below 300 cm
 *     - Downscale to 1 Hz: object must be gone for 4 seconds AND
 *       2 consecutive readings above 330 cm (hysteresis)
 *
 *   Buzzer transitions are smoothed — profile steps one level at a time
 *   rather than jumping directly to the target profile.
 *
 * Wiring:
 *   Sensor : SDA=GPIO8, SCL=GPIO9, SPI/I2C=3.3V, LPn=3.3V
 *   Buzzer : positive=3.3V, negative=transistor collector
 *   Transistor base : 1kΩ resistor → GPIO 1
 *
 * Scaling: designed for 1 sensor, architecture supports expansion to 6.
 * ===========================================================================
 */

#include "vl53l8cx_api.h"
#include "power_test_common.h"

/* ── Pin assignment ─────────────────────────────────────────────────────── */
#define BUZZER_PIN   1

/* ── Adaptive frequency thresholds (mm) ─────────────────────────────────── */
#define MODE_ENTER_THRESHOLD   1200   /* switch to 6 Hz when object < 300 cm */
#define MODE_EXIT_THRESHOLD    1300   /* switch to 1 Hz when object > 330 cm */
#define MODE_STABILITY_COUNT   2      /* consecutive readings to confirm     */
#define DOWNSCALE_HOLD_MS      4000   /* hold 6 Hz for 4s after last detect  */

/* ── Frequency settings ─────────────────────────────────────────────────── */
#define FREQ_FAR   5
#define FREQ_NEAR  10

/* ── Detection thresholds for buzzer profiles (mm) ─────────────────────── */
#define DIST_VERY_CLOSE   200    /*  < 10 cm  → very fast beep */
#define DIST_CLOSE        400    /*  < 20 cm → fast beep      */
#define DIST_MEDIUM       600    /*  < 25 cm → medium beep    */
#define DIST_FAR          800    /*  < 32.5 cm → slow beep      */
#define DIST_APPROACH     1000    /*  < 40 cm → very slow beep */
                                  /* >= 40 cm → silent         */

/* ── Beep timing profiles (on_ms, off_ms) ──────────────────────────────── */
struct BeepProfile {
    uint32_t on_ms;
    uint32_t off_ms;
};

static const BeepProfile PROFILES[] = {
    {0,    0},      /* 0: silent      (>300 cm)    */
    {1000, 1500},   /* 1: very slow   (200-300 cm) */
    {800,  1200},   /* 2: slow        (150-200 cm) */
    {500,  500},    /* 3: medium      (100-150 cm) */
    {200,  200},    /* 4: fast        (50-100 cm)  */
    {100,  100},    /* 5: very fast   (<50 cm)     */
};

#define NUM_PROFILES  6

/* ── Sensor mode enumeration ────────────────────────────────────────────── */
enum SensorMode {
    MODE_FAR  = 0,   /* 1 Hz */
    MODE_NEAR = 1    /* 6 Hz */
};

/* ── Globals ────────────────────────────────────────────────────────────── */
static VL53L8CX_Configuration Dev;
static VL53L8CX_ResultsData   Results;

/* ── Sensor mode state ──────────────────────────────────────────────────── */
static SensorMode currentMode        = MODE_FAR;
static uint8_t    modeStabilityCount = 0;
static SensorMode pendingMode        = MODE_FAR;
static uint32_t   lastNearDetection  = 0;

/* ── Buzzer state machine ───────────────────────────────────────────────── */
static uint8_t  currentProfile    = 0;   /* active buzzer profile          */
static uint8_t  targetProfile     = 0;   /* profile we're transitioning to */
static uint8_t  buzzerState       = 0;   /* 0=off, 1=on                   */
static uint32_t lastBuzzerChange  = 0;   /* last toggle timestamp          */
static uint32_t lastProfileStep   = 0;   /* last profile transition time   */

/* ── Smooth transition timing ───────────────────────────────────────────── */
#define PROFILE_STEP_INTERVAL_MS  150    /* ms between profile steps       */

/* ── Forward declarations ───────────────────────────────────────────────── */
static void apply_frequency(SensorMode mode);
static void evaluate_mode_switch(int16_t dist_mm);
static int16_t get_closest_distance();
static uint8_t get_profile(int16_t dist_mm);
static void update_buzzer();
static void update_profile_stepping();

/* ════════════════════════════════════════════════════════════════════════ */
/*  ADAPTIVE FREQUENCY SWITCHING                                           */
/* ════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Stop ranging, change frequency, restart ranging.
 *        Resolution stays fixed at 4x4 — only frequency changes.
 */
static void apply_frequency(SensorMode mode)
{
    vl53l8cx_stop_ranging(&Dev);

    if (mode == MODE_FAR) {
        vl53l8cx_set_ranging_frequency_hz(&Dev, FREQ_FAR);
        LOG("[MODE]  → FAR  (4x4, 1 Hz)");
    } else {
        vl53l8cx_set_ranging_frequency_hz(&Dev, FREQ_NEAR);
        LOG("[MODE]  → NEAR (4x4, 6 Hz)");
    }

    currentMode = mode;
    modeStabilityCount = 0;

    vl53l8cx_start_ranging(&Dev);
    delay(20);
}

/**
 * @brief Evaluate whether frequency should switch based on distance.
 *        Upscale: fast (2 consecutive readings below 300 cm)
 *        Downscale: slow (object gone for DOWNSCALE_HOLD_MS AND
 *                   2 consecutive readings above 330 cm)
 */
static void evaluate_mode_switch(int16_t dist_mm)
{
    SensorMode desired;

    if (currentMode == MODE_FAR) {
        /* Currently in FAR — check if object entered detection zone */
        if (dist_mm > 0 && dist_mm < MODE_ENTER_THRESHOLD) {
            desired = MODE_NEAR;
            lastNearDetection = millis();
        } else {
            /* No object or too far — stay in FAR */
            modeStabilityCount = 0;
            pendingMode = currentMode;
            return;
        }
    } else {
        /* Currently in NEAR */
        if (dist_mm > 0 && dist_mm < MODE_EXIT_THRESHOLD) {
            /* Object still in range — reset hold timer, stay NEAR */
            lastNearDetection = millis();
            modeStabilityCount = 0;
            pendingMode = currentMode;
            return;
        }

        /* Object gone or past exit threshold — check hold time */
        if (millis() - lastNearDetection < DOWNSCALE_HOLD_MS) {
            /* Still within hold window — stay at 6 Hz */
            return;
        }

        desired = MODE_FAR;
    }

    /* Stability counter — require consecutive confirmations */
    if (desired != currentMode) {
        if (desired == pendingMode) {
            modeStabilityCount++;
        } else {
            pendingMode = desired;
            modeStabilityCount = 1;
        }

        if (modeStabilityCount >= MODE_STABILITY_COUNT) {
            apply_frequency(desired);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  DISTANCE MEASUREMENT                                                   */
/* ════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Scan all 16 zones (4x4) and return closest valid target (mm).
 *        Returns 0 if no valid target found.
 */
static int16_t get_closest_distance()
{
    int16_t closest = 0;

    for (uint8_t i = 0; i < 16; i++) {
        uint8_t status = Results.target_status[i];
        int16_t dist   = Results.distance_mm[i];

        /* status 5 = valid, status 6 = valid lower confidence */
        if ((status == 5 || status == 6) && dist > 0) {
            if (closest == 0 || dist < closest) {
                closest = dist;
            }
        }
    }
    return closest;
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  BUZZER LOGIC                                                           */
/* ════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Determine target beep profile based on distance.
 */
static uint8_t get_profile(int16_t dist_mm)
{
    if (dist_mm <= 0 || dist_mm > DIST_APPROACH)   return 0;  /* silent     */
    if (dist_mm > DIST_FAR)                         return 1;  /* very slow  */
    if (dist_mm > DIST_MEDIUM)                      return 2;  /* slow       */
    if (dist_mm > DIST_CLOSE)                       return 3;  /* medium     */
    if (dist_mm > DIST_VERY_CLOSE)                  return 4;  /* fast       */
    return 5;                                                   /* very fast  */
}

/**
 * @brief Step currentProfile toward targetProfile one level at a time.
 *        Called on a timed interval for smooth transitions.
 */
static void update_profile_stepping()
{
    if (currentProfile == targetProfile) return;

    uint32_t now = millis();
    if (now - lastProfileStep < PROFILE_STEP_INTERVAL_MS) return;

    /* Step one profile at a time */
    if (targetProfile > currentProfile) {
        currentProfile++;
    } else {
        currentProfile--;
    }

    lastProfileStep = now;

    /* Reset buzzer timing on profile change for clean transition */
    lastBuzzerChange = now;
    if (currentProfile > 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = 1;
    } else {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = 0;
    }
}

/**
 * @brief Non-blocking buzzer state machine — call every loop iteration.
 */
static void update_buzzer()
{
    /* First, step profile toward target if needed */
    update_profile_stepping();

    /* Silent profile — ensure buzzer is off */
    if (currentProfile == 0) {
        if (buzzerState != 0) {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerState = 0;
        }
        return;
    }

    /* Toggle buzzer based on current profile timing */
    uint32_t elapsed = millis() - lastBuzzerChange;
    uint32_t on_time  = PROFILES[currentProfile].on_ms;
    uint32_t off_time = PROFILES[currentProfile].off_ms;

    if (buzzerState == 1 && elapsed >= on_time) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = 0;
        lastBuzzerChange = millis();
    } else if (buzzerState == 0 && elapsed >= off_time) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = 1;
        lastBuzzerChange = millis();
    }
}

/* ════════════════════════════════════════════════════════════════════════ */
/*  ARDUINO ENTRY POINTS                                                   */
/* ════════════════════════════════════════════════════════════════════════ */

void setup()
{
    if (!sensor_init(&Dev)) { while (1) delay(1000); }

    /* Fixed 4x4 resolution — never changes */
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    delay(10);
    vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    delay(10);
    vl53l8cx_set_ranging_frequency_hz(&Dev, FREQ_FAR);
    delay(10);
    vl53l8cx_set_target_order(&Dev, VL53L8CX_TARGET_ORDER_CLOSEST);
    delay(10);

    /* Buzzer setup */
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    /* Print confirmed configuration */
    uint8_t  res, freq, mode;
    uint32_t integ;
    vl53l8cx_get_resolution(&Dev, &res);
    vl53l8cx_get_ranging_frequency_hz(&Dev, &freq);
    vl53l8cx_get_ranging_mode(&Dev, &mode);
    vl53l8cx_get_integration_time_ms(&Dev, &integ);

    LOG("=== ADAPTIVE PROXIMITY DETECTION ===");
    LOGF("  Resolution  : %u zones (4x4 fixed)\n", res);
    LOGF("  Initial freq: %u Hz\n",                freq);
    LOGF("  Mode        : %u (3=AUTONOMOUS)\n",    mode);
    LOGF("  Integration : %lu ms\n",               integ);
    LOG("  Target order: CLOSEST");
    LOG("");
    LOG("  FAR mode  : 4x4, 1 Hz  (object > 300 cm)");
    LOG("  NEAR mode : 4x4, 6 Hz  (object < 300 cm)");
    LOG("  Hysteresis : 300 cm enter / 330 cm exit");
    LOGF("  Downscale hold: %u ms\n", DOWNSCALE_HOLD_MS);
    LOG("  Buzzer     : smooth profile stepping");
    LOG("==========================================");

    vl53l8cx_start_ranging(&Dev);
    delay(100);
}

void loop()
{
    /* Update buzzer state machine every iteration — non-blocking */
    update_buzzer();

    /* Check if ranging data is ready */
    uint8_t ready = 0;
    vl53l8cx_check_data_ready(&Dev, &ready);

    if (ready) {
        vl53l8cx_get_ranging_data(&Dev, &Results);

        /* Get closest valid target across all 16 zones */
        int16_t closest_mm = get_closest_distance();

        /* Evaluate whether to switch frequency */
        evaluate_mode_switch(closest_mm);

        /* Update target beep profile based on distance */
        targetProfile = get_profile(closest_mm);

        /* Log data */
        const char *profileLabel;
        switch (targetProfile) {
            case 0: profileLabel = "SILENT";    break;
            case 1: profileLabel = "VERYSLOW";  break;
            case 2: profileLabel = "SLOW";      break;
            case 3: profileLabel = "MEDIUM";    break;
            case 4: profileLabel = "FAST";      break;
            case 5: profileLabel = "VERYFAST";  break;
            default: profileLabel = "UNKNOWN";  break;
        }

        LOGF("[DETECT] dist=%d mm  target=%s  active=%u  freq=%s  buzzer=%s\n",
             closest_mm,
             profileLabel,
             currentProfile,
             (currentMode == MODE_FAR) ? "1Hz" : "6Hz",
             buzzerState ? "ON" : "OFF");
    }
}