/*
 * NM-CYD-C5-Muse2
 *
 * Copyright (c) 2026 Shibily VM. All rights reserved.
 *
 * PUBLIC-v1.0
 * Version 1.0.0
 *
 * Project-specific source code is protected by LICENSE.
 *
 * Third-party libraries, hardware, firmware, trademarks, fonts,
 * graphics, and other external material remain the property of
 * their respective owners.
 */
#include "mind_control.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TAG "MIND"

#define NVS_NAMESPACE "mindctrl"
#define NVS_VERSION   1

#define CAL_DURATION_MS 3000
#define CAL_MIN_SAMPLES 3

typedef struct {
    float mean[MIND_FEATURE_COUNT];
    float m2[MIND_FEATURE_COUNT];
    uint32_t count;
    bool valid;
} profile_accum_t;

typedef struct {
    float mean[MIND_FEATURE_COUNT];
    float stddev[MIND_FEATURE_COUNT];
    uint32_t count;
    bool valid;
} stored_profile_t;

static mind_state_t state = MIND_STATE_IDLE;

static profile_accum_t baseline;
static profile_accum_t buttons[MIND_BUTTON_COUNT];

static stored_profile_t stored_baseline;
static stored_profile_t stored_buttons[MIND_BUTTON_COUNT];

static uint16_t captured = 0;
static int64_t calibration_start_us = 0;

static uint8_t detected_button = 0;
static float last_distance = 0.0f;
static float last_confidence = 0.0f;

/*
 * Require two consecutive frames to agree before reporting
 * a detected button. This prevents one noisy FFT frame from
 * immediately changing the selected control.
 */
static uint8_t candidate_button = 0;
static uint8_t candidate_count = 0;

#define DETECTION_CONFIRM_FRAMES 2

/*
 * A calibration session is considered complete only when a
 * fresh baseline plus fresh Button 1-4 profiles have all been
 * captured. Existing NVS profiles remain usable until SAVE.
 */
static bool current_baseline_valid = false;
static bool current_button_valid[MIND_BUTTON_COUNT];

static bool initialized = false;

static void accum_reset(profile_accum_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void features_to_array(
    const eeg_af7_features_t *f,
    float x[MIND_FEATURE_COUNT])
{
    x[0] = f->rel_theta;
    x[1] = f->rel_alpha;
    x[2] = f->rel_beta;
    x[3] = f->alpha_beta;
    x[4] = f->theta_beta;
    x[5] = f->peak_frequency;
}

/*
 * Convert raw AF7 features into baseline-relative z-scores.
 *
 * The baseline is the user's neutral/reference state.
 * Button profiles are therefore stored in the same normalized
 * coordinate system that is used by the live classifier.
 */
static bool normalize_against_baseline(
    const eeg_af7_features_t *f,
    const stored_profile_t *base,
    float z[MIND_FEATURE_COUNT])
{
    if (f == NULL || base == NULL || !base->valid) {
        return false;
    }

    float x[MIND_FEATURE_COUNT];
    features_to_array(f, x);

    for (int i = 0; i < MIND_FEATURE_COUNT; i++) {
        float sd = base->stddev[i];

        if (!isfinite(sd) || sd < 1e-4f) {
            sd = 1.0f;
        }

        z[i] = (x[i] - base->mean[i]) / sd;

        if (!isfinite(z[i])) {
            return false;
        }
    }

    return true;
}

/*
 * The active baseline accumulator has not necessarily been
 * copied into stored_baseline yet. This helper computes its
 * current mean/stddev without requiring SAVE.
 */
static bool normalize_against_baseline_accum(
    const eeg_af7_features_t *f,
    const profile_accum_t *base,
    float z[MIND_FEATURE_COUNT])
{
    if (f == NULL || base == NULL || !base->valid ||
        base->count < 2) {
        return false;
    }

    float x[MIND_FEATURE_COUNT];
    features_to_array(f, x);

    for (int i = 0; i < MIND_FEATURE_COUNT; i++) {
        float variance =
            base->m2[i] / (float)(base->count - 1);

        if (!isfinite(variance) || variance < 1e-8f) {
            variance = 1e-8f;
        }

        float sd = sqrtf(variance);

        if (!isfinite(sd) || sd < 1e-4f) {
            sd = 1.0f;
        }

        z[i] = (x[i] - base->mean[i]) / sd;

        if (!isfinite(z[i])) {
            return false;
        }
    }

    return true;
}

/*
 * Push an already-normalized feature vector into an accumulator.
 */
static void accum_push_array(
    profile_accum_t *p,
    const float x[MIND_FEATURE_COUNT])
{
    p->count++;

    for (int i = 0; i < MIND_FEATURE_COUNT; i++) {
        float delta = x[i] - p->mean[i];
        p->mean[i] += delta / (float)p->count;

        float delta2 = x[i] - p->mean[i];
        p->m2[i] += delta * delta2;
    }

    if (p->count >= CAL_MIN_SAMPLES) {
        p->valid = true;
    }
}

static void accum_push(
    profile_accum_t *p,
    const eeg_af7_features_t *f)
{
    float x[MIND_FEATURE_COUNT];

    features_to_array(f, x);
    accum_push_array(p, x);
}

static void accum_to_profile(
    const profile_accum_t *src,
    stored_profile_t *dst)
{
    memset(dst, 0, sizeof(*dst));

    dst->count = src->count;
    dst->valid = src->valid;

    for (int i = 0; i < MIND_FEATURE_COUNT; i++) {
        dst->mean[i] = src->mean[i];

        if (src->count > 1) {
            float variance =
                src->m2[i] / (float)(src->count - 1);

            if (variance < 1e-8f) {
                variance = 1e-8f;
            }

            dst->stddev[i] = sqrtf(variance);
        } else {
            dst->stddev[i] = 1.0f;
        }
    }
}

static bool write_profile(
    nvs_handle_t nvs,
    const char *key,
    const stored_profile_t *profile)
{
    return nvs_set_blob(
        nvs,
        key,
        profile,
        sizeof(*profile)
    ) == ESP_OK;
}

static bool read_profile(
    nvs_handle_t nvs,
    const char *key,
    stored_profile_t *profile)
{
    size_t size = sizeof(*profile);

    if (nvs_get_blob(
            nvs,
            key,
            profile,
            &size) != ESP_OK) {
        return false;
    }

    return size == sizeof(*profile) &&
           profile->valid;
}

static void load_profiles(void)
{
    nvs_handle_t nvs;

    if (nvs_open(
            NVS_NAMESPACE,
            NVS_READONLY,
            &nvs) != ESP_OK) {
        return;
    }

    uint32_t version = 0;

    if (nvs_get_u32(
            nvs,
            "version",
            &version) != ESP_OK ||
        version != NVS_VERSION) {
        nvs_close(nvs);
        return;
    }

    read_profile(
        nvs,
        "baseline",
        &stored_baseline
    );

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        char key[16];

        snprintf(
            key,
            sizeof(key),
            "button%d",
            i + 1
        );

        read_profile(
            nvs,
            key,
            &stored_buttons[i]
        );
    }

    nvs_close(nvs);
}

void mind_control_init(void)
{
    memset(&baseline, 0, sizeof(baseline));
    memset(buttons, 0, sizeof(buttons));
    memset(&stored_baseline, 0, sizeof(stored_baseline));
    memset(&stored_buttons, 0, sizeof(stored_buttons));

    state = MIND_STATE_IDLE;
    captured = 0;
    detected_button = 0;
    last_distance = 0.0f;
    last_confidence = 0.0f;

    candidate_button = 0;
    candidate_count = 0;

    current_baseline_valid = false;

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        current_button_valid[i] = false;
    }

    load_profiles();

    initialized = true;

    bool ready = stored_baseline.valid;

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        if (!stored_buttons[i].valid) {
            ready = false;
        }
    }

    if (ready) {
        state = MIND_STATE_READY;
        ESP_LOGI(TAG, "Stored calibration loaded");
    } else {
        ESP_LOGI(TAG, "Calibration required");
    }
}

void mind_control_start_baseline(void)
{
    accum_reset(&baseline);

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        accum_reset(&buttons[i]);
        current_button_valid[i] = false;
    }

    captured = 0;
    calibration_start_us = esp_timer_get_time();
    current_baseline_valid = false;

    candidate_button = 0;
    candidate_count = 0;
    detected_button = 0;
    last_distance = 0.0f;
    last_confidence = 0.0f;

    state = MIND_STATE_BASELINE;

    ESP_LOGI(
        TAG,
        "New calibration session started: baseline"
    );
}

void mind_control_start_button(uint8_t button)
{
    if (button < 1 || button > MIND_BUTTON_COUNT) {
        return;
    }

    if (!current_baseline_valid) {
        ESP_LOGW(
            TAG,
            "Cannot calibrate Button %u: baseline missing",
            button
        );
        return;
    }

    accum_reset(&buttons[button - 1]);
    captured = 0;
    calibration_start_us = esp_timer_get_time();

    current_button_valid[button - 1] = false;

    candidate_button = 0;
    candidate_count = 0;

    state = (mind_state_t)(
        MIND_STATE_BUTTON_1 + button - 1
    );

    ESP_LOGI(
        TAG,
        "Button %u normalized calibration started",
        button
    );
}

static void classify(
    const eeg_af7_features_t *f)
{
    float x[MIND_FEATURE_COUNT];

    if (!normalize_against_baseline(
            f,
            &stored_baseline,
            x)) {
        return;
    }

    float best = 1e30f;
    int best_button = -1;

    for (int b = 0; b < MIND_BUTTON_COUNT; b++) {

        if (!stored_buttons[b].valid) {
            continue;
        }

        float sum = 0.0f;

        for (int i = 0; i < MIND_FEATURE_COUNT; i++) {
            float delta =
                x[i] - stored_buttons[b].mean[i];

            sum += delta * delta;
        }

        float distance = sqrtf(sum);

        if (distance < best) {
            best = distance;
            best_button = b;
        }
    }

    if (best_button < 0) {
        return;
    }

    last_distance = best;

    /*
     * Relative confidence estimate only.
     * This is not a probability or medical metric.
     */
    last_confidence =
        1.0f / (1.0f + best / 4.0f);

    if (last_confidence > 1.0f) {
        last_confidence = 1.0f;
    }

    uint8_t button = (uint8_t)(best_button + 1);

    /*
     * Require consecutive agreement before accepting
     * the button as the active detection.
     */
    if (button == candidate_button) {
        if (candidate_count < DETECTION_CONFIRM_FRAMES) {
            candidate_count++;
        }
    } else {
        candidate_button = button;
        candidate_count = 1;
    }

    if (candidate_count >= DETECTION_CONFIRM_FRAMES) {
        detected_button = button;
    }
}

void mind_control_update_features(
    const eeg_af7_features_t *features)
{
    if (!initialized || features == NULL) {
        return;
    }

    switch (state) {

        case MIND_STATE_BASELINE:
            accum_push(&baseline, features);
            captured = baseline.count;

            bool time_done =
                (esp_timer_get_time() - calibration_start_us) >=
                ((int64_t)CAL_DURATION_MS * 1000);

            if (time_done && captured >= CAL_MIN_SAMPLES) {
                baseline.valid = true;
                /*
                 * Convert the newly captured baseline into the
                 * reference profile immediately. It is not
                 * persisted until the user presses SAVE.
                 */
                accum_to_profile(
                    &baseline,
                    &stored_baseline
                );

                current_baseline_valid = true;

                ESP_LOGI(
                    TAG,
                    "Baseline capture complete"
                );

                state = MIND_STATE_IDLE;
            }

            return;

        case MIND_STATE_BUTTON_1:
        case MIND_STATE_BUTTON_2:
        case MIND_STATE_BUTTON_3:
        case MIND_STATE_BUTTON_4: {
            int button =
                (int)state - (int)MIND_STATE_BUTTON_1;

            float z[MIND_FEATURE_COUNT];

            /*
             * Button samples are stored relative to the freshly
             * captured baseline.
             */
            if (!normalize_against_baseline_accum(
                    features,
                    &baseline,
                    z)) {
                return;
            }

            accum_push_array(
                &buttons[button],
                z
            );

            captured = buttons[button].count;

            bool time_done =
                (esp_timer_get_time() - calibration_start_us) >=
                ((int64_t)CAL_DURATION_MS * 1000);

            if (time_done && captured >= CAL_MIN_SAMPLES) {
                buttons[button].valid = true;
                current_button_valid[button] = true;

                ESP_LOGI(
                    TAG,
                    "Button %d normalized capture complete",
                    button + 1
                );

                state = MIND_STATE_IDLE;
            }

            return;
        }

        case MIND_STATE_READY:
            classify(features);
            return;

        default:
            return;
    }
}

bool mind_control_save(void)
{
    /*
     * SAVE commits one complete, fresh calibration session.
     * Do not silently mix newly captured profiles with old NVS
     * profiles.
     */
    if (!current_baseline_valid || !baseline.valid) {
        ESP_LOGW(
            TAG,
            "Cannot save: fresh baseline missing"
        );
        return false;
    }

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        if (!current_button_valid[i] ||
            !buttons[i].valid) {
            ESP_LOGW(
                TAG,
                "Cannot save: fresh Button %d missing",
                i + 1
            );
            return false;
        }
    }

    /*
     * Baseline is already represented by stored_baseline in RAM.
     * Button accumulators contain baseline-normalized features.
     */
    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        accum_to_profile(
            &buttons[i],
            &stored_buttons[i]
        );
    }

    nvs_handle_t nvs;

    if (nvs_open(
            NVS_NAMESPACE,
            NVS_READWRITE,
            &nvs) != ESP_OK) {
        return false;
    }

    bool ok = true;

    ok &= nvs_set_u32(
        nvs,
        "version",
        NVS_VERSION
    ) == ESP_OK;

    ok &= write_profile(
        nvs,
        "baseline",
        &stored_baseline
    );

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        char key[16];

        snprintf(
            key,
            sizeof(key),
            "button%d",
            i + 1
        );

        ok &= write_profile(
            nvs,
            key,
            &stored_buttons[i]
        );
    }

    if (ok) {
        ok = nvs_commit(nvs) == ESP_OK;
    }

    nvs_close(nvs);

    if (ok) {
        state = MIND_STATE_READY;

        candidate_button = 0;
        candidate_count = 0;
        detected_button = 0;
        last_distance = 0.0f;
        last_confidence = 0.0f;

        ESP_LOGI(
            TAG,
            "Calibration saved to NVS"
        );
    }

    return ok;
}

bool mind_control_clear(void)
{
    nvs_handle_t nvs;

    if (nvs_open(
            NVS_NAMESPACE,
            NVS_READWRITE,
            &nvs) == ESP_OK) {

        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    memset(
        &stored_baseline,
        0,
        sizeof(stored_baseline)
    );

    memset(
        &stored_buttons,
        0,
        sizeof(stored_buttons)
    );

    accum_reset(&baseline);

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        accum_reset(&buttons[i]);
    }

    captured = 0;
    detected_button = 0;
    last_distance = 0.0f;
    last_confidence = 0.0f;

    candidate_button = 0;
    candidate_count = 0;

    current_baseline_valid = false;

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        current_button_valid[i] = false;
    }

    state = MIND_STATE_IDLE;

    ESP_LOGW(TAG, "Calibration cleared");

    return true;
}

void mind_control_get_status(
    mind_control_status_t *status)
{
    if (status == NULL) {
        return;
    }

    memset(status, 0, sizeof(*status));

    status->state = state;

    /*
     * captured/target remain available as sample counters,
     * but calibration progress is now time-based.
     */
    status->captured = captured;
    status->target = CAL_MIN_SAMPLES;

    status->elapsed_ms = 0;
    status->target_ms = CAL_DURATION_MS;

    if (state == MIND_STATE_BASELINE ||
        state == MIND_STATE_BUTTON_1 ||
        state == MIND_STATE_BUTTON_2 ||
        state == MIND_STATE_BUTTON_3 ||
        state == MIND_STATE_BUTTON_4) {

        int64_t elapsed_us =
            esp_timer_get_time() - calibration_start_us;

        if (elapsed_us > 0) {
            uint32_t elapsed_ms =
                (uint32_t)(elapsed_us / 1000);

            if (elapsed_ms > CAL_DURATION_MS) {
                elapsed_ms = CAL_DURATION_MS;
            }

            status->elapsed_ms =
                (uint16_t)elapsed_ms;
        }
    }

    status->detected_button = detected_button;
    status->distance = last_distance;
    status->confidence = last_confidence;
    status->baseline_valid = stored_baseline.valid;

    for (int i = 0; i < MIND_BUTTON_COUNT; i++) {
        status->button_valid[i] =
            stored_buttons[i].valid;
    }

    status->profiles_saved =
        stored_baseline.valid &&
        stored_buttons[0].valid &&
        stored_buttons[1].valid &&
        stored_buttons[2].valid &&
        stored_buttons[3].valid;
}
