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
#include "ppg_hr.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define PPG_FS                  64.0f
#define MIN_IBI_MS              300.0f
#define MAX_IBI_MS             2000.0f
#define MIN_BPM                  30.0f
#define MAX_BPM                 200.0f

#define BASELINE_ALPHA           0.0025f
#define DC_ALPHA                 0.02f
#define ENV_ALPHA                0.02f

#define MIN_PEAK_DISTANCE       ((uint32_t)(PPG_FS * 0.30f))
#define MAX_HISTORY              8

static SemaphoreHandle_t s_mutex;

static ppg_hr_state_t s_state;

static float s_dc[3];
static float s_ac[3];
static float s_env[3];

static float s_prev[3];
static float s_prev2[3];

static uint32_t s_last_peak[3];
static bool s_have_peak[3];

static float s_ibi_history[MAX_HISTORY];
static uint8_t s_ibi_count;
static uint8_t s_ibi_pos;

static uint8_t s_active_channel;

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }

    if (x > hi) {
        return hi;
    }

    return x;
}

void ppg_hr_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }

    if (s_mutex != NULL) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
    }

    memset(&s_state, 0, sizeof(s_state));
    memset(s_dc, 0, sizeof(s_dc));
    memset(s_ac, 0, sizeof(s_ac));
    memset(s_env, 0, sizeof(s_env));
    memset(s_prev, 0, sizeof(s_prev));
    memset(s_prev2, 0, sizeof(s_prev2));
    memset(s_last_peak, 0, sizeof(s_last_peak));
    memset(s_have_peak, 0, sizeof(s_have_peak));
    memset(s_ibi_history, 0, sizeof(s_ibi_history));

    s_ibi_count = 0;
    s_ibi_pos = 0;
    s_active_channel = 0;

    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
}

void ppg_hr_push(uint8_t channel, uint32_t sample)
{
    if (channel >= 3) {
        return;
    }

    if (s_mutex != NULL) {
        if (xSemaphoreTake(s_mutex, 0) != pdTRUE) {
            return;
        }
    }

    s_state.raw[channel] = sample;
    s_state.sample_count++;

    /*
     * Muse PPG channels have different DC levels.
     * Remove the slow DC component first.
     */
    float x = (float)sample;

    if (s_state.sample_count < 10) {
        s_dc[channel] = x;
    } else {
        s_dc[channel] += DC_ALPHA * (x - s_dc[channel]);
    }

    float ac = x - s_dc[channel];

    s_ac[channel] += 0.25f * (ac - s_ac[channel]);

    float env = fabsf(s_ac[channel]);
    s_env[channel] += ENV_ALPHA * (env - s_env[channel]);

    /*
     * Pick the channel with the strongest AC component.
     * This allows the three Muse PPG channels to be used
     * without assuming one fixed optical channel is best.
     */
    if (s_env[channel] > s_env[s_active_channel]) {
        s_active_channel = channel;
    }

    if (channel == s_active_channel) {
        /*
         * Local-maximum detector.
         *
         * s_prev2 < s_prev >= current
         *
         * The threshold is relative to the recent signal
         * envelope rather than an absolute ADC value.
         */
        bool local_peak =
            (s_prev[channel] > s_prev2[channel]) &&
            (s_prev[channel] >= s_ac[channel]);

        float threshold = s_env[channel] * 0.55f;

        uint32_t now = s_state.sample_count;

        if (local_peak &&
            s_prev[channel] > threshold &&
            (!s_have_peak[channel] ||
             now - s_last_peak[channel] >= MIN_PEAK_DISTANCE)) {

            if (s_have_peak[channel]) {
                uint32_t interval_samples =
                    now - s_last_peak[channel];

                float ibi_ms =
                    ((float)interval_samples * 1000.0f) / PPG_FS;

                if (ibi_ms >= MIN_IBI_MS &&
                    ibi_ms <= MAX_IBI_MS) {

                    float bpm = 60000.0f / ibi_ms;

                    if (bpm >= MIN_BPM && bpm <= MAX_BPM) {
                        s_ibi_history[s_ibi_pos] = ibi_ms;

                        s_ibi_pos++;
                        if (s_ibi_pos >= MAX_HISTORY) {
                            s_ibi_pos = 0;
                        }

                        if (s_ibi_count < MAX_HISTORY) {
                            s_ibi_count++;
                        }

                        float sum = 0.0f;

                        for (uint8_t i = 0;
                             i < s_ibi_count;
                             i++) {
                            sum += s_ibi_history[i];
                        }

                        float mean_ibi =
                            sum / (float)s_ibi_count;

                        float mean_bpm =
                            60000.0f / mean_ibi;

                        s_state.ibi_ms = mean_ibi;
                        s_state.bpm = clampf_local(
                            mean_bpm,
                            MIN_BPM,
                            MAX_BPM
                        );

                        s_state.beats++;
                        s_state.valid = true;

                        /*
                         * More accepted intervals means
                         * greater confidence, up to 100%.
                         */
                        s_state.quality =
                            100.0f *
                            ((float)s_ibi_count /
                             (float)MAX_HISTORY);
                    }
                }
            }

            s_last_peak[channel] = now;
            s_have_peak[channel] = true;
        }

        s_prev2[channel] = s_prev[channel];
        s_prev[channel] = s_ac[channel];
    }

    s_state.active_channel = s_active_channel;

    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
}

void ppg_hr_get_state(ppg_hr_state_t *state)
{
    if (state == NULL) {
        return;
    }

    if (s_mutex != NULL) {
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return;
        }
    }

    *state = s_state;

    if (s_mutex != NULL) {
        xSemaphoreGive(s_mutex);
    }
}


void ppg_hr_get_metrics(ppg_hr_metrics_t *metrics)
{
    ppg_hr_get_state(metrics);
}
