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
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#pragma once

#include "eeg_dsp.h"


/* ============================================================
 * HEART / PPG DASHBOARD
 * ============================================================ */

typedef struct {
    float bpm;
    float ibi_ms;
    float signal_quality;
    float ppg1;
    float ppg2;
    float ppg3;
    bool beat;
    bool valid;
} eeg_heart_metrics_t;

void eeg_display_set_heart_metrics(const eeg_heart_metrics_t *metrics);

void eeg_display_next_page(void);
void eeg_display_previous_page(void);


#ifdef __cplusplus
extern "C" {
#endif

void eeg_display_start(void);

/*
 * Touchscreen gesture interface.
 *
 * Coordinates must already be calibrated to:
 *
 *   X = 0..319
 *   Y = 0..239
 */
typedef enum {
    TOUCH_GESTURE_NONE = 0,
    TOUCH_GESTURE_LEFT,
    TOUCH_GESTURE_RIGHT,
    TOUCH_GESTURE_UP,
    TOUCH_GESTURE_DOWN,
    TOUCH_GESTURE_TAP
} touch_gesture_t;

void eeg_display_touch(uint16_t x, uint16_t y, bool pressed);

void eeg_display_next_page(void);
void eeg_display_previous_page(void);

void eeg_display_push_samples(
    eeg_channel_t channel,
    const float *samples,
    size_t count
);


void eeg_display_update(
    eeg_channel_t channel,
    float delta,
    float theta,
    float alpha,
    float beta,
    float gamma,
    float peak_frequency
);

#ifdef __cplusplus
}
#endif
