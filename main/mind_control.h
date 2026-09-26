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
#ifndef MIND_CONTROL_H
#define MIND_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "eeg_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIND_FEATURE_COUNT 6
#define MIND_BUTTON_COUNT  4


typedef enum {
    MIND_STATE_IDLE = 0,
    MIND_STATE_BASELINE,
    MIND_STATE_BUTTON_1,
    MIND_STATE_BUTTON_2,
    MIND_STATE_BUTTON_3,
    MIND_STATE_BUTTON_4,
    MIND_STATE_READY
} mind_state_t;

typedef struct {
    mind_state_t state;

    uint16_t captured;
    uint16_t target;

    uint16_t elapsed_ms;
    uint16_t target_ms;

    uint8_t detected_button;

    float distance;
    float confidence;

    bool baseline_valid;
    bool button_valid[MIND_BUTTON_COUNT];
    bool profiles_saved;
} mind_control_status_t;

void mind_control_init(void);

void mind_control_update_features(
    const eeg_af7_features_t *features
);

void mind_control_start_baseline(void);

void mind_control_start_button(uint8_t button);

bool mind_control_save(void);

bool mind_control_clear(void);

void mind_control_get_status(
    mind_control_status_t *status
);

#ifdef __cplusplus
}
#endif

#endif
