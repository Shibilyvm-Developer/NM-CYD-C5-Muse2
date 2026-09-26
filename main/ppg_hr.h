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
#ifndef PPG_HR_H
#define PPG_HR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float bpm;
    float ibi_ms;
    float quality;
    uint32_t beats;
    uint32_t sample_count;
    uint8_t active_channel;
    uint32_t raw[3];
    bool valid;
} ppg_hr_state_t;

/*
 * Compatibility API used by eeg_display.c.
 * Keep the existing implementation/state structure while
 * exposing the older names expected by the display.
 */
typedef ppg_hr_state_t ppg_hr_metrics_t;

void ppg_hr_get_metrics(ppg_hr_metrics_t *metrics);

void ppg_hr_init(void);
void ppg_hr_push(uint8_t channel, uint32_t sample);
void ppg_hr_get_state(ppg_hr_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
