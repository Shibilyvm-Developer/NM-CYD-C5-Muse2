#ifndef EEG_DSP_H
#define EEG_DSP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EEG_CH_TP9 = 0,
    EEG_CH_AF7,
    EEG_CH_AF8,
    EEG_CH_TP10,
    EEG_CH_COUNT
} eeg_channel_t;

#define EEG_AF7_FEATURE_COUNT 6

typedef struct {
    float rel_theta;
    float rel_alpha;
    float rel_beta;
    float alpha_beta;
    float theta_beta;
    float peak_frequency;
} eeg_af7_features_t;

typedef void (*eeg_af7_feature_callback_t)(
    const eeg_af7_features_t *features
);

void eeg_dsp_set_af7_feature_callback(
    eeg_af7_feature_callback_t callback
);

/*
 * Start the EEG DSP FreeRTOS task.
 */
void eeg_dsp_start(void);

/*
 * Push one Muse EEG notification into the DSP queue.
 *
 * channel:
 *   EEG_CH_TP9
 *   EEG_CH_AF7
 *   EEG_CH_AF8
 *   EEG_CH_TP10
 *
 * data:
 *   Muse 20-byte EEG packet
 */
void eeg_dsp_push_packet(eeg_channel_t channel,
                         const uint8_t *data,
                         size_t len);



/* ============================================================
 * PPG / HEART RATE
 * ============================================================ */

void eeg_dsp_push_ppg(
    int channel,
    const uint32_t *samples,
    size_t count
);


#ifdef __cplusplus
}
#endif

#endif
