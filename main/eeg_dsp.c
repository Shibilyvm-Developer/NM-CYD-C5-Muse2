/*
 * NM-CYD-C5-Muse2
 *
 * Copyright (c) 2026 Shibily VM. All rights reserved.
 *
 * PUBLIC-v1.0
 * Version 1.0.0
 *
 * Project-specific DSP implementation.
 *
 * Third-party components retain their respective copyrights and licenses.
 */
#include "eeg_dsp.h"
#include "eeg_display.h"
#include "mind_control.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

/* ============================================================
 * Muse 2 EEG configuration
 * ============================================================ */

#define TAG "MUSE_DSP"

#define EEG_FS                  256.0f
#define EEG_FFT_SIZE            256
#define EEG_SAMPLES_PER_PACKET  12
#define EEG_PACKET_SIZE         20

#define EEG_QUEUE_LENGTH        32
#define EEG_TASK_STACK          8192
#define EEG_TASK_PRIORITY       4

#define EEG_PI                  3.14159265358979323846f

/*
 * Muse classic 12-bit EEG conversion:
 *
 * uV = 0.48828125 * (raw - 2048)
 */
#define MUSE_EEG_SCALE          0.48828125f
#define MUSE_EEG_OFFSET         2048.0f

/* ============================================================
 * FFT structures
 * ============================================================ */

typedef struct {
    float re;
    float im;
} fft_complex_t;

typedef struct {
    uint8_t channel;
    uint8_t len;
    uint8_t data[EEG_PACKET_SIZE];
} eeg_packet_t;

/* ============================================================
 * Band result
 * ============================================================ */

typedef struct {
    float delta;
    float theta;
    float alpha;
    float beta;
    float gamma;

    float total;

    float rel_delta;
    float rel_theta;
    float rel_alpha;
    float rel_beta;
    float rel_gamma;

    float alpha_beta;
    float theta_beta;

    float peak_frequency;
    float peak_psd;
} eeg_band_result_t;

/* ============================================================
 * Globals
 * ============================================================ */

static QueueHandle_t eeg_queue = NULL;

static float eeg_ring[EEG_CH_COUNT][EEG_FFT_SIZE];

static uint16_t eeg_write_index[EEG_CH_COUNT];
static uint16_t eeg_filled[EEG_CH_COUNT];
static uint16_t eeg_since_analysis[EEG_CH_COUNT];

static bool eeg_analysis_ready[EEG_CH_COUNT];

static fft_complex_t fft_buffer[EEG_FFT_SIZE];

static float hann_window[EEG_FFT_SIZE];
static float hann_window_power = 0.0f;

static eeg_af7_feature_callback_t af7_feature_callback = NULL;

void eeg_dsp_set_af7_feature_callback(
    eeg_af7_feature_callback_t callback)
{
    af7_feature_callback = callback;
}

static const char *channel_names[EEG_CH_COUNT] = {
    "TP9",
    "AF7",
    "AF8",
    "TP10"
};

/* ============================================================
 * Muse packet decoder
 *
 * Packet:
 *
 * byte 0-1   : sequence number
 * byte 2-19  : 18 bytes
 *
 * 18 bytes = 12 x 12-bit samples
 *
 * sample0 = (b0 << 4) | (b1 >> 4)
 * sample1 = ((b1 & 0x0F) << 8) | b2
 * ============================================================ */

static bool muse_decode_eeg(const uint8_t *packet,
                            size_t len,
                            float samples[EEG_SAMPLES_PER_PACKET])
{
    if (packet == NULL || samples == NULL) {
        return false;
    }

    if (len < EEG_PACKET_SIZE) {
        return false;
    }

    for (int i = 0; i < 6; i++) {

        int j = 2 + (i * 3);

        uint16_t raw0 =
            ((uint16_t)packet[j] << 4) |
            ((uint16_t)packet[j + 1] >> 4);

        uint16_t raw1 =
            (((uint16_t)packet[j + 1] & 0x0F) << 8) |
            packet[j + 2];

        samples[(i * 2)] =
            MUSE_EEG_SCALE *
            ((float)raw0 - MUSE_EEG_OFFSET);

        samples[(i * 2) + 1] =
            MUSE_EEG_SCALE *
            ((float)raw1 - MUSE_EEG_OFFSET);
    }

    return true;
}

/* ============================================================
 * Add samples to channel ring buffer
 * ============================================================ */

static void eeg_add_sample(eeg_channel_t channel, float sample)
{
    if (channel >= EEG_CH_COUNT) {
        return;
    }

    eeg_ring[channel][eeg_write_index[channel]] = sample;

    eeg_write_index[channel]++;

    if (eeg_write_index[channel] >= EEG_FFT_SIZE) {
        eeg_write_index[channel] = 0;
    }

    if (eeg_filled[channel] < EEG_FFT_SIZE) {

        eeg_filled[channel]++;

        /*
         * First complete 1-second window.
         */
        if (eeg_filled[channel] >= EEG_FFT_SIZE) {
            eeg_analysis_ready[channel] = true;
            eeg_since_analysis[channel] = 0;
        }

    } else {

        eeg_since_analysis[channel]++;

        /*
         * New complete 1-second window available.
         */
        if (eeg_since_analysis[channel] >= EEG_FFT_SIZE) {

            eeg_since_analysis[channel] = 0;

            eeg_analysis_ready[channel] = true;
        }
    }
}

/* ============================================================
 * FFT
 * ============================================================ */

static void fft_compute(fft_complex_t *x, int n)
{
    int j = 0;

    /*
     * Bit reversal.
     */
    for (int i = 1; i < n; i++) {

        int bit = n >> 1;

        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }

        j ^= bit;

        if (i < j) {

            fft_complex_t temp = x[i];

            x[i] = x[j];
            x[j] = temp;
        }
    }

    /*
     * Cooley-Tukey radix-2 FFT.
     */
    for (int len = 2; len <= n; len <<= 1) {

        float angle =
            -2.0f * EEG_PI / (float)len;

        float wlen_re = cosf(angle);
        float wlen_im = sinf(angle);

        for (int i = 0; i < n; i += len) {

            float w_re = 1.0f;
            float w_im = 0.0f;

            for (int j = 0; j < len / 2; j++) {

                int u = i + j;
                int v = i + j + len / 2;

                float v_re =
                    x[v].re * w_re -
                    x[v].im * w_im;

                float v_im =
                    x[v].re * w_im +
                    x[v].im * w_re;

                float u_re = x[u].re;
                float u_im = x[u].im;

                x[u].re = u_re + v_re;
                x[u].im = u_im + v_im;

                x[v].re = u_re - v_re;
                x[v].im = u_im - v_im;

                float next_w_re =
                    w_re * wlen_re -
                    w_im * wlen_im;

                float next_w_im =
                    w_re * wlen_im +
                    w_im * wlen_re;

                w_re = next_w_re;
                w_im = next_w_im;
            }
        }
    }
}

/* ============================================================
 * PSD calculation
 *
 * Hann window + one-sided PSD.
 *
 * Result is approximately uV^2/Hz.
 * ============================================================ */

static float calculate_psd(int bin)
{
    if (bin < 0 || bin >= EEG_FFT_SIZE / 2) {
        return 0.0f;
    }

    float magnitude_squared =
        fft_buffer[bin].re * fft_buffer[bin].re +
        fft_buffer[bin].im * fft_buffer[bin].im;

    if (hann_window_power <= 0.0f) {
        return 0.0f;
    }

    float psd =
        magnitude_squared /
        (EEG_FS * hann_window_power);

    /*
     * One-sided PSD.
     *
     * Double bins except DC and Nyquist.
     */
    if (bin > 0 && bin < EEG_FFT_SIZE / 2) {
        psd *= 2.0f;
    }

    return psd;
}

/* ============================================================
 * Calculate EEG frequency bands
 *
 * Delta : 1-4 Hz
 * Theta : 4-8 Hz
 * Alpha : 8-13 Hz
 * Beta  : 13-30 Hz
 * Gamma : 30-45 Hz
 *
 * FFT resolution:
 *
 * Fs/N = 256/256 = 1 Hz
 * ============================================================ */

static void calculate_bands(eeg_band_result_t *result)
{
    memset(result, 0, sizeof(*result));

    float peak_psd = 0.0f;
    int peak_bin = 0;

    /*
     * Only analyse 1-45 Hz.
     */
    for (int k = 1; k <= 45; k++) {

        float frequency = (float)k;

        float psd = calculate_psd(k);

        /*
         * Band definitions.
         */
        if (frequency >= 1.0f &&
            frequency < 4.0f) {

            result->delta += psd;
        }

        else if (frequency >= 4.0f &&
                 frequency < 8.0f) {

            result->theta += psd;
        }

        else if (frequency >= 8.0f &&
                 frequency < 13.0f) {

            result->alpha += psd;
        }

        else if (frequency >= 13.0f &&
                 frequency < 30.0f) {

            result->beta += psd;
        }

        else if (frequency >= 30.0f &&
                 frequency <= 45.0f) {

            result->gamma += psd;
        }

        if (psd > peak_psd) {

            peak_psd = psd;
            peak_bin = k;
        }
    }

    result->total =
        result->delta +
        result->theta +
        result->alpha +
        result->beta +
        result->gamma;

    if (result->total > 0.000001f) {

        result->rel_delta =
            100.0f * result->delta / result->total;

        result->rel_theta =
            100.0f * result->theta / result->total;

        result->rel_alpha =
            100.0f * result->alpha / result->total;

        result->rel_beta =
            100.0f * result->beta / result->total;

        result->rel_gamma =
            100.0f * result->gamma / result->total;
    }

    if (result->beta > 0.000001f) {

        result->alpha_beta =
            result->alpha / result->beta;

        result->theta_beta =
            result->theta / result->beta;
    }

    result->peak_frequency =
        (float)peak_bin;

    result->peak_psd =
        peak_psd;
}

/* ============================================================
 * Analyse one EEG channel
 * ============================================================ */

static void analyse_channel(eeg_channel_t channel)
{
    if (channel >= EEG_CH_COUNT) {
        return;
    }

    if (eeg_filled[channel] < EEG_FFT_SIZE) {
        return;
    }

    /*
     * write_index points to the oldest sample because
     * the buffer is full.
     */
    uint16_t start = eeg_write_index[channel];

    float mean = 0.0f;

    /*
     * Calculate DC mean.
     */
    for (int i = 0; i < EEG_FFT_SIZE; i++) {

        int index =
            (start + i) % EEG_FFT_SIZE;

        mean += eeg_ring[channel][index];
    }

    mean /= (float)EEG_FFT_SIZE;

    /*
     * Apply DC removal + Hann window.
     */
    for (int i = 0; i < EEG_FFT_SIZE; i++) {

        int index =
            (start + i) % EEG_FFT_SIZE;

        float sample =
            eeg_ring[channel][index] - mean;

        fft_buffer[i].re =
            sample * hann_window[i];

        fft_buffer[i].im = 0.0f;
    }

    /*
     * FFT.
     */
    fft_compute(fft_buffer, EEG_FFT_SIZE);

    /*
     * Calculate bands.
     */
    eeg_band_result_t result;

    calculate_bands(&result);

    /*
     * Publish the latest EEG analysis to the LCD task.
     * BLE callback remains free of FFT/LCD work.
     */
    eeg_display_update(
        channel,
        result.rel_delta,
        result.rel_theta,
        result.rel_alpha,
        result.rel_beta,
        result.rel_gamma,
        result.peak_frequency
    );

    /*
     * Publish AF7 features to the mind-control classifier.
     *
     * This runs in the EEG DSP task, never in the NimBLE
     * notification callback.
     */
    if (channel == EEG_CH_AF7 &&
        af7_feature_callback != NULL) {

        eeg_af7_features_t features = {
            .rel_theta = result.rel_theta,
            .rel_alpha = result.rel_alpha,
            .rel_beta = result.rel_beta,
            .alpha_beta = result.alpha_beta,
            .theta_beta = result.theta_beta,
            .peak_frequency = result.peak_frequency
        };

        af7_feature_callback(&features);
    }

    /*
     * Output.
     */
    ESP_LOGI(
        TAG,
        "%s | "
        "D=%.3f "
        "T=%.3f "
        "A=%.3f "
        "B=%.3f "
        "G=%.3f uV^2 | "
        "REL "
        "D=%.1f%% "
        "T=%.1f%% "
        "A=%.1f%% "
        "B=%.1f%% "
        "G=%.1f%% | "
        "A/B=%.3f "
        "T/B=%.3f "
        "PEAK=%.1fHz",
        channel_names[channel],

        result.delta,
        result.theta,
        result.alpha,
        result.beta,
        result.gamma,

        result.rel_delta,
        result.rel_theta,
        result.rel_alpha,
        result.rel_beta,
        result.rel_gamma,

        result.alpha_beta,
        result.theta_beta,

        result.peak_frequency
    );
}

/* ============================================================
 * DSP initialization
 * ============================================================ */

static void eeg_dsp_init(void)
{
    hann_window_power = 0.0f;

    for (int i = 0; i < EEG_FFT_SIZE; i++) {

        float w =
            0.5f *
            (
                1.0f -
                cosf(
                    (2.0f * EEG_PI * i) /
                    (EEG_FFT_SIZE - 1)
                )
            );

        hann_window[i] = w;

        hann_window_power +=
            w * w;
    }

    memset(eeg_ring, 0, sizeof(eeg_ring));
    memset(eeg_write_index, 0, sizeof(eeg_write_index));
    memset(eeg_filled, 0, sizeof(eeg_filled));
    memset(eeg_since_analysis, 0, sizeof(eeg_since_analysis));
    memset(eeg_analysis_ready, 0, sizeof(eeg_analysis_ready));
}

/* ============================================================
 * DSP task
 * ============================================================ */

static void eeg_dsp_task(void *arg)
{
    (void)arg;

    eeg_dsp_init();

    eeg_packet_t packet;

    ESP_LOGI(
        TAG,
        "Muse EEG DSP started | Fs=256Hz | FFT=256 | Resolution=1Hz"
    );

    ESP_LOGI(
        TAG,
        "Bands: Delta 1-4 | Theta 4-8 | Alpha 8-13 | Beta 13-30 | Gamma 30-45"
    );

    while (1) {

        if (xQueueReceive(
                eeg_queue,
                &packet,
                portMAX_DELAY) == pdTRUE) {

            if (packet.channel >= EEG_CH_COUNT) {
                continue;
            }

            float samples[EEG_SAMPLES_PER_PACKET];

            if (!muse_decode_eeg(
                    packet.data,
                    packet.len,
                    samples)) {

                continue;
            }

            /*
             * Feed the decoded Muse samples to the live
             * waveform display.
             *
             * This is deliberately outside the NimBLE callback.
             */
            eeg_display_push_samples(
                (eeg_channel_t)packet.channel,
                samples,
                EEG_SAMPLES_PER_PACKET
            );

            /*
             * Add all 12 decoded samples.
             */
            for (int i = 0;
                 i < EEG_SAMPLES_PER_PACKET;
                 i++) {

                eeg_add_sample(
                    (eeg_channel_t)packet.channel,
                    samples[i]
                );
            }

            /*
             * Analyse any channel which has a
             * complete new 1-second window.
             */
            for (int ch = 0;
                 ch < EEG_CH_COUNT;
                 ch++) {

                if (eeg_analysis_ready[ch]) {

                    eeg_analysis_ready[ch] = false;

                    analyse_channel(
                        (eeg_channel_t)ch
                    );
                }
            }
        }
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

void eeg_dsp_start(void)
{
    if (eeg_queue != NULL) {
        return;
    }

    eeg_queue =
        xQueueCreate(
            EEG_QUEUE_LENGTH,
            sizeof(eeg_packet_t)
        );

    if (eeg_queue == NULL) {

        ESP_LOGE(
            TAG,
            "Failed to create EEG queue"
        );

        return;
    }

    BaseType_t result =
        xTaskCreate(
            eeg_dsp_task,
            "eeg_dsp",
            EEG_TASK_STACK,
            NULL,
            EEG_TASK_PRIORITY,
            NULL
        );

    if (result != pdPASS) {

        ESP_LOGE(
            TAG,
            "Failed to create EEG DSP task"
        );

        vQueueDelete(eeg_queue);

        eeg_queue = NULL;

        return;
    }
}

void eeg_dsp_push_packet(eeg_channel_t channel,
                         const uint8_t *data,
                         size_t len)
{
    if (eeg_queue == NULL) {
        return;
    }

    if (data == NULL) {
        return;
    }

    if (len < EEG_PACKET_SIZE) {
        return;
    }

    eeg_packet_t packet;

    packet.channel = (uint8_t)channel;
    packet.len = EEG_PACKET_SIZE;

    memcpy(
        packet.data,
        data,
        EEG_PACKET_SIZE
    );

    /*
     * Do NOT block the NimBLE task.
     */
    if (xQueueSend(
            eeg_queue,
            &packet,
            0
        ) != pdTRUE) {

        /*
         * Queue full.
         * Drop packet rather than blocking BLE.
         */
        static uint32_t dropped = 0;

        dropped++;

        if ((dropped % 100) == 0) {

            ESP_LOGW(
                TAG,
                "EEG queue full, dropped=%lu",
                (unsigned long)dropped
            );
        }
    }
}



/*
 * PPG processing is owned by ppg_hr.c.
 *
 * The historical PPG implementation that used to live below
 * this point has intentionally been removed to prevent duplicate
 * heart-rate processing paths.
 */
