/*
 * NM-CYD-C5-Muse2
 * ESP32-C5 + Muse 2 EEG Device Controller
 *
 * Copyright (c) 2026 Shibily VM. All rights reserved.
 *
 * PUBLIC-v1.0
 * Version 1.0.0
 *
 * Project-specific source code is protected by the accompanying LICENSE.
 *
 * Third-party components retain their own copyrights, licenses, and
 * trademarks. This includes ESP-IDF, FreeRTOS, NimBLE, Muse hardware,
 * Muse firmware, and other third-party material.
 *
 * Do not include passwords, API keys, private keys, private EEG data,
 * private calibration data, private device identifiers, or confidential
 * employer/client/institutional material.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "eeg_dsp.h"
#include "eeg_display.h"
#include "ppg_hr.h"
#include "mind_control.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define TAG "MUSE2"

#define MUSE_NAME "Muse-xxxx" // "xxxx" replace to your muse2 device name

/*
 * Confirmed from the actual MU-03 GATT discovery.
 *
 * Characteristic value handles:
 *
 * TP9  = 0x0020
 * AF7  = 0x0023
 * AF8  = 0x0026
 * TP10 = 0x0029
 *
 * CCCD handles:
 *
 * TP9  = 0x0021
 * AF7  = 0x0024
 * AF8  = 0x0027
 * TP10 = 0x002A
 */

#define MUSE_TP9_VALUE_HANDLE   0x0020
#define MUSE_TP9_CCCD_HANDLE    0x0021

#define MUSE_AF7_VALUE_HANDLE   0x0023
#define MUSE_AF7_CCCD_HANDLE    0x0024

#define MUSE_AF8_VALUE_HANDLE   0x0026
#define MUSE_AF8_CCCD_HANDLE    0x0027

#define MUSE_TP10_VALUE_HANDLE  0x0029
#define MUSE_TP10_CCCD_HANDLE   0x002A


#define MUSE_PPG1_VALUE_HANDLE   0x0038
#define MUSE_PPG1_CCCD_HANDLE    0x0039

#define MUSE_PPG2_VALUE_HANDLE   0x003B
#define MUSE_PPG2_CCCD_HANDLE    0x003C

#define MUSE_PPG3_VALUE_HANDLE   0x003E
#define MUSE_PPG3_CCCD_HANDLE    0x003F

/* Muse control characteristic */
#define MUSE_CONTROL_VALUE_HANDLE 0x000E

/* -------------------------------------------------------------------------- */
/* Global state                                                               */
/* -------------------------------------------------------------------------- */

static bool muse_found = false;
static bool connecting = false;
static bool connected = false;

static uint16_t muse_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static ble_addr_t muse_addr;
static uint8_t own_addr_type;

/* Notification subscription state */

static bool tp9_subscribed = false;
static bool af7_subscribed = false;
static bool af8_subscribed = false;
static bool tp10_subscribed = false;

static bool ppg1_subscribed = false;
static bool ppg2_subscribed = false;
static bool ppg3_subscribed = false;

static uint32_t ppg1_packets = 0;
static uint32_t ppg2_packets = 0;
static uint32_t ppg3_packets = 0;

static bool streaming_started = false;

/* Packet counters */

static uint32_t tp9_packets = 0;
static uint32_t af7_packets = 0;
static uint32_t af8_packets = 0;
static uint32_t tp10_packets = 0;

/* -------------------------------------------------------------------------- */
/* UUID printing                                                              */
/* -------------------------------------------------------------------------- */

static void print_uuid(const ble_uuid_any_t *uuid)
{
    char uuid_str[BLE_UUID_STR_LEN];

    ble_uuid_to_str(
        &uuid->u,
        uuid_str
    );

    ESP_LOGI(
        TAG,
        "UUID: %s",
        uuid_str
    );
}

/*
 * Muse 2 PPG packet decoder
 *
 * Observed 20-byte packet:
 *
 *   byte 0..1   : packet/header bytes
 *   byte 2..19  : six 24-bit big-endian PPG samples
 *
 * Therefore:
 *
 *   sample[0] = bytes 2,3,4
 *   sample[1] = bytes 5,6,7
 *   ...
 *   sample[5] = bytes 17,18,19
 *
 * This function intentionally does NOT calculate BPM yet.
 * First we verify the decoded sample stream and continuity.
 */

static void decode_ppg_packet(const char *name,
                              const uint8_t *packet,
                              size_t len)
{
    if (packet == NULL || name == NULL) {
        return;
    }

    if (len < 20) {
        return;
    }

    /*
     * Muse PPG packet:
     *
     * byte 0..1  = packet/header bytes
     * byte 2..19 = six 24-bit big-endian samples
     */
    uint32_t samples[6];

    for (int i = 0; i < 6; ++i) {
        int j = 2 + (i * 3);

        samples[i] =
            ((uint32_t)packet[j] << 16) |
            ((uint32_t)packet[j + 1] << 8) |
            (uint32_t)packet[j + 2];
    }

    /*
     * Determine which PPG channel this packet belongs to.
     */
    int channel = -1;

    if (strcmp(name, "PPG1") == 0) {
        channel = 0;
    } else if (strcmp(name, "PPG2") == 0) {
        channel = 1;
    } else if (strcmp(name, "PPG3") == 0) {
        channel = 2;
    }

    if (channel < 0 || channel >= 3) {
        return;
    }

    /*
     * Feed the six decoded samples into the HR processor.
     *
     * No high-rate logging here. This keeps the NimBLE notification
     * path lightweight so EEG packets are not starved.
     */
    for (int i = 0; i < 6; ++i) {
        ppg_hr_push((uint8_t)channel, samples[i]);
    }

    /*
     * Very occasional diagnostic output.
     * One line per 128 packets per channel.
     */
    static uint32_t log_count[3] = {0, 0, 0};

    log_count[channel]++;

    if ((log_count[channel] % 128U) == 0U) {
        ESP_LOGI(TAG,
                 "PPG %s: %lu %lu %lu %lu %lu %lu",
                 name,
                 (unsigned long)samples[0],
                 (unsigned long)samples[1],
                 (unsigned long)samples[2],
                 (unsigned long)samples[3],
                 (unsigned long)samples[4],
                 (unsigned long)samples[5]);
    }
}

static void handle_muse_notification(
    uint16_t attr_handle,
    struct os_mbuf *om)
{
    if (om == NULL) {
        return;
    }

    /*
     * ------------------------------------------------------------
     * PPG CHANNELS
     * ------------------------------------------------------------
     *
     * PPG packets are decoded here only into six 24-bit samples.
     * The samples are then handed to ppg_hr.c.
     * No BPM/FFT/classification work occurs in the BLE callback.
     */

    bool is_ppg = false;
    const char *ppg_name = NULL;

    switch (attr_handle) {

        case MUSE_PPG1_VALUE_HANDLE:
            ppg1_packets++;
            is_ppg = true;
            ppg_name = "PPG1";
            break;

        case MUSE_PPG2_VALUE_HANDLE:
            ppg2_packets++;
            is_ppg = true;
            ppg_name = "PPG2";
            break;

        case MUSE_PPG3_VALUE_HANDLE:
            ppg3_packets++;
            is_ppg = true;
            ppg_name = "PPG3";
            break;

        default:
            break;
    }

    
    if (is_ppg) {
        uint16_t packet_len = OS_MBUF_PKTLEN(om);

        if (packet_len < 20) {
            ESP_LOGW(TAG,
                     "PPG %s packet too short: %u",
                     ppg_name,
                     (unsigned)packet_len);
            return;
        }

        if (packet_len > 256) {
            ESP_LOGW(TAG,
                     "PPG %s packet too large: %u",
                     ppg_name,
                     (unsigned)packet_len);
            return;
        }

        uint8_t packet[256];

        int rc = os_mbuf_copydata(
            om,
            0,
            packet_len,
            packet
        );

        if (rc != 0) {
            ESP_LOGE(TAG,
                     "PPG %s packet copy failed: rc=%d",
                     ppg_name,
                     rc);
            return;
        }

        /*
         * decode_ppg_packet() determines PPG1/PPG2/PPG3
         * internally and receives the actual packet length.
         */
        decode_ppg_packet(
            ppg_name,
            packet,
            packet_len
        );

        return;
    }

    
    /*
     * ------------------------------------------------------------
     * EEG CHANNELS
     * ------------------------------------------------------------
     */

    eeg_channel_t channel;
    const char *name;

    switch (attr_handle) {

        case MUSE_TP9_VALUE_HANDLE:
            channel = EEG_CH_TP9;
            name = "TP9";
            tp9_packets++;
            break;

        case MUSE_AF7_VALUE_HANDLE:
            channel = EEG_CH_AF7;
            name = "AF7";
            af7_packets++;
            break;

        case MUSE_AF8_VALUE_HANDLE:
            channel = EEG_CH_AF8;
            name = "AF8";
            af8_packets++;
            break;

        case MUSE_TP10_VALUE_HANDLE:
            channel = EEG_CH_TP10;
            name = "TP10";
            tp10_packets++;
            break;

        default:
            ESP_LOGW(
                TAG,
                "Unknown notification handle=0x%04X len=%u",
                attr_handle,
                OS_MBUF_PKTLEN(om)
            );
            return;
    }

    uint16_t packet_len = OS_MBUF_PKTLEN(om);

    if (packet_len == 0) {

        ESP_LOGW(
            TAG,
            "%s: empty EEG notification",
            name
        );

        return;
    }

    if (packet_len > 256) {

        ESP_LOGW(
            TAG,
            "%s: notification too large: %u bytes",
            name,
            packet_len
        );

        return;
    }

    uint8_t packet[256];

    int rc = os_mbuf_copydata(
        om,
        0,
        packet_len,
        packet
    );

    if (rc != 0) {

        ESP_LOGE(
            TAG,
            "%s: failed to copy EEG notification: rc=%d",
            name,
            rc
        );

        return;
    }

    /*
     * BLE callback does only:
     *
     *   identify -> copy -> queue
     *
     * DSP/FFT remains outside NimBLE.
     */
    eeg_dsp_push_packet(
        channel,
        packet,
        packet_len
    );
}

/* -------------------------------------------------------------------------- */
/* Enable CCCD notifications                                                 */
/* -------------------------------------------------------------------------- */

/* Forward declaration: sequential CCCD callback */
static int cccd_write_sequential_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    struct ble_gatt_attr *attr,
    void *arg
);

/* Forward declaration: CCCD subscription helper */
static void subscribe_cccd(
    uint16_t cccd_handle,
    const char *name
);

static void subscribe_cccd(
    uint16_t cccd_handle,
    const char *name
)
{
    static const uint8_t notify_enable[] = {
        0x01,
        0x00
    };

    ESP_LOGI(
        TAG,
        "Enabling CCCD %s: handle=0x%04X",
        name,
        cccd_handle
    );

    int rc = ble_gattc_write_flat(
        muse_conn_handle,
        cccd_handle,
        notify_enable,
        sizeof(notify_enable),
        cccd_write_sequential_cb,
        (void *)name
    );

    if (rc != 0) {
        ESP_LOGE(
            TAG,
            "CCCD %s write failed: handle=0x%04X rc=%d",
            name,
            cccd_handle,
            rc
        );
    }
}

static int cccd_write_sequential_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    struct ble_gatt_attr *attr,
    void *arg)
{
    const char *name = (const char *)arg;

    if (error == NULL) {
        ESP_LOGE(TAG, "CCCD %s: NULL error structure", name);
        return 0;
    }

    if (error->status != 0) {

        ESP_LOGE(
            TAG,
            "CCCD %s FAILED: status=%d",
            name,
            error->status
        );

        return 0;
    }

    ESP_LOGI(
        TAG,
        "CCCD %s ENABLED",
        name
    );

    if (strcmp(name, "TP9") == 0) {
        tp9_subscribed = true;
    }
    else if (strcmp(name, "AF7") == 0) {
        af7_subscribed = true;
    }
    else if (strcmp(name, "AF8") == 0) {
        af8_subscribed = true;
    }
    else if (strcmp(name, "TP10") == 0) {
        tp10_subscribed = true;
    }
    else if (strcmp(name, "PPG1") == 0) {
        ppg1_subscribed = true;
    }
    else if (strcmp(name, "PPG2") == 0) {
        ppg2_subscribed = true;
    }
    else if (strcmp(name, "PPG3") == 0) {
        ppg3_subscribed = true;
    }

    /*
     * Continue one CCCD write at a time.
     */
    if (!tp9_subscribed) {

        subscribe_cccd(
            MUSE_TP9_CCCD_HANDLE,
            "TP9"
        );
    }
    else if (!af7_subscribed) {

        subscribe_cccd(
            MUSE_AF7_CCCD_HANDLE,
            "AF7"
        );
    }
    else if (!af8_subscribed) {

        subscribe_cccd(
            MUSE_AF8_CCCD_HANDLE,
            "AF8"
        );
    }
    else if (!tp10_subscribed) {

        subscribe_cccd(
            MUSE_TP10_CCCD_HANDLE,
            "TP10"
        );
    }
    else if (!ppg1_subscribed) {

        subscribe_cccd(
            MUSE_PPG1_CCCD_HANDLE,
            "PPG1"
        );
    }
    else if (!ppg2_subscribed) {

        subscribe_cccd(
            MUSE_PPG2_CCCD_HANDLE,
            "PPG2"
        );
    }
    else if (!ppg3_subscribed) {

        subscribe_cccd(
            MUSE_PPG3_CCCD_HANDLE,
            "PPG3"
        );
    }
    else {

        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "ALL EEG + PPG NOTIFICATIONS ENABLED");
        ESP_LOGI(TAG, "========================================");

        /*
         * Keep the currently proven-good Muse streaming command.
         */
        static const uint8_t start_cmd[] = {
            0x02,
            0x64,
            0x0A
        };

        int rc = ble_gattc_write_no_rsp_flat(
            muse_conn_handle,
            MUSE_CONTROL_VALUE_HANDLE,
            start_cmd,
            sizeof(start_cmd)
        );

        if (rc != 0) {

            ESP_LOGE(
                TAG,
                "Muse start command failed: rc=%d",
                rc
            );

        } else {

            streaming_started = true;

            ESP_LOGI(
                TAG,
                "Muse EEG + PPG streaming START command sent"
            );

            ESP_LOGI(
                TAG,
                "Command bytes: 02 64 0A"
            );

            ESP_LOGI(
                TAG,
                "Waiting for EEG + PPG notifications..."
            );
        }

        return 0;
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Start sequential EEG subscriptions                                         */
/* -------------------------------------------------------------------------- */

static void start_eeg_notifications(void);

static void muse_subscription_task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(200));

    start_eeg_notifications();

    vTaskDelete(NULL);
}

static void start_eeg_notifications(void)
{
    static const uint8_t notify_enable[] = {
        0x01,
        0x00
    };

    tp9_subscribed = false;
    af7_subscribed = false;
    af8_subscribed = false;
    tp10_subscribed = false;

    ppg1_subscribed = false;
    ppg2_subscribed = false;
    ppg3_subscribed = false;

    ppg1_packets = 0;
    ppg2_packets = 0;
    ppg3_packets = 0;

    streaming_started = false;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ENABLING EEG + PPG NOTIFICATIONS");
    ESP_LOGI(TAG, "========================================");

    int rc = ble_gattc_write_flat(
        muse_conn_handle,
        MUSE_TP9_CCCD_HANDLE,
        notify_enable,
        sizeof(notify_enable),
        cccd_write_sequential_cb,
        (void *)"TP9"
    );

    if (rc != 0) {

        ESP_LOGE(
            TAG,
            "TP9 CCCD write failed: rc=%d",
            rc
        );
    }
}

/* -------------------------------------------------------------------------- */
/* GATT characteristic discovery                                              */
/* -------------------------------------------------------------------------- */

#define MAX_SERVICES 32

struct muse_service {
    uint16_t start_handle;
    uint16_t end_handle;
    ble_uuid_any_t uuid;
};

static struct muse_service services[MAX_SERVICES];

static int service_count = 0;
static int current_service_index = 0;

/* -------------------------------------------------------------------------- */
/* Characteristic discovery callback                                          */
/* -------------------------------------------------------------------------- */

static int gatt_chr_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_chr *chr,
    void *arg)
{
    int status = error ? error->status : -999;

    ESP_LOGI(TAG,
             ">>> GATT CHR CALLBACK <<< status=%d chr=%p",
             status,
             (void *)chr);

    if (status == 0 && chr != NULL) {

        ESP_LOGI(TAG, "----------------------------------------");
        ESP_LOGI(TAG, "CHARACTERISTIC");

        ESP_LOGI(
            TAG,
            "  Definition handle : 0x%04X",
            chr->def_handle
        );

        ESP_LOGI(
            TAG,
            "  Value handle      : 0x%04X",
            chr->val_handle
        );

        ESP_LOGI(
            TAG,
            "  Properties        : 0x%02X",
            chr->properties
        );

        if (chr->properties & BLE_GATT_CHR_F_READ) {
            ESP_LOGI(TAG, "    READ");
        }

        if (chr->properties & BLE_GATT_CHR_F_WRITE) {
            ESP_LOGI(TAG, "    WRITE");
        }

        if (chr->properties & BLE_GATT_CHR_F_WRITE_NO_RSP) {
            ESP_LOGI(TAG, "    WRITE WITHOUT RESPONSE");
        }

        if (chr->properties & BLE_GATT_CHR_F_NOTIFY) {
            ESP_LOGI(TAG, "    NOTIFY");
        }

        if (chr->properties & BLE_GATT_CHR_F_INDICATE) {
            ESP_LOGI(TAG, "    INDICATE");
        }

        print_uuid(&chr->uuid);

        return 0;
    }

    if (error->status == BLE_HS_EDONE) {

        ESP_LOGI(
            TAG,
            "Characteristic discovery complete"
        );

        current_service_index++;

        if (current_service_index < service_count) {

            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "Discovering characteristics");
            ESP_LOGI(
                TAG,
                "Service: %d",
                current_service_index
            );

            ESP_LOGI(
                TAG,
                "Handles: 0x%04X - 0x%04X",
                services[current_service_index].start_handle,
                services[current_service_index].end_handle
            );

            int rc = ble_gattc_disc_all_chrs(
                conn_handle,
                services[current_service_index].start_handle,
                services[current_service_index].end_handle,
                gatt_chr_cb,
                NULL
            );

            if (rc != 0) {

                ESP_LOGE(
                    TAG,
                    "Characteristic discovery failed: rc=%d",
                    rc
                );
            }

        } else {

            ESP_LOGI(TAG, "========================================");
            ESP_LOGI(TAG, "GATT DISCOVERY COMPLETE");
            ESP_LOGI(TAG, "========================================");

            ESP_LOGI(
                TAG,
                "Muse connection is ready for EEG."
            );

            /*
             * Do not delay inside the NimBLE GATT callback.
             * Schedule subscription startup on a FreeRTOS task.
             */
            BaseType_t task_rc = xTaskCreate(
                muse_subscription_task,
                "muse_subscribe",
                4096,
                NULL,
                5,
                NULL
            );

            if (task_rc != pdPASS) {
                ESP_LOGE(
                    TAG,
                    "Failed to create subscription task"
                );
            }
        }

        return 0;
    }

    if (error->status != 0) {

        ESP_LOGE(
            TAG,
            "Characteristic discovery error: status=%d",
            error->status
        );
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* GATT service discovery callback                                            */
/* -------------------------------------------------------------------------- */

static int gatt_svc_cb(
    uint16_t conn_handle,
    const struct ble_gatt_error *error,
    const struct ble_gatt_svc *service,
    void *arg)
{
    if (error->status == 0 && service != NULL) {

        if (service_count >= MAX_SERVICES) {
            ESP_LOGW(TAG, "Maximum service count reached");
            return 0;
        }

        ESP_LOGI(TAG, "----------------------------------------");

        ESP_LOGI(
            TAG,
            "SERVICE %d",
            service_count
        );

        ESP_LOGI(
            TAG,
            "  Start handle: 0x%04X",
            service->start_handle
        );

        ESP_LOGI(
            TAG,
            "  End handle  : 0x%04X",
            service->end_handle
        );

        ESP_LOGI(
            TAG,
            "  UUID:"
        );

        print_uuid(&service->uuid);

        services[service_count].start_handle =
            service->start_handle;

        services[service_count].end_handle =
            service->end_handle;

        memcpy(
            &services[service_count].uuid,
            &service->uuid,
            sizeof(ble_uuid_any_t)
        );

        service_count++;

        return 0;
    }

    if (error->status == BLE_HS_EDONE) {

        ESP_LOGI(TAG, "========================================");

        ESP_LOGI(
            TAG,
            "Service discovery complete: %d services",
            service_count
        );

        ESP_LOGI(TAG, "========================================");

        if (service_count == 0) {

            ESP_LOGE(
                TAG,
                "No GATT services discovered"
            );

            return 0;
        }

        current_service_index = 0;

        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "Discovering characteristics");
        ESP_LOGI(TAG, "Service: 0");

        ESP_LOGI(
            TAG,
            "Handles: 0x%04X - 0x%04X",
            services[0].start_handle,
            services[0].end_handle
        );

        int rc = ble_gattc_disc_all_chrs(
            conn_handle,
            services[0].start_handle,
            services[0].end_handle,
            gatt_chr_cb,
            NULL
        );

        if (rc != 0) {

            ESP_LOGE(
                TAG,
                "Failed to start characteristic discovery: rc=%d",
                rc
            );
        }

        return 0;
    }

    if (error->status != 0) {

        ESP_LOGE(
            TAG,
            "Service discovery error: status=%d",
            error->status
        );
    }

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Start GATT discovery                                                       */
/* -------------------------------------------------------------------------- */

static void start_gatt_discovery(void)
{
    service_count = 0;
    current_service_index = 0;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "STARTING GATT SERVICE DISCOVERY");
    ESP_LOGI(TAG, "========================================");

    int rc = ble_gattc_disc_all_svcs(
        muse_conn_handle,
        gatt_svc_cb,
        NULL
    );

    if (rc != 0) {

        ESP_LOGE(
            TAG,
            "Failed to start service discovery: rc=%d",
            rc
        );
    }
}

/* -------------------------------------------------------------------------- */
/* Forward declaration                                                        */
/* -------------------------------------------------------------------------- */

static int gap_event(struct ble_gap_event *event, void *arg);

/* -------------------------------------------------------------------------- */
/* Delayed BLE rescan task                                                    */
/* -------------------------------------------------------------------------- */

static void muse_rescan_task(void *arg)
{
    (void)arg;

    /*
     * IMPORTANT:
     * This delay is outside the NimBLE host/GAP callback.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Starting BLE scan again...");

    int rc = ble_gap_disc(
        own_addr_type,
        BLE_HS_FOREVER,
        NULL,
        gap_event,
        NULL
    );

    if (rc != 0) {
        ESP_LOGE(
            TAG,
            "Failed to restart scan: rc=%d",
            rc
        );
    } else {
        ESP_LOGI(
            TAG,
            "BLE scan restarted"
        );
    }

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* GAP event                                                                  */
/* -------------------------------------------------------------------------- */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

        /* ------------------------------------------------------------------ */
        /* BLE scan result                                                     */
        /* ------------------------------------------------------------------ */

        case BLE_GAP_EVENT_DISC: {

            struct ble_hs_adv_fields fields;

            memset(
                &fields,
                0,
                sizeof(fields)
            );

            int rc = ble_hs_adv_parse_fields(
                &fields,
                event->disc.data,
                event->disc.length_data
            );

            if (rc != 0) {
                return 0;
            }

            char name[64] = {0};

            if (fields.name != NULL &&
                fields.name_len > 0) {

                size_t n = fields.name_len;

                if (n >= sizeof(name)) {
                    n = sizeof(name) - 1;
                }

                memcpy(
                    name,
                    fields.name,
                    n
                );

                name[n] = '\0';
            }

            if (strlen(name) > 0) {

                ESP_LOGI(
                    TAG,
                    "BLE device: RSSI=%d",
                    event->disc.rssi
                );

                ESP_LOGI(
                    TAG,
                    "  Name: %s",
                    name
                );
            }

            if (!muse_found &&
                strcmp(name, MUSE_NAME) == 0) {

                muse_found = true;

                memcpy(
                    &muse_addr,
                    &event->disc.addr,
                    sizeof(ble_addr_t)
                );

                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, " MUSE 2 FOUND");
                ESP_LOGI(TAG, " Name: %s", name);

                ESP_LOGI(
                    TAG,
                    " Address: %02X:%02X:%02X:%02X:%02X:%02X",
                    muse_addr.val[5],
                    muse_addr.val[4],
                    muse_addr.val[3],
                    muse_addr.val[2],
                    muse_addr.val[1],
                    muse_addr.val[0]
                );

                ESP_LOGI(TAG, "========================================");

                /*
                 * Stop scanning before connecting.
                 */

                int rc = ble_gap_disc_cancel();

                if (rc != 0) {

                    ESP_LOGW(
                        TAG,
                        "Scan cancel returned rc=%d",
                        rc
                    );
                }

                connecting = true;

                ESP_LOGI(
                    TAG,
                    "Connecting to Muse..."
                );

                rc = ble_gap_connect(
                    own_addr_type,
                    &muse_addr,
                    30000,
                    NULL,
                    gap_event,
                    NULL
                );

                if (rc != 0) {

                    ESP_LOGE(
                        TAG,
                        "Connection start failed: rc=%d",
                        rc
                    );

                    connecting = false;
                    muse_found = false;
                }
            }

            return 0;
        }

        /* ------------------------------------------------------------------ */
        /* Connection established                                              */
        /* ------------------------------------------------------------------ */

        case BLE_GAP_EVENT_CONNECT: {

            if (event->connect.status == 0) {

                connected = true;
                connecting = false;

                muse_conn_handle =
                    event->connect.conn_handle;

                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, " MUSE CONNECTED");
                ESP_LOGI(
                    TAG,
                    " Connection handle: %d",
                    muse_conn_handle
                );
                ESP_LOGI(TAG, "========================================");

                start_gatt_discovery();

            } else {

                ESP_LOGE(
                    TAG,
                    "Connection failed: status=%d",
                    event->connect.status
                );

                connected = false;
                connecting = false;
                muse_found = false;

                /*
                 * Restart scanning from a separate FreeRTOS task.
                 *
                 * Do NOT call vTaskDelay() or ble_gap_disc()
                 * directly from the NimBLE GAP callback.
                 */
                ESP_LOGI(
                    TAG,
                    "Scheduling Muse scan restart..."
                );

                BaseType_t task_rc = xTaskCreate(
                    muse_rescan_task,
                    "muse_rescan",
                    4096,
                    NULL,
                    5,
                    NULL
                );

                if (task_rc != pdPASS) {
                    ESP_LOGE(
                        TAG,
                        "Failed to create BLE rescan task"
                    );
                }
            }

            return 0;
        }

        /* ------------------------------------------------------------------ */
        /* Notification received                                               */
        /* ------------------------------------------------------------------ */

        case BLE_GAP_EVENT_NOTIFY_RX: {

            /*
             * Notification RX is very high-rate because Muse sends
             * four EEG + three PPG streams. Do not log every packet;
             * UART logging can starve the DSP/display tasks.
             */
            static uint32_t rx_log_count = 0;
            rx_log_count++;

            if ((rx_log_count % 500U) == 0U) {
                ESP_LOGI(
                    TAG,
                    "BLE RX notifications=%lu",
                    (unsigned long)rx_log_count
                );
            }

            handle_muse_notification(
                event->notify_rx.attr_handle,
                event->notify_rx.om
            );

            return 0;
        }

        case BLE_GAP_EVENT_DISCONNECT: {

            ESP_LOGW(TAG, "========================================");
            ESP_LOGW(TAG, " MUSE DISCONNECTED");
            ESP_LOGW(
                TAG,
                " Reason: %d",
                event->disconnect.reason
            );
            ESP_LOGW(TAG, "========================================");

            connected = false;
            connecting = false;
            muse_conn_handle = BLE_HS_CONN_HANDLE_NONE;

            tp9_subscribed = false;
            af7_subscribed = false;
            af8_subscribed = false;
            tp10_subscribed = false;
            streaming_started = false;

            /*
             * Start scanning again from a separate FreeRTOS task.
             *
             * Do NOT delay or restart GAP directly inside this
             * NimBLE callback.
             */

            muse_found = false;

            BaseType_t task_rc = xTaskCreate(
                muse_rescan_task,
                "muse_rescan",
                4096,
                NULL,
                5,
                NULL
            );

            if (task_rc != pdPASS) {

                ESP_LOGE(
                    TAG,
                    "Failed to create BLE rescan task"
                );
            }

            return 0;
        }

        default:
            return 0;
    }
}

/* -------------------------------------------------------------------------- */
/* Start BLE scan                                                             */
/* -------------------------------------------------------------------------- */

static void start_scan(void)
{
    struct ble_gap_disc_params params;

    memset(
        &params,
        0,
        sizeof(params)
    );

    params.passive = 1;
    params.filter_duplicates = 0;
    params.itvl = 0x0010;
    params.window = 0x0010;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " STARTING BLE SCAN");
    ESP_LOGI(TAG, "========================================");

    int rc = ble_gap_disc(
        own_addr_type,
        BLE_HS_FOREVER,
        &params,
        gap_event,
        NULL
    );

    if (rc != 0) {

        ESP_LOGE(
            TAG,
            "BLE scan failed to start: rc=%d",
            rc
        );

    } else {

        ESP_LOGI(
            TAG,
            "BLE scan started"
        );
    }
}

/* -------------------------------------------------------------------------- */
/* NimBLE reset callback                                                      */
/* -------------------------------------------------------------------------- */

static void ble_on_reset(int reason)
{
    ESP_LOGE(
        TAG,
        "NimBLE reset, reason=%d",
        reason
    );
}

/* -------------------------------------------------------------------------- */
/* NimBLE synchronization callback                                            */
/* -------------------------------------------------------------------------- */

static void ble_on_sync(void)
{
    ESP_LOGI(
        TAG,
        "NimBLE host synchronized"
    );

    int rc = ble_hs_id_infer_auto(
        0,
        &own_addr_type
    );

    if (rc != 0) {

        ESP_LOGE(
            TAG,
            "Failed to infer own address type: rc=%d",
            rc
        );

        return;
    }

    start_scan();
}

/* -------------------------------------------------------------------------- */
/* NimBLE host task                                                           */
/* -------------------------------------------------------------------------- */

static void nimble_host_task(void *param)
{
    ESP_LOGI(
        TAG,
        "NimBLE host task started"
    );

    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    esp_log_level_set("MUSE2", ESP_LOG_INFO);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " NM CYD C5 + MUSE 2 EEG");
    ESP_LOGI(TAG, " ESP32-C5 / ESP-IDF 5.5.2");
    ESP_LOGI(TAG, " RAW EEG NOTIFICATION CAPTURE");
    ESP_LOGI(TAG, "========================================");

    /* ---------------------------------------------------------------------- */
    /* NVS                                                                     */
    /* ---------------------------------------------------------------------- */

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(
        TAG,
        "NVS initialized"
    );

    /*
     * Initialize subsystems only after NVS is ready.
     * mind_control loads persistent calibration profiles here.
     */
    ppg_hr_init();
    mind_control_init();

    eeg_dsp_set_af7_feature_callback(
        mind_control_update_features
    );

    eeg_dsp_start();
    eeg_display_start();

    /* ---------------------------------------------------------------------- */
    /* NimBLE                                                                  */
    /* ---------------------------------------------------------------------- */

    ret = nimble_port_init();

    if (ret != ESP_OK) {

        ESP_LOGE(
            TAG,
            "nimble_port_init failed: %s",
            esp_err_to_name(ret)
        );

        return;
    }

    ESP_LOGI(
        TAG,
        "NimBLE initialized"
    );

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;

    /*
     * Start NimBLE host task.
     */

    nimble_port_freertos_init(
        nimble_host_task
    );
}
