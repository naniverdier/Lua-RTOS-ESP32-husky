#include "sdkconfig.h"

#if CONFIG_LUA_RTOS_LUA_USE_HUSKYLENS

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_defs.h"

#include "huskylens_mapper.h"

#define TAG "HUSKY_BLE"

#define HUSKY_MAPPER_NAMESPACE "apriltag"
#define HUSKY_MAPPER_KEY       "map"
#define HUSKY_MAPPER_SEQ_KEY   "seq"

#define HUSKY_MAPPER_DEVICE_NAME "AprilTag_Mapper"

#define HUSKY_MAPPER_APP_ID 0x47
#define HUSKY_MAPPER_SVC_INST_ID 0

#define HUSKY_MAPPER_POLL_MS 100
#define HUSKY_MAPPER_SEQ_MAX_PAYLOAD (1 + (HUSKYLENS_SEQ_MAX_SEQS * (1 + HUSKYLENS_SEQ_MAX)))

typedef struct {
    uint8_t physical_id;
    uint8_t logical_id;
} husky_map_entry_t;

typedef struct {
    uint8_t count;
    husky_map_entry_t entries[HUSKYLENS_MAP_MAX];
} husky_map_table_t;

typedef struct {
    uint8_t len;
    uint8_t ids[HUSKYLENS_SEQ_MAX];
} husky_seq_t;

typedef struct {
    uint8_t count;
    uint8_t lens[HUSKYLENS_SEQ_MAX_SEQS];
    uint8_t ids[HUSKYLENS_SEQ_MAX_SEQS][HUSKYLENS_SEQ_MAX];
} husky_seq_list_t;

static husky_map_table_t s_map;
static SemaphoreHandle_t s_map_mutex;
static husky_seq_list_t s_seqs;
static SemaphoreHandle_t s_seq_mutex;

static huskylens_t *s_husky = NULL;
static TaskHandle_t s_task = NULL;
static bool s_task_running = false;

static bool s_ble_inited = false;
static bool s_notify_enabled = false;
static bool s_connected = false;
static uint16_t s_conn_id = 0xffff;
static esp_gatt_if_t s_gatts_if = ESP_GATT_IF_NONE;
static uint16_t s_mtu_size = 23;
static uint8_t s_frame_seq = 0;

static uint8_t s_last_pairs[HUSKYLENS_MAP_MAX * 2];
static uint8_t s_last_count = 0;
static bool s_map_loaded = false;
static bool s_seq_loaded = false;
static void (*s_conn_cb)(bool connected) = NULL;

// Formato de mapeo (binario):
//   [count:1] + count * {physical_id:1, logical_id:1}
//
// Formato de notificación (event-driven):
//   [flags:1][seq:1][count:1] + count * {physical_id:1, logical_id:1}
//   flags bit0 = chunked (1 si el frame se divide en varios paquetes)
//   flags bit1 = last (1 si es el último paquete del frame)

// 128-bit UUIDs (little endian)
static const uint8_t HUSKY_MAPPER_SERVICE_UUID[16]      = {0x00,0x00,0xbc,0x9a,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12};
static const uint8_t HUSKY_MAPPER_CFG_WRITE_UUID[16]    = {0x01,0x00,0xbc,0x9a,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12};
static const uint8_t HUSKY_MAPPER_CFG_READ_UUID[16]     = {0x02,0x00,0xbc,0x9a,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12};

// 16-bit UUID for advertising (derived from 128-bit UUID)
static const uint16_t husky_mapper_service_uuid_16 = 0x9ABC;

// Raw advertising data
static uint8_t *s_adv_data_raw = NULL;
static size_t s_adv_data_raw_size = 0;
static const uint8_t HUSKY_MAPPER_DET_NOTIFY_UUID[16]   = {0x03,0x00,0xbc,0x9a,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12};
static const uint8_t HUSKY_MAPPER_SEQ_WRITE_UUID[16]    = {0x04,0x00,0xbc,0x9a,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12};
static const uint8_t HUSKY_MAPPER_SEQ_READ_UUID[16]     = {0x05,0x00,0xbc,0x9a,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12};

enum {
    HUSKY_IDX_SVC,
    HUSKY_IDX_CFG_WRITE_CHAR,
    HUSKY_IDX_CFG_WRITE_VAL,
    HUSKY_IDX_CFG_READ_CHAR,
    HUSKY_IDX_CFG_READ_VAL,
    HUSKY_IDX_SEQ_WRITE_CHAR,
    HUSKY_IDX_SEQ_WRITE_VAL,
    HUSKY_IDX_SEQ_READ_CHAR,
    HUSKY_IDX_SEQ_READ_VAL,
    HUSKY_IDX_DET_NOTIFY_CHAR,
    HUSKY_IDX_DET_NOTIFY_VAL,
    HUSKY_IDX_DET_NOTIFY_CCC,
    HUSKY_IDX_NB,
};

static uint16_t s_handle_table[HUSKY_IDX_NB];

static const uint16_t primary_service_uuid        = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid  = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read  = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t ccc_default[2] = {0x00, 0x00};

static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

typedef struct {
    uint8_t *buf;
    uint16_t len;
} prep_write_env_t;

static prep_write_env_t s_prep = {0};
static prep_write_env_t s_prep_seq = {0};

static const esp_gatts_attr_db_t s_gatt_db[HUSKY_IDX_NB] = {
    [HUSKY_IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(husky_mapper_service_uuid_16), sizeof(husky_mapper_service_uuid_16), (uint8_t *)&husky_mapper_service_uuid_16},
    },

    [HUSKY_IDX_CFG_WRITE_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write},
    },

    [HUSKY_IDX_CFG_WRITE_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, (uint8_t *)HUSKY_MAPPER_CFG_WRITE_UUID, ESP_GATT_PERM_WRITE,
         0, 0, NULL},
    },

    [HUSKY_IDX_CFG_READ_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read},
    },

    [HUSKY_IDX_CFG_READ_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, (uint8_t *)HUSKY_MAPPER_CFG_READ_UUID, ESP_GATT_PERM_READ,
         0, 0, NULL},
    },

    [HUSKY_IDX_SEQ_WRITE_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write},
    },

    [HUSKY_IDX_SEQ_WRITE_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, (uint8_t *)HUSKY_MAPPER_SEQ_WRITE_UUID, ESP_GATT_PERM_WRITE,
         0, 0, NULL},
    },

    [HUSKY_IDX_SEQ_READ_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read},
    },

    [HUSKY_IDX_SEQ_READ_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, (uint8_t *)HUSKY_MAPPER_SEQ_READ_UUID, ESP_GATT_PERM_READ,
         0, 0, NULL},
    },

    [HUSKY_IDX_DET_NOTIFY_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_notify},
    },

    [HUSKY_IDX_DET_NOTIFY_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, (uint8_t *)HUSKY_MAPPER_DET_NOTIFY_UUID, ESP_GATT_PERM_READ,
         0, 0, NULL},
    },

    [HUSKY_IDX_DET_NOTIFY_CCC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(ccc_default), sizeof(ccc_default), (uint8_t *)ccc_default},
    },
};

static void cleanup_prep_buffers(void) {
    if (s_prep.buf) {
        free(s_prep.buf);
        s_prep.buf = NULL;
        s_prep.len = 0;
    }
    if (s_prep_seq.buf) {
        free(s_prep_seq.buf);
        s_prep_seq.buf = NULL;
        s_prep_seq.len = 0;
    }
}

static void map_lock(void) {
    if (s_map_mutex) {
        xSemaphoreTake(s_map_mutex, portMAX_DELAY);
    }
}

static void map_unlock(void) {
    if (s_map_mutex) {
        xSemaphoreGive(s_map_mutex);
    }
}

static void seq_lock(void) {
    if (s_seq_mutex) {
        xSemaphoreTake(s_seq_mutex, portMAX_DELAY);
    }
}

static void seq_unlock(void) {
    if (s_seq_mutex) {
        xSemaphoreGive(s_seq_mutex);
    }
}

static void map_clear(void) {
    map_lock();
    s_map.count = 0;
    memset(s_map.entries, 0, sizeof(s_map.entries));
    map_unlock();
}

static void map_set_from_pairs(const uint8_t *data, uint8_t count) {
    map_lock();
    s_map.count = 0;
    memset(s_map.entries, 0, sizeof(s_map.entries));

    for (uint8_t i = 0; i < count && i < HUSKYLENS_MAP_MAX; i++) {
        uint8_t physical = data[i * 2];
        uint8_t logical = data[i * 2 + 1];

        bool replaced = false;
        for (uint8_t j = 0; j < s_map.count; j++) {
            if (s_map.entries[j].physical_id == physical) {
                s_map.entries[j].logical_id = logical;
                replaced = true;
                break;
            }
        }
        if (!replaced && s_map.count < HUSKYLENS_MAP_MAX) {
            s_map.entries[s_map.count].physical_id = physical;
            s_map.entries[s_map.count].logical_id = logical;
            s_map.count++;
        }
    }
    map_unlock();
}

static uint8_t map_pack(uint8_t *out, size_t out_len) {
    map_lock();
    uint8_t count = s_map.count;
    size_t needed = (size_t)(1 + count * 2);
    if (out_len < needed) {
        count = (uint8_t)((out_len - 1) / 2);
        needed = (size_t)(1 + count * 2);
    }
    out[0] = count;
    for (uint8_t i = 0; i < count; i++) {
        out[1 + (i * 2)] = s_map.entries[i].physical_id;
        out[1 + (i * 2) + 1] = s_map.entries[i].logical_id;
    }
    map_unlock();
    return (uint8_t)needed;
}

static void seq_clear(void) {
    seq_lock();
    s_seqs.count = 0;
    memset(s_seqs.lens, 0, sizeof(s_seqs.lens));
    memset(s_seqs.ids, 0, sizeof(s_seqs.ids));
    seq_unlock();
}

static void seq_set_from_list(const uint8_t *data, uint8_t len) {
    if (!data) {
        return;
    }
    if (len > HUSKYLENS_SEQ_MAX) {
        len = HUSKYLENS_SEQ_MAX;
    }
    seq_lock();
    s_seqs.count = 1;
    memset(s_seqs.lens, 0, sizeof(s_seqs.lens));
    memset(s_seqs.ids, 0, sizeof(s_seqs.ids));
    s_seqs.lens[0] = len;
    if (len > 0) {
        memcpy(s_seqs.ids[0], data, len);
    }
    seq_unlock();
}

static uint8_t seq_pack(uint8_t *out, size_t out_len) {
    seq_lock();
    uint8_t count = s_seqs.count;
    size_t offset = 0;
    if (out_len == 0) {
        seq_unlock();
        return 0;
    }
    out[offset++] = count;
    for (uint8_t i = 0; i < count && i < HUSKYLENS_SEQ_MAX_SEQS; i++) {
        uint8_t len = s_seqs.lens[i];
        if (len > HUSKYLENS_SEQ_MAX) {
            len = HUSKYLENS_SEQ_MAX;
        }
        if (offset + 1 > out_len) {
            break;
        }
        out[offset++] = len;
        size_t copy_len = len;
        if (offset + copy_len > out_len) {
            copy_len = out_len - offset;
        }
        if (copy_len > 0) {
            memcpy(&out[offset], s_seqs.ids[i], copy_len);
            offset += copy_len;
        }
        if (copy_len < len) {
            break;
        }
    }
    seq_unlock();
    return (uint8_t)offset;
}

static void map_sort_pairs(uint8_t *pairs, uint8_t count) {
    for (uint8_t i = 1; i < count; i++) {
        uint8_t p = pairs[i * 2];
        uint8_t l = pairs[i * 2 + 1];
        int j = i - 1;
        while (j >= 0 && pairs[j * 2] > p) {
            pairs[(j + 1) * 2] = pairs[j * 2];
            pairs[(j + 1) * 2 + 1] = pairs[j * 2 + 1];
            j--;
        }
        pairs[(j + 1) * 2] = p;
        pairs[(j + 1) * 2 + 1] = l;
    }
}

static uint8_t map_dedup_add(uint8_t *pairs, uint8_t count, uint8_t physical, uint8_t logical) {
    for (uint8_t i = 0; i < count; i++) {
        if (pairs[i * 2] == physical) {
            pairs[i * 2 + 1] = logical;
            return count;
        }
    }
    if (count < HUSKYLENS_MAP_MAX) {
        pairs[count * 2] = physical;
        pairs[count * 2 + 1] = logical;
        return (uint8_t)(count + 1);
    }
    return count;
}

static bool map_changed(const uint8_t *pairs, uint8_t count) {
    if (count != s_last_count) {
        return true;
    }
    if (count == 0) {
        return false;
    }
    return memcmp(pairs, s_last_pairs, (size_t)(count * 2)) != 0;
}

static void map_save_last(const uint8_t *pairs, uint8_t count) {
    s_last_count = count;
    if (count > 0) {
        memcpy(s_last_pairs, pairs, (size_t)(count * 2));
    }
}

static esp_err_t mapper_nvs_load(void) {
    nvs_handle handle;
    size_t size = 0;
    esp_err_t err = nvs_open(HUSKY_MAPPER_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        map_clear();
        s_map_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_blob(handle, HUSKY_MAPPER_KEY, NULL, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        map_clear();
        s_map_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK || size < 1) {
        ESP_LOGE(TAG, "nvs_get_blob size failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        map_clear();
        s_map_loaded = true;
        return err;
    }

    uint8_t *buf = (uint8_t *)malloc(size);
    if (!buf) {
        nvs_close(handle);
        ESP_LOGE(TAG, "sin memoria para cargar mapa");
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_blob(handle, HUSKY_MAPPER_KEY, buf, &size);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob failed: %s", esp_err_to_name(err));
        free(buf);
        map_clear();
        s_map_loaded = true;
        return err;
    }

    uint8_t count = buf[0];
    uint8_t max_pairs = (uint8_t)((size - 1) / 2);
    if (count > max_pairs) {
        count = max_pairs;
    }
    map_set_from_pairs(&buf[1], count);
    free(buf);
    s_map_loaded = true;
    ESP_LOGI(TAG, "mapa cargado desde NVS (%u entradas)", count);
    return ESP_OK;
}

static esp_err_t mapper_nvs_load_seq(void) {
    nvs_handle handle;
    size_t size = 0;
    esp_err_t err = nvs_open(HUSKY_MAPPER_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        seq_clear();
        s_seq_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open seq failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_blob(handle, HUSKY_MAPPER_SEQ_KEY, NULL, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        seq_clear();
        s_seq_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK || size < 1) {
        ESP_LOGE(TAG, "nvs_get_blob seq size failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        seq_clear();
        s_seq_loaded = true;
        return err;
    }

    uint8_t *buf = (uint8_t *)malloc(size);
    if (!buf) {
        nvs_close(handle);
        ESP_LOGE(TAG, "sin memoria para cargar secuencia");
        return ESP_ERR_NO_MEM;
    }

    err = nvs_get_blob(handle, HUSKY_MAPPER_SEQ_KEY, buf, &size);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob seq failed: %s", esp_err_to_name(err));
        free(buf);
        seq_clear();
        s_seq_loaded = true;
        return err;
    }

    uint8_t seq_count = buf[0];
    if (seq_count > HUSKYLENS_SEQ_MAX_SEQS) {
        seq_count = HUSKYLENS_SEQ_MAX_SEQS;
    }

    seq_lock();
    s_seqs.count = 0;
    memset(s_seqs.lens, 0, sizeof(s_seqs.lens));
    memset(s_seqs.ids, 0, sizeof(s_seqs.ids));

    size_t offset = 1;
    for (uint8_t i = 0; i < seq_count && offset < size; i++) {
        uint8_t slen = buf[offset++];
        if (slen > HUSKYLENS_SEQ_MAX) {
            slen = HUSKYLENS_SEQ_MAX;
        }
        size_t remaining = size - offset;
        uint8_t copy_len = slen;
        if (copy_len > remaining) {
            copy_len = (uint8_t)remaining;
        }
        if (copy_len > 0) {
            memcpy(s_seqs.ids[i], &buf[offset], copy_len);
            offset += copy_len;
        }
        s_seqs.lens[i] = copy_len;
        s_seqs.count = (uint8_t)(i + 1);
        if (copy_len < slen) {
            break;
        }
    }
    seq_unlock();

    free(buf);
    s_seq_loaded = true;
    ESP_LOGI(TAG, "secuencias cargadas desde NVS (%u)", s_seqs.count);
    return ESP_OK;
}

static esp_err_t mapper_nvs_save(void) {
    nvs_handle handle;
    esp_err_t err = nvs_open(HUSKY_MAPPER_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t buf[1 + (HUSKYLENS_MAP_MAX * 2)];
    uint8_t len = map_pack(buf, sizeof(buf));
    err = nvs_set_blob(handle, HUSKY_MAPPER_KEY, buf, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "mapa guardado en NVS (%u bytes)", len);
    return ESP_OK;
}

static esp_err_t mapper_nvs_save_seq(void) {
    nvs_handle handle;
    esp_err_t err = nvs_open(HUSKY_MAPPER_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open seq failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t buf[HUSKY_MAPPER_SEQ_MAX_PAYLOAD];
    uint8_t len = seq_pack(buf, sizeof(buf));
    err = nvs_set_blob(handle, HUSKY_MAPPER_SEQ_KEY, buf, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob seq failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit seq failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "secuencia guardada en NVS (%u bytes)", len);
    return ESP_OK;
}

uint8_t huskylens_remap_april_tag(uint8_t physical_id) {
    uint8_t logical = physical_id;

    map_lock();
    for (uint8_t i = 0; i < s_map.count; i++) {
        if (s_map.entries[i].physical_id == physical_id) {
            logical = s_map.entries[i].logical_id;
            break;
        }
    }
    map_unlock();

    return logical;
}

uint8_t huskylens_mapper_get_pairs(uint8_t *pairs, uint8_t max_pairs) {
    if (!pairs || max_pairs == 0) {
        return 0;
    }

    if (!s_map_loaded) {
        (void)mapper_nvs_load();
    }

    map_lock();
    uint8_t count = s_map.count;
    if (count > max_pairs) {
        count = max_pairs;
    }
    for (uint8_t i = 0; i < count; i++) {
        pairs[i * 2] = s_map.entries[i].physical_id;
        pairs[i * 2 + 1] = s_map.entries[i].logical_id;
    }
    map_unlock();

    return count;
}

uint8_t huskylens_mapper_get_sequence(uint8_t *seq, uint8_t max_len) {
    if (!seq || max_len == 0) {
        return 0;
    }

    if (!s_seq_loaded) {
        (void)mapper_nvs_load_seq();
    }

    seq_lock();
    uint8_t len = (s_seqs.count > 0) ? s_seqs.lens[0] : 0;
    if (len > max_len) {
        len = max_len;
    }
    if (len > 0) {
        memcpy(seq, s_seqs.ids[0], len);
    }
    seq_unlock();
    return len;
}

bool huskylens_mapper_set_sequence(const uint8_t *seq, uint8_t len) {
    if (!seq) {
        return false;
    }
    if (len > HUSKYLENS_SEQ_MAX) {
        len = HUSKYLENS_SEQ_MAX;
    }

    seq_set_from_list(seq, len);
    s_seq_loaded = true;
    return mapper_nvs_save_seq() == ESP_OK;
}

uint8_t huskylens_mapper_get_sequences(uint8_t *seqs, uint8_t *lens, uint8_t max_seqs, uint8_t max_len) {
    if (!seqs || !lens || max_seqs == 0 || max_len == 0) {
        return 0;
    }
    if (!s_seq_loaded) {
        (void)mapper_nvs_load_seq();
    }

    seq_lock();
    uint8_t count = s_seqs.count;
    if (count > max_seqs) {
        count = max_seqs;
    }
    for (uint8_t i = 0; i < count; i++) {
        uint8_t len = s_seqs.lens[i];
        if (len > max_len) {
            len = max_len;
        }
        lens[i] = len;
        if (len > 0) {
            memcpy(&seqs[i * max_len], s_seqs.ids[i], len);
        }
    }
    seq_unlock();
    return count;
}

bool huskylens_mapper_set_pairs(const uint8_t *pairs, uint8_t count) {
    if (!pairs) {
        return false;
    }
    if (count > HUSKYLENS_MAP_MAX) {
        count = HUSKYLENS_MAP_MAX;
    }

    map_set_from_pairs(pairs, count);
    s_map_loaded = true;

    return mapper_nvs_save() == ESP_OK;
}

void huskylens_mapper_set_conn_cb(void (*cb)(bool connected)) {
    s_conn_cb = cb;
}

static void notify_detections(const uint8_t *pairs, uint8_t count) {
    if (!s_notify_enabled || !s_connected || s_gatts_if == ESP_GATT_IF_NONE) {
        return;
    }

    uint16_t max_payload = (s_mtu_size > 3) ? (uint16_t)(s_mtu_size - 3) : 0;
    if (max_payload < 5) {
        ESP_LOGE(TAG, "MTU demasiado bajo para notificaciones");
        return;
    }

    uint8_t header_size = 3; // flags + seq + count
    uint8_t max_pairs = (uint8_t)((max_payload - header_size) / 2);
    if (max_pairs == 0) {
        ESP_LOGE(TAG, "MTU demasiado bajo para pares");
        return;
    }

    uint8_t seq = s_frame_seq++;
    uint8_t sent = 0;
    while (sent < count) {
        uint8_t chunk_count = (uint8_t)((count - sent) > max_pairs ? max_pairs : (count - sent));
        uint8_t flags = 0x00;
        bool chunked = (count > max_pairs);
        bool last = (sent + chunk_count) >= count;
        if (chunked) {
            flags |= 0x01;
        }
        if (last) {
            flags |= 0x02;
        }

        uint8_t payload_len = (uint8_t)(header_size + (chunk_count * 2));
        uint8_t *payload = (uint8_t *)malloc(payload_len);
        if (!payload) {
            ESP_LOGE(TAG, "sin memoria para notificar");
            return;
        }

        payload[0] = flags;
        payload[1] = seq;
        payload[2] = chunk_count;
        memcpy(&payload[3], &pairs[sent * 2], (size_t)(chunk_count * 2));

        esp_err_t err = esp_ble_gatts_send_indicate(s_gatts_if, s_conn_id,
                                                    s_handle_table[HUSKY_IDX_DET_NOTIFY_VAL],
                                                    payload_len, payload, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "fallo notify: %s", esp_err_to_name(err));
        }

        free(payload);
        sent = (uint8_t)(sent + chunk_count);
    }
}

static void mapper_task(void *arg) {
    (void)arg;
    uint8_t pairs[HUSKYLENS_MAP_MAX * 2];

    while (s_task_running) {
        if (!s_husky) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (huskylens_request_blocks(s_husky)) {
            int16_t count = huskylens_count_blocks(s_husky);
            uint8_t n = 0;
            for (int16_t i = 0; i < count; i++) {
                huskylens_result_t result = huskylens_get_block(s_husky, i);
                if (result.command != COMMAND_RETURN_BLOCK) {
                    continue;
                }
                uint8_t physical = (uint8_t)result.fifth;
                uint8_t logical = huskylens_remap_april_tag(physical);
                n = map_dedup_add(pairs, n, physical, logical);
            }

            map_sort_pairs(pairs, n);

            if (map_changed(pairs, n)) {
                map_save_last(pairs, n);
                notify_detections(pairs, n);
            }
        } else {
            ESP_LOGE(TAG, "huskylens_request_blocks fallo");
        }

        vTaskDelay(pdMS_TO_TICKS(HUSKY_MAPPER_POLL_MS));
    }

    vTaskDelete(NULL);
}

static void handle_config_write(const uint8_t *data, uint16_t len) {
    if (len < 1) {
        ESP_LOGE(TAG, "config write: payload vacio");
        return;
    }

    uint8_t count = data[0];
    uint16_t expected = (uint16_t)(1 + (count * 2));
    if (expected > len) {
        ESP_LOGE(TAG, "config write: longitud invalida (recibido=%u, esperado=%u para count=%u)", 
                 len, expected, count);
        ESP_LOGE(TAG, "  -> El cliente debe usar Long Write (prepared write) para payloads >MTU");
        ESP_LOGE(TAG, "  -> MTU actual=%u, max_payload=%u", s_mtu_size, (s_mtu_size > 3) ? (s_mtu_size - 3) : 0);
        return;
    }

    map_set_from_pairs(&data[1], count);
    (void)mapper_nvs_save();
}

static void handle_sequence_write(const uint8_t *data, uint16_t len) {
    if (len < 1) {
        ESP_LOGE(TAG, "sequence write: payload vacio");
        return;
    }

    // Legacy format: [len][ids...]
    if (len == (uint16_t)(1 + data[0]) && data[0] <= HUSKYLENS_SEQ_MAX) {
        seq_set_from_list(&data[1], data[0]);
        s_seq_loaded = true;
        (void)mapper_nvs_save_seq();
        return;
    }

    uint8_t seq_count = data[0];
    if (seq_count > HUSKYLENS_SEQ_MAX_SEQS) {
        seq_count = HUSKYLENS_SEQ_MAX_SEQS;
    }

    seq_lock();
    s_seqs.count = 0;
    memset(s_seqs.lens, 0, sizeof(s_seqs.lens));
    memset(s_seqs.ids, 0, sizeof(s_seqs.ids));

    uint16_t offset = 1;
    for (uint8_t i = 0; i < seq_count; i++) {
        if (offset >= len) {
            break;
        }
        uint8_t slen = data[offset++];
        if (slen > HUSKYLENS_SEQ_MAX) {
            slen = HUSKYLENS_SEQ_MAX;
        }
        uint16_t remaining = (uint16_t)(len - offset);
        uint8_t copy_len = slen;
        if (copy_len > remaining) {
            copy_len = (uint8_t)remaining;
        }
        if (copy_len > 0) {
            memcpy(s_seqs.ids[i], &data[offset], copy_len);
            offset = (uint16_t)(offset + copy_len);
        }
        s_seqs.lens[i] = copy_len;
        s_seqs.count = (uint8_t)(i + 1);
        if (copy_len < slen) {
            break;
        }
    }
    seq_unlock();

    (void)mapper_nvs_save_seq();
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(TAG, "GAP event: %d", event);
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "adv data raw set complete, starting advertising");
            esp_ble_gap_start_advertising(&s_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ESP_LOGI(TAG, "advertising started, status=%d", param->adv_start_cmpl.status);
            break;
        default:
            break;
    }
    (void)param;
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "GATTS_REG_EVT: app_id=%d, status=%d", param->reg.app_id, param->reg.status);
            s_gatts_if = gatts_if;
            esp_ble_gap_set_device_name(HUSKY_MAPPER_DEVICE_NAME);
            ESP_LOGI(TAG, "Device name set to: %s", HUSKY_MAPPER_DEVICE_NAME);
            esp_ble_gap_config_adv_data_raw(s_adv_data_raw, s_adv_data_raw_size);
            ESP_LOGI(TAG, "Configuring raw advertising data (%d bytes)...", s_adv_data_raw_size);
            esp_ble_gatts_create_attr_tab(s_gatt_db, gatts_if, HUSKY_IDX_NB, HUSKY_MAPPER_SVC_INST_ID);
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "Create attr table failed, error=0x%x", param->add_attr_tab.status);
                break;
            }
            if (param->add_attr_tab.num_handle != HUSKY_IDX_NB) {
                ESP_LOGE(TAG, "Attr table size mismatch (%d != %d)", param->add_attr_tab.num_handle, HUSKY_IDX_NB);
                break;
            }
            memcpy(s_handle_table, param->add_attr_tab.handles, sizeof(s_handle_table));
            esp_ble_gatts_start_service(s_handle_table[HUSKY_IDX_SVC]);
            break;
        }
        case ESP_GATTS_CONNECT_EVT: {
            s_connected = true;
            s_conn_id = param->connect.conn_id;
            s_notify_enabled = false;
            if (s_conn_cb) {
                s_conn_cb(true);
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: {
            cleanup_prep_buffers();
            s_connected = false;
            s_notify_enabled = false;
            s_conn_id = 0xffff;
            esp_ble_gap_start_advertising(&s_adv_params);
            if (s_conn_cb) {
                s_conn_cb(false);
            }
            break;
        }
        case ESP_GATTS_MTU_EVT: {
            s_mtu_size = param->mtu.mtu;
            ESP_LOGI(TAG, "MTU negociado: %u", s_mtu_size);
            break;
        }
        case ESP_GATTS_READ_EVT: {
            if (param->read.handle == s_handle_table[HUSKY_IDX_CFG_READ_VAL]) {
                uint8_t buf[1 + (HUSKYLENS_MAP_MAX * 2)];
                uint8_t len = map_pack(buf, sizeof(buf));

                uint16_t offset = param->read.offset;
                if (offset > len) {
                    esp_gatt_rsp_t rsp = {0};
                    rsp.attr_value.handle = param->read.handle;
                    rsp.attr_value.offset = offset;
                    rsp.attr_value.len = 0;
                    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                                ESP_GATT_INVALID_OFFSET, &rsp);
                    break;
                }

                uint16_t out_len = (uint16_t)(len - offset);
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.offset = offset;
                rsp.attr_value.len = out_len;
                memcpy(rsp.attr_value.value, &buf[offset], out_len);

                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                            ESP_GATT_OK, &rsp);
            }
            if (param->read.handle == s_handle_table[HUSKY_IDX_SEQ_READ_VAL]) {
                uint8_t buf[HUSKY_MAPPER_SEQ_MAX_PAYLOAD];
                uint8_t len = seq_pack(buf, sizeof(buf));

                uint16_t offset = param->read.offset;
                if (offset > len) {
                    esp_gatt_rsp_t rsp = {0};
                    rsp.attr_value.handle = param->read.handle;
                    rsp.attr_value.offset = offset;
                    rsp.attr_value.len = 0;
                    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                                ESP_GATT_INVALID_OFFSET, &rsp);
                    break;
                }

                uint16_t out_len = (uint16_t)(len - offset);
                esp_gatt_rsp_t rsp = {0};
                rsp.attr_value.handle = param->read.handle;
                rsp.attr_value.offset = offset;
                rsp.attr_value.len = out_len;
                memcpy(rsp.attr_value.value, &buf[offset], out_len);

                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                            ESP_GATT_OK, &rsp);
            }
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            if (param->write.handle == s_handle_table[HUSKY_IDX_DET_NOTIFY_CCC]) {
                if (param->write.len == 2) {
                    uint16_t ccc = (uint16_t)(param->write.value[1] << 8) | param->write.value[0];
                    s_notify_enabled = (ccc == 0x0001 || ccc == 0x0002);
                }
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
                break;
            }

            if (param->write.handle == s_handle_table[HUSKY_IDX_CFG_WRITE_VAL]) {
                ESP_LOGI(TAG, "config write: len=%u is_prep=%d offset=%u MTU=%u", 
                         param->write.len, param->write.is_prep, param->write.offset, s_mtu_size);
                if (param->write.is_prep) {
                    ESP_LOGI(TAG, "prepared write chunk recibido");
                    if (!s_prep.buf) {
                        s_prep.buf = (uint8_t *)malloc(1 + (HUSKYLENS_MAP_MAX * 2));
                        s_prep.len = 0;
                    }
                    if (!s_prep.buf) {
                        ESP_LOGE(TAG, "sin memoria para prepare write");
                        break;
                    }
                    if ((param->write.offset + param->write.len) <= (1 + (HUSKYLENS_MAP_MAX * 2))) {
                        memcpy(s_prep.buf + param->write.offset, param->write.value, param->write.len);
                        if (s_prep.len < (param->write.offset + param->write.len)) {
                            s_prep.len = (uint16_t)(param->write.offset + param->write.len);
                        }
                    }
                    if (param->write.need_rsp) {
                        esp_gatt_rsp_t rsp = {0};
                        rsp.attr_value.handle = param->write.handle;
                        rsp.attr_value.offset = param->write.offset;
                        rsp.attr_value.len = param->write.len;
                        memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
                    }
                } else {
                    ESP_LOGI(TAG, "write directo (no preparado)");
                    handle_config_write(param->write.value, param->write.len);
                    if (param->write.need_rsp) {
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    }
                }
            }

            if (param->write.handle == s_handle_table[HUSKY_IDX_SEQ_WRITE_VAL]) {
                if (param->write.is_prep) {
                    if (!s_prep_seq.buf) {
                        s_prep_seq.buf = (uint8_t *)malloc(HUSKY_MAPPER_SEQ_MAX_PAYLOAD);
                        s_prep_seq.len = 0;
                    }
                    if (!s_prep_seq.buf) {
                        ESP_LOGE(TAG, "sin memoria para prepare write secuencia");
                        break;
                    }
                    if ((param->write.offset + param->write.len) <= HUSKY_MAPPER_SEQ_MAX_PAYLOAD) {
                        memcpy(s_prep_seq.buf + param->write.offset, param->write.value, param->write.len);
                        if (s_prep_seq.len < (param->write.offset + param->write.len)) {
                            s_prep_seq.len = (uint16_t)(param->write.offset + param->write.len);
                        }
                    }
                    if (param->write.need_rsp) {
                        esp_gatt_rsp_t rsp = {0};
                        rsp.attr_value.handle = param->write.handle;
                        rsp.attr_value.offset = param->write.offset;
                        rsp.attr_value.len = param->write.len;
                        memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
                    }
                } else {
                    handle_sequence_write(param->write.value, param->write.len);
                    if (param->write.need_rsp) {
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    }
                }
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT: {
            ESP_LOGI(TAG, "exec write: flag=%d prep_len=%u prep_seq_len=%u",
                     param->exec_write.exec_write_flag, s_prep.len, s_prep_seq.len);
            if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && s_prep.buf) {
                ESP_LOGI(TAG, "ejecutando config write preparado con %u bytes", s_prep.len);
                handle_config_write(s_prep.buf, s_prep.len);
            }
            if (s_prep.buf) {
                free(s_prep.buf);
                s_prep.buf = NULL;
                s_prep.len = 0;
            }
            if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && s_prep_seq.buf) {
                ESP_LOGI(TAG, "ejecutando sequence write preparado con %u bytes", s_prep_seq.len);
                handle_sequence_write(s_prep_seq.buf, s_prep_seq.len);
            }
            if (s_prep_seq.buf) {
                free(s_prep_seq.buf);
                s_prep_seq.buf = NULL;
                s_prep_seq.len = 0;
            }
            break;
        }
        default:
            break;
    }
}

bool huskylens_mapper_start(huskylens_t *husky) {
    if (!husky) {
        ESP_LOGE(TAG, "huskylens nulo");
        return false;
    }

    if (!s_map_mutex) {
        s_map_mutex = xSemaphoreCreateMutex();
        if (!s_map_mutex) {
            ESP_LOGE(TAG, "no se pudo crear mutex");
            return false;
        }
    }
    if (!s_seq_mutex) {
        s_seq_mutex = xSemaphoreCreateMutex();
        if (!s_seq_mutex) {
            ESP_LOGE(TAG, "no se pudo crear mutex secuencia");
            return false;
        }
    }

    if (!s_ble_inited) {
        ESP_LOGI(TAG, "Inicializando BLE para mapper...");
        
        // Build raw advertising data
        size_t name_len = strlen(HUSKY_MAPPER_DEVICE_NAME);
        s_adv_data_raw_size = 9 + name_len;
        if (s_adv_data_raw_size > 31) {
            ESP_LOGE(TAG, "Device name too long for advertising");
            return false;
        }
        
        s_adv_data_raw = (uint8_t *)malloc(s_adv_data_raw_size);
        if (!s_adv_data_raw) {
            ESP_LOGE(TAG, "Failed to allocate advertising data");
            return false;
        }
        
        // Flags (General Discoverable, BR/EDR Not Supported)
        s_adv_data_raw[0] = 0x02;  // length
        s_adv_data_raw[1] = 0x01;  // type: Flags
        s_adv_data_raw[2] = 0x06;  // value: General Discoverable | BR/EDR Not Supported
        
        // Complete 16-bit Service UUID
        s_adv_data_raw[3] = 0x03;  // length
        s_adv_data_raw[4] = 0x03;  // type: Complete List of 16-bit Service UUIDs
        s_adv_data_raw[5] = (husky_mapper_service_uuid_16 >> 0) & 0xFF;  // UUID LSB
        s_adv_data_raw[6] = (husky_mapper_service_uuid_16 >> 8) & 0xFF;  // UUID MSB
        
        // Complete Local Name
        s_adv_data_raw[7] = name_len + 1;  // length
        s_adv_data_raw[8] = 0x09;          // type: Complete Local Name
        memcpy(s_adv_data_raw + 9, HUSKY_MAPPER_DEVICE_NAME, name_len);
        
        ESP_LOGI(TAG, "Advertising data (%d bytes):", s_adv_data_raw_size);
        for (size_t i = 0; i < s_adv_data_raw_size; i++) {
            printf(" %02X", s_adv_data_raw[i]);
        }
        printf("\n");
        
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ret = nvs_flash_erase();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "nvs_flash_erase failed: %s", esp_err_to_name(ret));
                return false;
            }
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
            return false;
        }

        ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "esp_bt_controller_mem_release failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
            return false;
        }
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
            return false;
        }
        ret = esp_bluedroid_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
            return false;
        }
        ret = esp_bluedroid_enable();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
            return false;
        }

        ESP_LOGI(TAG, "Registrando callbacks BLE...");
        esp_ble_gatts_register_callback(gatts_event_handler);
        esp_ble_gap_register_callback(gap_event_handler);
        esp_ble_gatts_app_register(HUSKY_MAPPER_APP_ID);
        esp_ble_gatt_set_local_mtu(200);

        s_ble_inited = true;
        ESP_LOGI(TAG, "BLE mapper inicializado");
    } else {
        ESP_LOGW(TAG, "BLE ya estaba inicializado, omitiendo inicializacion");
    }

    s_husky = husky;
    mapper_nvs_load();
    mapper_nvs_load_seq();

    if (!s_task_running) {
        s_task_running = true;
        uint32_t free_heap = esp_get_free_heap_size();
        ESP_LOGI(TAG, "Heap libre antes de crear task: %u bytes", free_heap);
        if (xTaskCreate(mapper_task, "husky_mapper", 2048, NULL, 5, &s_task) != pdPASS) {
            s_task_running = false;
            ESP_LOGE(TAG, "no se pudo crear task (heap libre: %u)", free_heap);
            return false;
        }
    }

    ESP_LOGI(TAG, "mapper BLE iniciado");
    return true;
}

void huskylens_mapper_stop(void) {
    s_task_running = false;
    s_task = NULL;
    s_husky = NULL;
    cleanup_prep_buffers();
    s_notify_enabled = false;
    s_connected = false;
    s_conn_id = 0xffff;
    
    if (s_adv_data_raw) {
        free(s_adv_data_raw);
        s_adv_data_raw = NULL;
        s_adv_data_raw_size = 0;
    }
    
    ESP_LOGI(TAG, "mapper BLE detenido");
}

bool huskylens_mapper_is_running(void) {
    return s_task_running;
}

bool huskylens_mapper_is_connected(void) {
    return s_connected;
}

bool huskylens_mapper_is_ble_initialized(void) {
    return s_ble_inited;
}

#else

bool huskylens_mapper_start(huskylens_t *husky) {
    (void)husky;
    return false;
}

void huskylens_mapper_stop(void) {
}

uint8_t huskylens_remap_april_tag(uint8_t physical_id) {
    return physical_id;
}

uint8_t huskylens_mapper_get_pairs(uint8_t *pairs, uint8_t max_pairs) {
    (void)pairs;
    (void)max_pairs;
    return 0;
}

bool huskylens_mapper_set_pairs(const uint8_t *pairs, uint8_t count) {
    (void)pairs;
    (void)count;
    return false;
}

void huskylens_mapper_set_conn_cb(void (*cb)(bool connected)) {
    (void)cb;
}

uint8_t huskylens_mapper_get_sequence(uint8_t *seq, uint8_t max_len) {
    (void)seq;
    (void)max_len;
    return 0;
}

bool huskylens_mapper_set_sequence(const uint8_t *seq, uint8_t len) {
    (void)seq;
    (void)len;
    return false;
}

uint8_t huskylens_mapper_get_sequences(uint8_t *seqs, uint8_t *lens, uint8_t max_seqs, uint8_t max_len) {
    (void)seqs;
    (void)lens;
    (void)max_seqs;
    (void)max_len;
    return 0;
}

#endif
