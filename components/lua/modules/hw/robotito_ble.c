/*
 * Copyright (C) 2015 - 2020, IBEROXARXA SERVICIOS INTEGRALES, S.L.
 * Copyright (C) 2015 - 2020, Jaume Olive Petrus (jolive@whitecatboard.org)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *     * The WHITECAT logotype cannot be changed, you can remove it, but you
 *       cannot change it in any way. The WHITECAT logotype is:
 *
 *          /\       /\
 *         /  \_____/  \
 *        /_____________\
 *        W H I T E C A T
 *
 *     * Redistributions in binary form must retain all copyright notices printed
 *       to any local or remote output device. This include any reference to
 *       Lua RTOS, whitecatboard.org, Lua, and other copyright notices that may
 *       appear in the future.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Lua RTOS, Lua robotito BLE module
 * Includes huskylens mapper NVS storage functions.
 */

#include "sdkconfig.h"
#if CONFIG_LUA_RTOS_LUA_USE_ROBOTITO_BLE

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "error.h"
#include "sys.h"
#include "modules.h"
#include <sys/syslog.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/adds.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "robotito_ble.h"
#include "huskylens_mapper.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("*** STACK OVERFLOW en tarea: '%s' ***\n", pcTaskName);
    abort();
}

/* ============================================================================
 * Constants and macros
 * ============================================================================ */

#define MAPPER_EVT_BUFFER_SIZE_BYTES 512

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "ROBOTITO_SPP_SERVER"
#define SPP_SVC_INST_ID             0

#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4

#ifdef SUPPORT_HEARTBEAT
#define ESP_GATT_UUID_SPP_HEARTBEAT         0xABF5
#endif

#define ESP_GATT_UUID_MAPPER_CFG_WRITE      0xABF6
#define ESP_GATT_UUID_MAPPER_CFG_READ       0xABF7
#define ESP_GATT_UUID_MAPPER_SEQ_WRITE      0xABF8
#define ESP_GATT_UUID_MAPPER_SEQ_READ       0xABF9

#define STREAM_BUFFER_SIZE_BYTES CONFIG_ROBOTITO_BLE_LINEBUFFER
#define CHAR_DECLARATION_SIZE    (sizeof(uint8_t))

#define MAPPER_NVS_NAMESPACE  "apriltag"
#define MAPPER_NVS_KEY_MAP    "map"
#define MAPPER_NVS_KEY_SEQ    "seq"

/* ============================================================================
 * Global state
 * ============================================================================ */

bool robotito_ble_initialized = false;
int robotito_ble_rcv_callback = LUA_REFNIL;
int robotito_ble_line_callback = LUA_REFNIL;
int robotito_ble_connect_callback = LUA_REFNIL;
int robotito_ble_disconnect_callback = LUA_REFNIL;
int robotito_ble_mapper_cfg_callback = LUA_REFNIL;
int robotito_ble_mapper_seq_callback = LUA_REFNIL;

char *line_buff = NULL;
int line_buff_last = 0;
RingbufHandle_t stream_buffer_handle;
RingbufHandle_t mapper_evt_buffer_handle;

/* ============================================================================
 * BLE GATT state
 * ============================================================================ */

static const uint16_t spp_service_uuid = 0xABF0;
const char *ble_device_name = NULL;

uint8_t *spp_adv_data = NULL;
size_t spp_adv_data_size = 0;

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
static xQueueHandle cmd_cmd_queue = NULL;

#ifdef SUPPORT_HEARTBEAT
static xQueueHandle cmd_heartbeat_queue = NULL;
static uint8_t heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
static bool enable_heart_ntf = false;
static uint8_t heartbeat_count_num = 0;
#endif

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {0x0,};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ============================================================================
 * SPP receive data linked list
 * ============================================================================ */

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node {
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff {
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num   = 0,
    .buff_size  = 0,
    .first_node = NULL
};

/* Forward declaration */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

/* ============================================================================
 * GATT attribute table
 * ============================================================================ */

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;

#ifdef SUPPORT_HEARTBEAT
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
#endif

static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t  spp_data_receive_val[20] = {0x00};

static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t  spp_data_notify_val[20] = {0x00};
static const uint8_t  spp_data_notify_ccc[2] = {0x00, 0x00};

static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t  spp_command_val[10] = {0x00};

static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t  spp_status_val[10] = {0x00};
static const uint8_t  spp_status_ccc[2] = {0x00, 0x00};

#ifdef SUPPORT_HEARTBEAT
static const uint16_t spp_heart_beat_uuid = ESP_GATT_UUID_SPP_HEARTBEAT;
static const uint8_t  spp_heart_beat_val[2] = {0x00, 0x00};
static const uint8_t  spp_heart_beat_ccc[2] = {0x00, 0x00};
#endif

static const uint16_t mapper_cfg_write_uuid = ESP_GATT_UUID_MAPPER_CFG_WRITE;
static const uint8_t  mapper_cfg_write_val[20] = {0x00};

static const uint16_t mapper_cfg_read_uuid = ESP_GATT_UUID_MAPPER_CFG_READ;
static const uint8_t  mapper_cfg_read_val[20] = {0x00};

static const uint16_t mapper_seq_write_uuid = ESP_GATT_UUID_MAPPER_SEQ_WRITE;
static const uint8_t  mapper_seq_write_val[20] = {0x00};

static const uint16_t mapper_seq_read_uuid = ESP_GATT_UUID_MAPPER_SEQ_READ;
static const uint8_t  mapper_seq_read_val[20] = {0x00};

static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
    /* Service Declaration */
    [SPP_IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

    /* Data receive characteristic */
    [SPP_IDX_SPP_DATA_RECV_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    [SPP_IDX_SPP_DATA_RECV_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

    /* Data notify characteristic */
    [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    [SPP_IDX_SPP_DATA_NTY_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ,
    SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

    [SPP_IDX_SPP_DATA_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

    /* Command characteristic */
    [SPP_IDX_SPP_COMMAND_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    [SPP_IDX_SPP_COMMAND_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t *)spp_command_val}},

    /* Status characteristic */
    [SPP_IDX_SPP_STATUS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    [SPP_IDX_SPP_STATUS_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ,
    SPP_STATUS_MAX_LEN, sizeof(spp_status_val), (uint8_t *)spp_status_val}},

    [SPP_IDX_SPP_STATUS_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

#ifdef SUPPORT_HEARTBEAT
    /* Heartbeat characteristic */
    [SPP_IDX_SPP_HEARTBEAT_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [SPP_IDX_SPP_HEARTBEAT_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_heart_beat_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(spp_heart_beat_val), sizeof(spp_heart_beat_val), (uint8_t *)spp_heart_beat_val}},

    [SPP_IDX_SPP_HEARTBEAT_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_heart_beat_ccc}},
#endif

    /* Mapper config write */
    [SPP_IDX_MAPPER_CFG_WRITE_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    [SPP_IDX_MAPPER_CFG_WRITE_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&mapper_cfg_write_uuid, ESP_GATT_PERM_WRITE,
    256, sizeof(mapper_cfg_write_val), (uint8_t *)mapper_cfg_write_val}},

    /* Mapper config read */
    [SPP_IDX_MAPPER_CFG_READ_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    [SPP_IDX_MAPPER_CFG_READ_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&mapper_cfg_read_uuid, ESP_GATT_PERM_READ,
    256, sizeof(mapper_cfg_read_val), (uint8_t *)mapper_cfg_read_val}},

    /* Mapper sequence write */
    [SPP_IDX_MAPPER_SEQ_WRITE_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    [SPP_IDX_MAPPER_SEQ_WRITE_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&mapper_seq_write_uuid, ESP_GATT_PERM_WRITE,
    512, sizeof(mapper_seq_write_val), (uint8_t *)mapper_seq_write_val}},

    /* Mapper sequence read */
    [SPP_IDX_MAPPER_SEQ_READ_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    [SPP_IDX_MAPPER_SEQ_READ_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&mapper_seq_read_uuid, ESP_GATT_PERM_READ,
    512, sizeof(mapper_seq_read_val), (uint8_t *)mapper_seq_read_val}},
};

/* ============================================================================
 * Lua callback helper
 * ============================================================================ */

static void call_lua_bytes_callback(int callback_ref, const uint8_t *data, size_t len) {
    if (callback_ref == LUA_REFNIL) return;

    lua_State *L = pvGetLuaState();
    if (!L) {
        syslog(LOG_ERR, "call_lua_bytes_callback: Lua state NULL\n");
        return;
    }
    lua_State *TL = lua_newthread(L);
    int tref = luaL_ref(L, LUA_REGISTRYINDEX);
    lua_rawgeti(L, LUA_REGISTRYINDEX, callback_ref);
    lua_xmove(L, TL, 1);

    lua_pushlstring(TL, (const char *)data, len);
    int status = lua_pcall(TL, 1, 0, 0);
    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

    if (status != LUA_OK) {
        const char *msg = lua_tostring(TL, -1);
        lua_writestringerror("error in callback: %s\n", msg);
        lua_pop(TL, 1);
    }
}

/* ============================================================================
 * Mapper event ringbuffer and task
 * ============================================================================ */

static void enqueue_mapper_event(uint8_t type, const uint8_t *data, size_t len) {
    if (!mapper_evt_buffer_handle || !data || len == 0) return;
    size_t total_len = len + 1;
    uint8_t *buf = (uint8_t *)malloc(total_len);
    if (!buf) {
        syslog(LOG_ERR, "mapper_evt: malloc failed\n");
        return;
    }
    buf[0] = type;
    memcpy(&buf[1], data, len);
    UBaseType_t res = xRingbufferSend(mapper_evt_buffer_handle, buf, total_len, pdMS_TO_TICKS(100));
    free(buf);
    if (res != pdTRUE) {
        syslog(LOG_ERR, "mapper_evt: ringbuffer full\n");
    }
}

static void mapper_evt_task(void *arg)
{
    for (;;) {
        size_t item_size;
        uint8_t *item_ptr = (uint8_t *)xRingbufferReceiveUpTo(
            mapper_evt_buffer_handle, &item_size, portMAX_DELAY, MAPPER_EVT_BUFFER_SIZE_BYTES);
        if (item_ptr != NULL && item_size >= 2) {
            uint8_t type = item_ptr[0];
            const uint8_t *payload = &item_ptr[1];
            size_t payload_len = item_size - 1;
            if (type == 1) {
                call_lua_bytes_callback(robotito_ble_mapper_cfg_callback, payload, payload_len);
            } else if (type == 2) {
                call_lua_bytes_callback(robotito_ble_mapper_seq_callback, payload, payload_len);
            }
            vRingbufferReturnItem(mapper_evt_buffer_handle, (void *)item_ptr);
        }
    }
    vTaskDelete(NULL);
}

/* ============================================================================
 * GATT helper functions
 * ============================================================================ */

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    for (int i = 0; i < SPP_IDX_NB; i++) {
        if (handle == spp_handle_table[i]) {
            return i;
        }
    }
    return 0xff;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));
    if (temp_spp_recv_data_node_p1 == NULL) {
        syslog(LOG_INFO, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL) {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    if (SppRecvDataBuff.node_num == 0) {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
    }
    SppRecvDataBuff.node_num++;
    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;
    while (temp_spp_recv_data_node_p1 != NULL) {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }
    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
    temp_spp_recv_data_node_p1 = NULL;
    temp_spp_recv_data_node_p2 = NULL;
}

/* ============================================================================
 * SPP receive task - processes incoming BLE data via ringbuffer
 * ============================================================================ */

void spp_rcv_task(void *arg)
{
    for (;;) {
        size_t item_size;
        char *item_ptr = (char *)xRingbufferReceiveUpTo(stream_buffer_handle,
                &item_size, portMAX_DELAY, STREAM_BUFFER_SIZE_BYTES);

        if (item_ptr != NULL) {
            if (robotito_ble_rcv_callback != LUA_REFNIL) {
                lua_State *L = pvGetLuaState();
                lua_State *TL = lua_newthread(L);
                int tref = luaL_ref(L, LUA_REGISTRYINDEX);
                lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_rcv_callback);
                lua_xmove(L, TL, 1);

                lua_pushlstring(TL, item_ptr, item_size);
                int status = lua_pcall(TL, 1, 0, 0);
                luaL_unref(TL, LUA_REGISTRYINDEX, tref);

                if (status != LUA_OK) {
                    const char *msg = lua_tostring(TL, -1);
                    lua_writestringerror("error in rcv callback: %s\n", msg);
                    lua_pop(TL, 1);
                }
            }

            if (robotito_ble_line_callback != LUA_REFNIL) {
                if (line_buff_last + item_size > CONFIG_ROBOTITO_BLE_LINEBUFFER) {
                    /* Buffer overflow: flush current buffer as error */
                    lua_State *L = pvGetLuaState();
                    lua_State *TL = lua_newthread(L);
                    int tref = luaL_ref(L, LUA_REGISTRYINDEX);
                    lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_line_callback);
                    lua_xmove(L, TL, 1);

                    lua_pushnil(TL);
                    lua_pushlstring(TL, (char *)line_buff, line_buff_last);
                    line_buff_last = 0;
                    int status = lua_pcall(TL, 2, 0, 0);
                    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

                    if (status != LUA_OK) {
                        const char *msg = lua_tostring(TL, -1);
                        lua_writestringerror("error in line callback: %s\n", msg);
                        lua_pop(TL, 1);
                    }
                }

                memcpy(line_buff + line_buff_last, item_ptr, item_size);
                int start_search = line_buff_last;
                line_buff_last += item_size;
                char *pos = memchr(line_buff + start_search, '\n', line_buff_last - start_search);

                while (pos) {
                    lua_State *L = pvGetLuaState();
                    lua_State *TL = lua_newthread(L);
                    int tref = luaL_ref(L, LUA_REGISTRYINDEX);
                    lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_line_callback);
                    lua_xmove(L, TL, 1);

                    lua_pushlstring(TL, line_buff, pos - line_buff);
                    memcpy(line_buff, pos + 1, line_buff + line_buff_last - pos - 1);
                    int status = lua_pcall(TL, 1, 0, 0);
                    luaL_unref(TL, LUA_REGISTRYINDEX, tref);

                    if (status != LUA_OK) {
                        const char *msg = lua_tostring(TL, -1);
                        lua_writestringerror("error in line callback: %s\n", msg);
                        lua_pop(TL, 1);
                    }

                    line_buff_last -= (pos - line_buff + 1);
                    pos = memchr(line_buff, '\n', line_buff_last);
                }
            }

            vRingbufferReturnItem(stream_buffer_handle, (void *)item_ptr);
        } else {
            printf("Failed to receive item\n");
        }
    }
    vTaskDelete(NULL);
}

#ifdef SUPPORT_HEARTBEAT
void spp_heartbeat_task(void *arg)
{
    uint16_t cmd_id;

    for (;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_heartbeat_queue, &cmd_id, portMAX_DELAY)) {
            while (1) {
                heartbeat_count_num++;
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                if ((heartbeat_count_num > 3) && is_connected) {
                    esp_ble_gap_disconnect(spp_remote_bda);
                }
                if (is_connected && enable_heart_ntf) {
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                        spp_handle_table[SPP_IDX_SPP_HEARTBEAT_VAL],
                        sizeof(heartbeat_s), heartbeat_s, false);
                } else if (!is_connected) {
                    break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}
#endif

void spp_cmd_task(void *arg)
{
    uint8_t *cmd_id;

    for (;;) {
        if (xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY)) {
            printf("command: ");
            for (int i = 0; i < strlen((char *)cmd_id); i++) {
                printf("%c", cmd_id[i]);
            }
            printf("\n");
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void spp_task_init(void)
{
#ifdef SUPPORT_HEARTBEAT
    cmd_heartbeat_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_heartbeat_task, "spp_heartbeat_task", 2048, NULL, 10, NULL);
#endif
    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);
}

/* ============================================================================
 * GAP event handler
 * ============================================================================ */

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    syslog(LOG_ERR, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            syslog(LOG_ERR, "Advertising start failed: %s\n",
                esp_err_to_name(param->adv_start_cmpl.status));
        } else {
            syslog(LOG_ERR, "Advertising start success\n");
        }
        break;
    default:
        break;
    }
}

/* ============================================================================
 * HuskyLens Mapper - internal state and NVS storage
 * ============================================================================ */

typedef struct {
    uint8_t physical_id;
    uint8_t logical_id;
} mapper_entry_t;

typedef struct {
    uint8_t count;
    mapper_entry_t entries[HUSKYLENS_MAP_MAX];
} mapper_table_t;

typedef struct {
    uint8_t count;
    uint8_t lens[HUSKYLENS_SEQ_MAX_SEQS];
    uint8_t ids[HUSKYLENS_SEQ_MAX_SEQS][HUSKYLENS_SEQ_MAX];
} mapper_seq_list_t;

static mapper_table_t    s_mapper_map;
static SemaphoreHandle_t s_mapper_map_mutex = NULL;
static mapper_seq_list_t s_mapper_seqs;
static SemaphoreHandle_t s_mapper_seq_mutex = NULL;
static bool s_mapper_map_loaded = false;
static bool s_mapper_seq_loaded = false;

static void mapper_map_lock(void) {
    if (s_mapper_map_mutex) xSemaphoreTake(s_mapper_map_mutex, portMAX_DELAY);
}
static void mapper_map_unlock(void) {
    if (s_mapper_map_mutex) xSemaphoreGive(s_mapper_map_mutex);
}
static void mapper_seq_lock(void) {
    if (s_mapper_seq_mutex) xSemaphoreTake(s_mapper_seq_mutex, portMAX_DELAY);
}
static void mapper_seq_unlock(void) {
    if (s_mapper_seq_mutex) xSemaphoreGive(s_mapper_seq_mutex);
}

static bool mapper_mutexes_init(void) {
    if (!s_mapper_map_mutex) {
        s_mapper_map_mutex = xSemaphoreCreateMutex();
        if (!s_mapper_map_mutex) {
            syslog(LOG_ERR, "mapper: failed to create map mutex\n");
            return false;
        }
    }
    if (!s_mapper_seq_mutex) {
        s_mapper_seq_mutex = xSemaphoreCreateMutex();
        if (!s_mapper_seq_mutex) {
            syslog(LOG_ERR, "mapper: failed to create sequence mutex\n");
            return false;
        }
    }
    return true;
}

/* --- Map operations --- */

static void mapper_map_clear(void) {
    mapper_map_lock();
    s_mapper_map.count = 0;
    memset(s_mapper_map.entries, 0, sizeof(s_mapper_map.entries));
    mapper_map_unlock();
}

static void mapper_map_set_from_pairs(const uint8_t *data, uint8_t count) {
    mapper_map_lock();
    s_mapper_map.count = 0;
    memset(s_mapper_map.entries, 0, sizeof(s_mapper_map.entries));
    for (uint8_t i = 0; i < count && i < HUSKYLENS_MAP_MAX; i++) {
        uint8_t physical = data[i * 2];
        uint8_t logical  = data[i * 2 + 1];
        bool replaced = false;
        for (uint8_t j = 0; j < s_mapper_map.count; j++) {
            if (s_mapper_map.entries[j].physical_id == physical) {
                s_mapper_map.entries[j].logical_id = logical;
                replaced = true;
                break;
            }
        }
        if (!replaced && s_mapper_map.count < HUSKYLENS_MAP_MAX) {
            s_mapper_map.entries[s_mapper_map.count].physical_id = physical;
            s_mapper_map.entries[s_mapper_map.count].logical_id  = logical;
            s_mapper_map.count++;
        }
    }
    mapper_map_unlock();
}

static uint8_t mapper_map_pack(uint8_t *out, size_t out_len) {
    mapper_map_lock();
    uint8_t count  = s_mapper_map.count;
    size_t  needed = (size_t)(1 + count * 2);
    if (out_len < needed) {
        count  = (uint8_t)((out_len - 1) / 2);
        needed = (size_t)(1 + count * 2);
    }
    out[0] = count;
    for (uint8_t i = 0; i < count; i++) {
        out[1 + (i * 2)]     = s_mapper_map.entries[i].physical_id;
        out[1 + (i * 2) + 1] = s_mapper_map.entries[i].logical_id;
    }
    mapper_map_unlock();
    return (uint8_t)needed;
}

/* --- Sequence operations --- */

static void mapper_seq_clear(void) {
    mapper_seq_lock();
    s_mapper_seqs.count = 0;
    memset(s_mapper_seqs.lens, 0, sizeof(s_mapper_seqs.lens));
    memset(s_mapper_seqs.ids,  0, sizeof(s_mapper_seqs.ids));
    mapper_seq_unlock();
}

static void mapper_seq_set_from_list(const uint8_t *data, uint8_t len) {
    if (!data) return;
    if (len > HUSKYLENS_SEQ_MAX) len = HUSKYLENS_SEQ_MAX;
    mapper_seq_lock();
    s_mapper_seqs.count = 1;
    memset(s_mapper_seqs.lens, 0, sizeof(s_mapper_seqs.lens));
    memset(s_mapper_seqs.ids,  0, sizeof(s_mapper_seqs.ids));
    s_mapper_seqs.lens[0] = len;
    if (len > 0) memcpy(s_mapper_seqs.ids[0], data, len);
    mapper_seq_unlock();
}

static void mapper_seq_set_from_buffer(const uint8_t *data, size_t len) {
    if (!data || len < 1) return;

    uint8_t seq_count = data[0];
    if (seq_count > HUSKYLENS_SEQ_MAX_SEQS) seq_count = HUSKYLENS_SEQ_MAX_SEQS;

    mapper_seq_lock();
    s_mapper_seqs.count = 0;
    memset(s_mapper_seqs.lens, 0, sizeof(s_mapper_seqs.lens));
    memset(s_mapper_seqs.ids,  0, sizeof(s_mapper_seqs.ids));

    size_t offset = 1;
    for (uint8_t i = 0; i < seq_count && offset < len; i++) {
        uint8_t seq_len = data[offset++];
        if (seq_len > HUSKYLENS_SEQ_MAX) seq_len = HUSKYLENS_SEQ_MAX;

        size_t remaining = len - offset;
        uint8_t copy_len = seq_len;
        if (copy_len > remaining) copy_len = (uint8_t)remaining;

        if (copy_len > 0) {
            memcpy(s_mapper_seqs.ids[i], &data[offset], copy_len);
            offset += copy_len;
        }

        s_mapper_seqs.lens[i] = copy_len;
        s_mapper_seqs.count   = (uint8_t)(i + 1);

        if (copy_len < seq_len) break;
    }
    mapper_seq_unlock();
}

static uint8_t mapper_seq_pack(uint8_t *out, size_t out_len) {
    mapper_seq_lock();
    uint8_t count  = s_mapper_seqs.count;
    size_t  offset = 0;
    if (out_len == 0) { mapper_seq_unlock(); return 0; }
    out[offset++] = count;
    for (uint8_t i = 0; i < count && i < HUSKYLENS_SEQ_MAX_SEQS; i++) {
        uint8_t len = s_mapper_seqs.lens[i];
        if (len > HUSKYLENS_SEQ_MAX) len = HUSKYLENS_SEQ_MAX;
        if (offset + 1 > out_len) break;
        out[offset++] = len;
        size_t copy_len = len;
        if (offset + copy_len > out_len) copy_len = out_len - offset;
        if (copy_len > 0) {
            memcpy(&out[offset], s_mapper_seqs.ids[i], copy_len);
            offset += copy_len;
        }
        if (copy_len < len) break;
    }
    mapper_seq_unlock();
    return (uint8_t)offset;
}

/* --- NVS: load map --- */

static esp_err_t mapper_nvs_load(void) {
    nvs_handle handle;
    size_t     size = 0;
    esp_err_t  err  = nvs_open(MAPPER_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        mapper_map_clear();
        s_mapper_map_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_open failed: %s\n", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_blob(handle, MAPPER_NVS_KEY_MAP, NULL, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        mapper_map_clear();
        s_mapper_map_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK || size < 1) {
        syslog(LOG_ERR, "mapper nvs_get_blob size failed: %s\n", esp_err_to_name(err));
        nvs_close(handle);
        mapper_map_clear();
        s_mapper_map_loaded = true;
        return err;
    }

    uint8_t *buf = (uint8_t *)malloc(size);
    if (!buf) { nvs_close(handle); return ESP_ERR_NO_MEM; }

    err = nvs_get_blob(handle, MAPPER_NVS_KEY_MAP, buf, &size);
    nvs_close(handle);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_get_blob failed: %s\n", esp_err_to_name(err));
        free(buf);
        mapper_map_clear();
        s_mapper_map_loaded = true;
        return err;
    }

    uint8_t count     = buf[0];
    uint8_t max_pairs = (uint8_t)((size - 1) / 2);
    if (count > max_pairs) count = max_pairs;
    mapper_map_set_from_pairs(&buf[1], count);
    free(buf);
    s_mapper_map_loaded = true;
    syslog(LOG_INFO, "mapper: map loaded from NVS (%u entries)\n", count);
    return ESP_OK;
}

/* --- NVS: save map --- */

static esp_err_t mapper_nvs_save(void) {
    nvs_handle handle;
    esp_err_t  err = nvs_open(MAPPER_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_open (rw) failed: %s\n", esp_err_to_name(err));
        return err;
    }

    uint8_t buf[1 + (HUSKYLENS_MAP_MAX * 2)];
    uint8_t len = mapper_map_pack(buf, sizeof(buf));
    err = nvs_set_blob(handle, MAPPER_NVS_KEY_MAP, buf, len);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_set_blob failed: %s\n", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_commit failed: %s\n", esp_err_to_name(err));
        return err;
    }

    syslog(LOG_INFO, "mapper: map saved to NVS (%u bytes)\n", len);
    return ESP_OK;
}

/* --- NVS: load sequences --- */

static esp_err_t mapper_nvs_load_seq(void) {
    nvs_handle handle;
    size_t     size = 0;
    esp_err_t  err  = nvs_open(MAPPER_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        mapper_seq_clear();
        s_mapper_seq_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_open seq failed: %s\n", esp_err_to_name(err));
        return err;
    }

    err = nvs_get_blob(handle, MAPPER_NVS_KEY_SEQ, NULL, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(handle);
        mapper_seq_clear();
        s_mapper_seq_loaded = true;
        return ESP_OK;
    }
    if (err != ESP_OK || size < 1) {
        syslog(LOG_ERR, "mapper nvs_get_blob seq size failed: %s\n", esp_err_to_name(err));
        nvs_close(handle);
        mapper_seq_clear();
        s_mapper_seq_loaded = true;
        return err;
    }

    uint8_t *buf = (uint8_t *)malloc(size);
    if (!buf) { nvs_close(handle); return ESP_ERR_NO_MEM; }

    err = nvs_get_blob(handle, MAPPER_NVS_KEY_SEQ, buf, &size);
    nvs_close(handle);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_get_blob seq failed: %s\n", esp_err_to_name(err));
        free(buf);
        mapper_seq_clear();
        s_mapper_seq_loaded = true;
        return err;
    }

    uint8_t seq_count = buf[0];
    if (seq_count > HUSKYLENS_SEQ_MAX_SEQS) seq_count = HUSKYLENS_SEQ_MAX_SEQS;

    mapper_seq_lock();
    s_mapper_seqs.count = 0;
    memset(s_mapper_seqs.lens, 0, sizeof(s_mapper_seqs.lens));
    memset(s_mapper_seqs.ids,  0, sizeof(s_mapper_seqs.ids));

    size_t offset = 1;
    for (uint8_t i = 0; i < seq_count && offset < size; i++) {
        uint8_t slen      = buf[offset++];
        if (slen > HUSKYLENS_SEQ_MAX) slen = HUSKYLENS_SEQ_MAX;
        size_t  remaining = size - offset;
        uint8_t copy_len  = slen;
        if (copy_len > remaining) copy_len = (uint8_t)remaining;
        if (copy_len > 0) {
            memcpy(s_mapper_seqs.ids[i], &buf[offset], copy_len);
            offset += copy_len;
        }
        s_mapper_seqs.lens[i] = copy_len;
        s_mapper_seqs.count   = (uint8_t)(i + 1);
        if (copy_len < slen) break;
    }
    mapper_seq_unlock();

    free(buf);
    s_mapper_seq_loaded = true;
    syslog(LOG_INFO, "mapper: sequences loaded from NVS (%u)\n", s_mapper_seqs.count);
    return ESP_OK;
}

/* --- NVS: save sequences --- */

static esp_err_t mapper_nvs_save_seq(void) {
    nvs_handle handle;
    esp_err_t  err = nvs_open(MAPPER_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_open seq (rw) failed: %s\n", esp_err_to_name(err));
        return err;
    }

    uint8_t buf[1 + (HUSKYLENS_SEQ_MAX_SEQS * (1 + HUSKYLENS_SEQ_MAX))];
    uint8_t len = mapper_seq_pack(buf, sizeof(buf));
    err = nvs_set_blob(handle, MAPPER_NVS_KEY_SEQ, buf, len);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_set_blob seq failed: %s\n", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        syslog(LOG_ERR, "mapper nvs_commit seq failed: %s\n", esp_err_to_name(err));
        return err;
    }

    syslog(LOG_INFO, "mapper: sequence saved to NVS (%u bytes)\n", len);
    return ESP_OK;
}

/* ============================================================================
 * GATTS profile event handler
 * ============================================================================ */

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    syslog(LOG_INFO, "event = %x\n", event);
    switch (event) {
    case ESP_GATTS_REG_EVT:
        syslog(LOG_INFO, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_set_device_name(ble_device_name);

        syslog(LOG_INFO, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, spp_adv_data_size);

        syslog(LOG_INFO, "%s %d\n", __func__, __LINE__);
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;

    case ESP_GATTS_READ_EVT:
        res = find_char_and_desr_index(p_data->read.handle);
        if (res == SPP_IDX_SPP_STATUS_VAL) {
            /* Client read the status characteristic - no action needed */
        } else if (res == SPP_IDX_MAPPER_CFG_READ_VAL) {
            uint8_t *buf = (uint8_t *)malloc(256);
            esp_gatt_rsp_t *rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));

            if (buf == NULL || rsp == NULL) {
                syslog(LOG_ERR, "Failed to allocate memory for mapper config read\n");
                if (buf) free(buf);
                if (rsp) free(rsp);
                esp_ble_gatts_send_response(gatts_if, p_data->read.conn_id,
                                            p_data->read.trans_id, ESP_GATT_OUT_OF_RANGE, NULL);
                break;
            }

            uint8_t count = huskylens_mapper_get_pairs(&buf[1], 100);
            buf[0] = count;
            uint16_t len = 1 + (count * 2);

            memset(rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp->attr_value.handle = p_data->read.handle;
            rsp->attr_value.len = len;
            memcpy(rsp->attr_value.value, buf, len);
            esp_ble_gatts_send_response(gatts_if, p_data->read.conn_id,
                                        p_data->read.trans_id, ESP_GATT_OK, rsp);
            syslog(LOG_INFO, "Mapper config read: %d pairs\n", count);

            free(buf);
            free(rsp);
        } else if (res == SPP_IDX_MAPPER_SEQ_READ_VAL) {
            uint8_t *buf = (uint8_t *)malloc(512);
            uint8_t *lens = (uint8_t *)malloc(HUSKYLENS_SEQ_MAX_SEQS);
            uint8_t *seqs = (uint8_t *)malloc(HUSKYLENS_SEQ_MAX_SEQS * HUSKYLENS_SEQ_MAX);
            esp_gatt_rsp_t *rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));

            if (buf == NULL || lens == NULL || seqs == NULL || rsp == NULL) {
                syslog(LOG_ERR, "Failed to allocate memory for mapper sequence read\n");
                if (buf) free(buf);
                if (lens) free(lens);
                if (seqs) free(seqs);
                if (rsp) free(rsp);
                esp_ble_gatts_send_response(gatts_if, p_data->read.conn_id,
                                            p_data->read.trans_id, ESP_GATT_OUT_OF_RANGE, NULL);
                break;
            }

            uint8_t seq_count = huskylens_mapper_get_sequences(seqs, lens,
                                                                HUSKYLENS_SEQ_MAX_SEQS,
                                                                HUSKYLENS_SEQ_MAX);

            /* Pack: [count][len1][ids...][len2][ids...]... */
            uint16_t offset = 0;
            buf[offset++] = seq_count;
            for (uint8_t i = 0; i < seq_count && offset < 512; i++) {
                buf[offset++] = lens[i];
                for (uint8_t j = 0; j < lens[i] && offset < 512; j++) {
                    buf[offset++] = seqs[i * HUSKYLENS_SEQ_MAX + j];
                }
            }

            memset(rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp->attr_value.handle = p_data->read.handle;
            rsp->attr_value.len = offset;
            memcpy(rsp->attr_value.value, buf, offset);
            esp_ble_gatts_send_response(gatts_if, p_data->read.conn_id,
                                        p_data->read.trans_id, ESP_GATT_OK, rsp);
            syslog(LOG_INFO, "Mapper sequence read: %d sequences\n", seq_count);

            free(buf);
            free(lens);
            free(seqs);
            free(rsp);
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        res = find_char_and_desr_index(p_data->write.handle);
        if (p_data->write.is_prep == false) {
            syslog(LOG_INFO, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);

            if (res == SPP_IDX_SPP_COMMAND_VAL) {
                uint8_t *spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                if (spp_cmd_buff == NULL) {
                    syslog(LOG_ERR, "%s malloc failed\n", __func__);
                    break;
                }
                memset(spp_cmd_buff, 0x0, (spp_mtu_size - 3));
                memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);

            } else if (res == SPP_IDX_SPP_DATA_NTF_CFG) {
                if (p_data->write.len == 2 && p_data->write.value[0] == 0x01 && p_data->write.value[1] == 0x00) {
                    enable_data_ntf = true;
                } else if (p_data->write.len == 2 && p_data->write.value[0] == 0x00 && p_data->write.value[1] == 0x00) {
                    enable_data_ntf = false;
                }
            }
#ifdef SUPPORT_HEARTBEAT
            else if (res == SPP_IDX_SPP_HEARTBEAT_CFG) {
                if (p_data->write.len == 2 && p_data->write.value[0] == 0x01 && p_data->write.value[1] == 0x00) {
                    enable_heart_ntf = true;
                } else if (p_data->write.len == 2 && p_data->write.value[0] == 0x00 && p_data->write.value[1] == 0x00) {
                    enable_heart_ntf = false;
                }
            } else if (res == SPP_IDX_SPP_HEARTBEAT_VAL) {
                if (p_data->write.len == sizeof(heartbeat_s) &&
                    memcmp(heartbeat_s, p_data->write.value, sizeof(heartbeat_s)) == 0) {
                    heartbeat_count_num = 0;
                }
            }
#endif
            else if (res == SPP_IDX_MAPPER_CFG_WRITE_VAL) {
                if (p_data->write.len >= 1) {
                    uint8_t count = p_data->write.value[0];
                    uint16_t expected = 1 + (count * 2);
                    if (expected <= p_data->write.len) {
                        if (!mapper_mutexes_init()) {
                            syslog(LOG_ERR, "Mapper: mutexes not initialized\n");
                        } else {
                            mapper_map_set_from_pairs(&p_data->write.value[1], count);
                            s_mapper_map_loaded = true;
                            huskylens_mapper_set_pairs(&p_data->write.value[1], count);

                            if (mapper_nvs_save() == ESP_OK) {
                                syslog(LOG_INFO, "Mapper config written: %d pairs\n", count);
                            } else {
                                syslog(LOG_ERR, "Mapper: error saving map to NVS\n");
                            }
                        }
                        enqueue_mapper_event(1, p_data->write.value, p_data->write.len);
                    } else {
                        syslog(LOG_ERR, "Mapper config invalid length: got %d, need %d\n",
                               p_data->write.len, expected);
                    }
                }

            } else if (res == SPP_IDX_MAPPER_SEQ_WRITE_VAL) {
                if (p_data->write.len >= 1) {
                    uint8_t seq_count = p_data->write.value[0];

                    /* Detect legacy single sequence format: [len, id1, id2, ...] */
                    if (p_data->write.len == (1 + seq_count) &&
                        seq_count <= HUSKYLENS_SEQ_MAX) {
                        if (!mapper_mutexes_init()) {
                            syslog(LOG_ERR, "Mapper: mutexes not initialized\n");
                        } else {
                            mapper_seq_set_from_list(&p_data->write.value[1], seq_count);
                            s_mapper_seq_loaded = true;
                            huskylens_mapper_set_sequence(&p_data->write.value[1], seq_count);

                            if (mapper_nvs_save_seq() == ESP_OK) {
                                syslog(LOG_INFO, "Mapper single sequence written: %d IDs\n", seq_count);
                            } else {
                                syslog(LOG_ERR, "Mapper: error saving sequence to NVS\n");
                            }
                        }
                        enqueue_mapper_event(2, p_data->write.value, p_data->write.len);
                    } else {
                        /* Multi-sequence format: [count, len1, ids..., len2, ids...] */
                        if (!mapper_mutexes_init()) {
                            syslog(LOG_ERR, "Mapper: mutexes not initialized\n");
                        } else {
                            mapper_seq_set_from_buffer(p_data->write.value, p_data->write.len);
                            s_mapper_seq_loaded = true;

                            if (mapper_nvs_save_seq() == ESP_OK) {
                                syslog(LOG_INFO, "Mapper multi-sequence written: %d sequences\n", seq_count);
                            } else {
                                syslog(LOG_ERR, "Mapper: error saving sequences to NVS\n");
                            }
                        }
                        enqueue_mapper_event(2, p_data->write.value, p_data->write.len);
                    }
                }

            } else if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
#ifdef SPP_DEBUG_MODE
                printf("Arrived %d bytes: ", p_data->write.len);
                for (int i = 0; i < p_data->write.len; i++) {
                    printf("%c", p_data->write.value[i]);
                }
                printf("\n");
#endif
                UBaseType_t send_res = xRingbufferSend(stream_buffer_handle,
                            (void *)p_data->write.value,
                            p_data->write.len,
                            pdMS_TO_TICKS(100));
                if (send_res != pdTRUE) {
                    printf("Failed to send item\n");
                }
            }

        } else if (p_data->write.is_prep == true && res == SPP_IDX_SPP_DATA_RECV_VAL) {
            syslog(LOG_INFO, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            store_wr_buffer(p_data);
        }
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        syslog(LOG_INFO, "ESP_GATTS_EXEC_WRITE_EVT\n");
        free_write_buffer();
        break;

    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;

    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        syslog(LOG_INFO, "Client connected, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
            p_data->connect.remote_bda[0], p_data->connect.remote_bda[1],
            p_data->connect.remote_bda[2], p_data->connect.remote_bda[3],
            p_data->connect.remote_bda[4], p_data->connect.remote_bda[5]);
        if (robotito_ble_connect_callback != LUA_REFNIL) {
            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                p_data->connect.remote_bda[0], p_data->connect.remote_bda[1],
                p_data->connect.remote_bda[2], p_data->connect.remote_bda[3],
                p_data->connect.remote_bda[4], p_data->connect.remote_bda[5]);
            lua_State *L = pvGetLuaState();
            lua_State *TL = lua_newthread(L);
            int tref = luaL_ref(L, LUA_REGISTRYINDEX);
            lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_connect_callback);
            lua_xmove(L, TL, 1);
            lua_pushstring(TL, mac_str);
            lua_pcall(TL, 1, 0, 0);
            luaL_unref(L, LUA_REGISTRYINDEX, tref);
        }
#ifdef SUPPORT_HEARTBEAT
        {
            uint16_t cmd = 0;
            xQueueSend(cmd_heartbeat_queue, &cmd, 10 / portTICK_PERIOD_MS);
        }
#endif
        break;

    case ESP_GATTS_DISCONNECT_EVT: {
        const char *disconnect_reason;
        switch ((int)p_data->disconnect.reason) {
            case 0x00: disconnect_reason = "OK (local)"; break;
            case 0x01: disconnect_reason = "L2CAP failure"; break;
            case 0x08: disconnect_reason = "timeout / connection lost"; break;
            case 0x13: disconnect_reason = "client closed connection"; break;
            case 0x16: disconnect_reason = "host closed connection"; break;
            case 0x22: disconnect_reason = "LMP / LL response timeout"; break;
            case 0x3e: disconnect_reason = "failed to establish connection"; break;
            case 0xff: disconnect_reason = "no connection"; break;
            default:   disconnect_reason = "unknown reason"; break;
        }
        syslog(LOG_INFO, "Client disconnected, reason: 0x%02x (%s)\n",
            (int)p_data->disconnect.reason, disconnect_reason);
        if (robotito_ble_disconnect_callback != LUA_REFNIL) {
            lua_State *L = pvGetLuaState();
            lua_State *TL = lua_newthread(L);
            int tref = luaL_ref(L, LUA_REGISTRYINDEX);
            lua_rawgeti(L, LUA_REGISTRYINDEX, robotito_ble_disconnect_callback);
            lua_xmove(L, TL, 1);
            lua_pushstring(TL, disconnect_reason);
            lua_pcall(TL, 1, 0, 0);
            luaL_unref(L, LUA_REGISTRYINDEX, tref);
        }
        is_connected = false;
        enable_data_ntf = false;
#ifdef SUPPORT_HEARTBEAT
        enable_heart_ntf = false;
        heartbeat_count_num = 0;
#endif
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    }

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        syslog(LOG_INFO, "The number handle =%x\n", param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            syslog(LOG_ERR, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != SPP_IDX_NB) {
            syslog(LOG_ERR, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)",
                param->add_attr_tab.num_handle, SPP_IDX_NB);
        } else {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;

    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    syslog(LOG_INFO, "EVT %d, gatts if %d\n", event, gatts_if);

    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            syslog(LOG_INFO, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    for (int idx = 0; idx < SPP_PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == spp_profile_tab[idx].gatts_if) {
            if (spp_profile_tab[idx].gatts_cb) {
                spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

/* ============================================================================
 * Lua API: mapper functions
 * ============================================================================ */

static bool s_mapper_initialized = false;
     
static int robotito_ble_mapper_init(lua_State *L) {
    if (s_mapper_initialized) {
        lua_pushboolean(L, true);
        return 1;
    }
    if (!mapper_mutexes_init()) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: failed to create mutexes");
    }
    mapper_nvs_load();
    mapper_nvs_load_seq();
    s_mapper_initialized = true;
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_mapper_get_pairs(lua_State *L) {
    if (!s_mapper_map_loaded) mapper_nvs_load();

    mapper_map_lock();
    lua_newtable(L);
    for (uint8_t i = 0; i < s_mapper_map.count; i++) {
        lua_newtable(L);
        lua_pushinteger(L, s_mapper_map.entries[i].physical_id);
        lua_rawseti(L, -2, 1);
        lua_pushinteger(L, s_mapper_map.entries[i].logical_id);
        lua_rawseti(L, -2, 2);
        lua_rawseti(L, -2, (int)(i + 1));
    }
    mapper_map_unlock();
    return 1;
}

static int robotito_ble_mapper_set_pairs(lua_State *L) {
    luaL_checktype(L, 1, LUA_TTABLE);

    uint8_t pairs[HUSKYLENS_MAP_MAX * 2];
    uint8_t count = 0;

    lua_pushnil(L);
    while (lua_next(L, 1) != 0 && count < HUSKYLENS_MAP_MAX) {
        luaL_checktype(L, -1, LUA_TTABLE);
        lua_rawgeti(L, -1, 1);
        lua_rawgeti(L, -2, 2);
        pairs[count * 2]     = (uint8_t)luaL_checkinteger(L, -2);
        pairs[count * 2 + 1] = (uint8_t)luaL_checkinteger(L, -1);
        lua_pop(L, 2);
        count++;
        lua_pop(L, 1);
    }

    if (!mapper_mutexes_init()) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: mutexes not initialized");
        return 2;
    }

    mapper_map_set_from_pairs(pairs, count);
    s_mapper_map_loaded = true;

    if (mapper_nvs_save() != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: error saving map to NVS");
        return 2;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_mapper_get_sequence(lua_State *L) {
    if (!s_mapper_seq_loaded) mapper_nvs_load_seq();

    mapper_seq_lock();
    uint8_t len = (s_mapper_seqs.count > 0) ? s_mapper_seqs.lens[0] : 0;
    lua_newtable(L);
    for (uint8_t i = 0; i < len; i++) {
        lua_pushinteger(L, s_mapper_seqs.ids[0][i]);
        lua_rawseti(L, -2, (int)(i + 1));
    }
    mapper_seq_unlock();
    return 1;
}

static int robotito_ble_mapper_set_sequence(lua_State *L) {
    luaL_checktype(L, 1, LUA_TTABLE);

    uint8_t seq[HUSKYLENS_SEQ_MAX];
    uint8_t len = 0;

    lua_pushnil(L);
    while (lua_next(L, 1) != 0 && len < HUSKYLENS_SEQ_MAX) {
        seq[len++] = (uint8_t)luaL_checkinteger(L, -1);
        lua_pop(L, 1);
    }

    if (!mapper_mutexes_init()) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: mutexes not initialized");
        return 2;
    }

    mapper_seq_set_from_list(seq, len);
    s_mapper_seq_loaded = true;

    if (mapper_nvs_save_seq() != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: error saving sequence to NVS");
        return 2;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_mapper_get_sequences(lua_State *L) {
    if (!s_mapper_seq_loaded) mapper_nvs_load_seq();

    mapper_seq_lock();
    lua_newtable(L);
    for (uint8_t i = 0; i < s_mapper_seqs.count; i++) {
        lua_newtable(L);
        uint8_t len = s_mapper_seqs.lens[i];
        for (uint8_t j = 0; j < len; j++) {
            lua_pushinteger(L, s_mapper_seqs.ids[i][j]);
            lua_rawseti(L, -2, (int)(j + 1));
        }
        lua_rawseti(L, -2, (int)(i + 1));
    }
    mapper_seq_unlock();
    return 1;
}

static int robotito_ble_mapper_set_sequences(lua_State *L) {
    luaL_checktype(L, 1, LUA_TTABLE);

    uint8_t seq_count = 0;
    uint8_t lens[HUSKYLENS_SEQ_MAX_SEQS];
    uint8_t ids[HUSKYLENS_SEQ_MAX_SEQS][HUSKYLENS_SEQ_MAX];

    lua_pushnil(L);
    while (lua_next(L, 1) != 0 && seq_count < HUSKYLENS_SEQ_MAX_SEQS) {
        luaL_checktype(L, -1, LUA_TTABLE);
        uint8_t len = 0;
        lua_pushnil(L);
        while (lua_next(L, -2) != 0 && len < HUSKYLENS_SEQ_MAX) {
            ids[seq_count][len++] = (uint8_t)luaL_checkinteger(L, -1);
            lua_pop(L, 1);
        }
        lens[seq_count] = len;
        seq_count++;
        lua_pop(L, 1);
    }

    if (!mapper_mutexes_init()) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: mutexes not initialized");
        return 2;
    }

    mapper_seq_lock();
    s_mapper_seqs.count = seq_count;
    memset(s_mapper_seqs.lens, 0, sizeof(s_mapper_seqs.lens));
    memset(s_mapper_seqs.ids,  0, sizeof(s_mapper_seqs.ids));
    for (uint8_t i = 0; i < seq_count; i++) {
        s_mapper_seqs.lens[i] = lens[i];
        if (lens[i] > 0) memcpy(s_mapper_seqs.ids[i], ids[i], lens[i]);
    }
    mapper_seq_unlock();
    s_mapper_seq_loaded = true;

    if (mapper_nvs_save_seq() != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: error saving sequences to NVS");
        return 2;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_mapper_remap(lua_State *L) {
    uint8_t physical = (uint8_t)luaL_checkinteger(L, 1);
    uint8_t logical  = physical;

    if (!s_mapper_map_loaded) mapper_nvs_load();

    mapper_map_lock();
    for (uint8_t i = 0; i < s_mapper_map.count; i++) {
        if (s_mapper_map.entries[i].physical_id == physical) {
            logical = s_mapper_map.entries[i].logical_id;
            break;
        }
    }
    mapper_map_unlock();

    lua_pushinteger(L, logical);
    return 1;
}

static int robotito_ble_mapper_clear_map(lua_State *L) {
    if (!mapper_mutexes_init()) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: mutexes not initialized");
        return 2;
    }
    mapper_map_clear();
    s_mapper_map_loaded = true;
    if (mapper_nvs_save() != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: error clearing map in NVS");
        return 2;
    }
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_mapper_clear_sequence(lua_State *L) {
    if (!mapper_mutexes_init()) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: mutexes not initialized");
        return 2;
    }
    mapper_seq_clear();
    s_mapper_seq_loaded = true;
    if (mapper_nvs_save_seq() != ESP_OK) {
        lua_pushnil(L);
        lua_pushstring(L, "mapper: error clearing sequence in NVS");
        return 2;
    }
    lua_pushboolean(L, true);
    return 1;
}

/* ============================================================================
 * Lua API: BLE init, send, callback registration
 * ============================================================================ */

static int robotito_ble_init(lua_State *L) {
    if (robotito_ble_initialized) {
        lua_pushboolean(L, true);
        return 1;
    }

    printf("initializing robotito_ble\n");

    const char *name = luaL_checkstring(L, 1);
    if (name) {
        ble_device_name = strdup(name);
        printf("  ble device name: %s\n", ble_device_name);
    } else {
        ble_device_name = SAMPLE_DEVICE_NAME;
        printf("  ble device name (default): %s\n", ble_device_name);
    }

    size_t ble_device_name_length = strlen(ble_device_name);
    spp_adv_data_size = 9 + ble_device_name_length;
    if (spp_adv_data_size > 31) {
        syslog(LOG_ERR, "ble device name too long\n");
        lua_pushnil(L);
        lua_pushstring(L, "ble device name too long");
        return 2;
    }
    spp_adv_data = malloc(sizeof(uint8_t) * spp_adv_data_size);
    spp_adv_data[0] = 0x02; spp_adv_data[1] = 0x01; spp_adv_data[2] = 0x06;
    spp_adv_data[3] = 0x03; spp_adv_data[4] = 0x03; spp_adv_data[5] = 0xF0; spp_adv_data[6] = 0xAB;
    spp_adv_data[7] = ble_device_name_length + 1; spp_adv_data[8] = 0x09;
    memcpy(spp_adv_data + 9, ble_device_name, ble_device_name_length);

    printf("adv (len=%u):", spp_adv_data_size);
    for (size_t i = 0; i < spp_adv_data_size; i++) {
        printf(" %02X", spp_adv_data[i]);
    }
    printf("\n");

    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        syslog(LOG_ERR, "%s init controller failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "init controller failed");
        return 2;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        syslog(LOG_ERR, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "enable controller failed");
        return 2;
    }

    syslog(LOG_INFO, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        syslog(LOG_ERR, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "init bluetooth failed");
        return 2;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        syslog(LOG_ERR, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        lua_pushnil(L);
        lua_pushstring(L, "enable bluetooth failed");
        return 2;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    stream_buffer_handle = xRingbufferCreate(STREAM_BUFFER_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (stream_buffer_handle == NULL) {
        syslog(LOG_ERR, "%s failed to create ring buffer\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "failed to create ring buffer");
        return 2;
    }
    xTaskCreate(spp_rcv_task, "spp_rcv_task", CONFIG_ROBOTITO_BLE_STACK_SIZE, NULL, 10, NULL);

    mapper_evt_buffer_handle = xRingbufferCreate(MAPPER_EVT_BUFFER_SIZE_BYTES, RINGBUF_TYPE_BYTEBUF);
    if (mapper_evt_buffer_handle == NULL) {
        syslog(LOG_ERR, "%s failed to create mapper ring buffer\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "failed to create mapper ring buffer");
        return 2;
    }
    xTaskCreate(mapper_evt_task, "mapper_evt_task", CONFIG_ROBOTITO_BLE_STACK_SIZE, NULL, 10, NULL);

    spp_task_init();

    robotito_ble_initialized = true;

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_send(lua_State *L) {
    if (!enable_data_ntf) {
        syslog(LOG_ERR, "%s not enabled data Notify\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "not enabled data Notify");
        return 2;
    }

    size_t length;
    const uint8_t *string = (uint8_t *)luaL_checklstring(L, 1, &length);

    uint8_t *temp = (uint8_t *)malloc(sizeof(uint8_t) * length);
    if (temp == NULL) {
        syslog(LOG_ERR, "%s malloc.1 failed\n", __func__);
        lua_pushnil(L);
        lua_pushstring(L, "malloc.1 failed");
        return 2;
    }
    memcpy(temp, string, length);

    if (length <= (spp_mtu_size - 3)) {
        esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
            spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], length, temp, false);
    } else {
        uint8_t total_num;
        if ((length % (spp_mtu_size - 3)) == 0) {
            total_num = length / (spp_mtu_size - 3);
        } else {
            total_num = length / (spp_mtu_size - 3) + 1;
        }

        uint8_t *ntf_value_p = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
        if (ntf_value_p == NULL) {
            syslog(LOG_ERR, "%s malloc.2 failed\n", __func__);
            free(temp);
            lua_pushnil(L);
            lua_pushstring(L, "malloc.2 failed");
            return 2;
        }

        for (uint8_t current_num = 1; current_num <= total_num; current_num++) {
            if (current_num < total_num) {
                memcpy(ntf_value_p, temp + (current_num - 1) * (spp_mtu_size - 3), (spp_mtu_size - 3));
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                    spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                    (spp_mtu_size - 3), ntf_value_p, false);
            } else {
                size_t remaining = length - (current_num - 1) * (spp_mtu_size - 3);
                memcpy(ntf_value_p, temp + (current_num - 1) * (spp_mtu_size - 3), remaining);
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                    spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL],
                    remaining, ntf_value_p, false);
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        free(ntf_value_p);
    }
    free(temp);

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_rcv(lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        robotito_ble_rcv_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (robotito_ble_rcv_callback == LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no rcv callback set");
            return 2;
        }
        robotito_ble_rcv_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_line(lua_State *L) {
    bool enable = lua_toboolean(L, 1);
    if (enable) {
        if (line_buff == NULL) {
            line_buff = (char *)malloc(sizeof(char) * CONFIG_ROBOTITO_BLE_LINEBUFFER);
            if (line_buff == NULL) {
                syslog(LOG_ERR, "%s malloc failed\n", __func__);
                lua_pushnil(L);
                lua_pushstring(L, "malloc failed");
                return 2;
            }
        }
        luaL_checktype(L, 1, LUA_TFUNCTION);
        lua_pushvalue(L, 1);
        robotito_ble_line_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    } else {
        if (robotito_ble_line_callback == LUA_REFNIL) {
            lua_pushnil(L);
            lua_pushstring(L, "no line callback set");
            return 2;
        }
        robotito_ble_line_callback = LUA_REFNIL;
    }

    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_set_connect_callback(lua_State *L) {
    luaL_checktype(L, 1, LUA_TFUNCTION);
    lua_pushvalue(L, 1);
    robotito_ble_connect_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_set_disconnect_callback(lua_State *L) {
    luaL_checktype(L, 1, LUA_TFUNCTION);
    lua_pushvalue(L, 1);
    robotito_ble_disconnect_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_set_mapper_cfg_callback(lua_State *L) {
    luaL_checktype(L, 1, LUA_TFUNCTION);
    lua_pushvalue(L, 1);
    robotito_ble_mapper_cfg_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_set_mapper_seq_callback(lua_State *L) {
    luaL_checktype(L, 1, LUA_TFUNCTION);
    lua_pushvalue(L, 1);
    robotito_ble_mapper_seq_callback = luaL_ref(L, LUA_REGISTRYINDEX);
    lua_pushboolean(L, true);
    return 1;
}

static int robotito_ble_mem_info(lua_State *L) {    
    size_t ram_free = esp_get_free_heap_size();    
    size_t ram_min_free = esp_get_minimum_free_heap_size();
    lua_newtable(L);
    lua_pushinteger(L, (lua_Integer)ram_free);    lua_setfield(L, -2, "free");
    lua_pushinteger(L, (lua_Integer)ram_min_free);    lua_setfield(L, -2, "min_free");
    return 1;
}

/* ============================================================================
 * Module registration
 * ============================================================================ */

static const luaL_Reg robotito_ble[] = {
    {"init",                    robotito_ble_init},
    {"send",                    robotito_ble_send},
    {"set_rcv_callback",        robotito_ble_rcv},
    {"set_line_callback",       robotito_ble_line},
    {"set_connect_callback",    robotito_ble_set_connect_callback},
    {"set_disconnect_callback", robotito_ble_set_disconnect_callback},
    {"set_mapper_cfg_callback", robotito_ble_set_mapper_cfg_callback},
    {"set_mapper_seq_callback", robotito_ble_set_mapper_seq_callback},
    {"mapper_init",             robotito_ble_mapper_init},
    {"mapper_get_pairs",        robotito_ble_mapper_get_pairs},
    {"mapper_set_pairs",        robotito_ble_mapper_set_pairs},
    {"mapper_get_sequence",     robotito_ble_mapper_get_sequence},
    {"mapper_set_sequence",     robotito_ble_mapper_set_sequence},
    {"mapper_get_sequences",    robotito_ble_mapper_get_sequences},
    {"mapper_set_sequences",    robotito_ble_mapper_set_sequences},
    {"mapper_remap",            robotito_ble_mapper_remap},
    {"mapper_clear_map",        robotito_ble_mapper_clear_map},
    {"mapper_clear_sequence",   robotito_ble_mapper_clear_sequence},
    {"mem_info", robotito_ble_mem_info},
    {NULL, NULL}
};

LUALIB_API int luaopen_robotito_ble(lua_State *L) {
    luaL_newlib(L, robotito_ble);
    return 1;
}

MODULE_REGISTER_RAM(ROBOTITO_BLE, robotito_ble, luaopen_robotito_ble, 1);

#endif