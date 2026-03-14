#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

#include "MP4201.h"
#include "MP4201Driver.h"

#include "MCP4725.h"

static const char *BLE_TAG = "BLE_SUBS";

/* --- 外部变量引用 (必须与 main.c 中的定义严格一致) --- */
typedef struct {
    float vin, iin, pin;
    float vout, iout, pout;
    float efficiency;
    float target_voltage;
    float current_limit;
    bool is_output_active;
    int32_t work_mode_index;
    int32_t frequency_index;
} app_config_t;

extern app_config_t g_sys_cfg;
extern bool g_config_is_dirty;
extern uint32_t g_last_mod_timestamp;

/* --- UUID 定义 --- */
#define BLE_POWER_SVC_UUID          0xFF01
#define BLE_TELEMETRY_CHR_UUID      0xFF02 // 实时遥测数据 (Notify)
#define BLE_CONTROL_CHR_UUID        0xFF03 // 控制指令 (Write)

static uint16_t g_ble_conn_handle;
static uint16_t g_ble_telem_attr_handle;
bool g_is_connected = false;
static uint8_t g_ble_addr_type;

/* --- 函数声明 --- */
static void ble_app_advertise(void);
static int ble_gap_event(struct ble_gap_event *event, void *arg);

/**
 * @brief 处理网页端写入的控制指令
 * 协议格式：[指令码(1B), 数据高位(1B), 数据低位(1B)]
 */
static int ble_gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t *data = ctxt->om->om_data;
        uint16_t len = ctxt->om->om_len;

        if (len >= 3) {
            uint8_t cmd = data[0];
            uint16_t val = (data[1] << 8) | data[2];

            // 标记配置已改变，触发 main.c 的 5秒后 NVS 自动保存
            g_config_is_dirty = true;
            g_last_mod_timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

            switch (cmd) {
                case 0x01: // 设置电压 (10倍)
                    g_sys_cfg.target_voltage = val / 10.0f;
                    mp4201_vout_set(&MP4201, g_sys_cfg.target_voltage);
                    break;
                case 0x02: // 【新增】设置限流 (网页传 25 代表 2.5A)
                    g_sys_cfg.current_limit = val / 10.0f;
                    // 调用硬件接口设置 MP4201 的限流值
                    mp4201_iout_oc_fault_limit_set(&MP4201, g_sys_cfg.current_limit);
                    break;
                case 0x03: // 开关输出
                    g_sys_cfg.is_output_active = (val > 0);
                    mp4201_operation_set(&MP4201, g_sys_cfg.is_output_active);
                    break;
                case 0x04: // 修改工作模式 (val 为 0-3 索引)
                    // 限制范围防止数组越界，更新全局变量
                    g_sys_cfg.work_mode_index = (val > 3) ? 3 : val;
                    // 通过 DAC 设置 MP4201 硬件引脚
                    MP4201_SET_MODE_DAC(g_sys_cfg.work_mode_index);
                    break;
                case 0x05: // 修改频率 (val 为 0-3 索引)
                    // 限制范围，更新全局变量
                    g_sys_cfg.frequency_index = (val > 3) ? 3 : val;
                    // 通过 DAC 设置 MP4201 硬件引脚
                    MP4201_SET_FREQ_DAC(g_sys_cfg.frequency_index);
                    break;
            }
            ESP_LOGI(BLE_TAG, "Command: 0x%02X, Value: %d", cmd, val);
        }
    }
    return 0;
}

/* --- GATT 服务定义表 --- */
static const struct ble_gatt_svc_def g_ble_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_POWER_SVC_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BLE_TELEMETRY_CHR_UUID),
                .access_cb = ble_gatt_svr_access_cb,
                .val_handle = &g_ble_telem_attr_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_CONTROL_CHR_UUID),
                .access_cb = ble_gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {0}
        },
    },
    {0}
};

/* --- 开启蓝牙广播 --- */
static void ble_app_advertise(void) {
    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));

    // 1. 设置标志位 (发现模式)
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    // 2. 设置设备名称
    const char *name = "MP4201_PWR";
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    // 3. 设置包含的服务UUID
    adv_fields.uuids16 = (ble_uuid16_t[]){ BLE_UUID16_INIT(BLE_POWER_SVC_UUID) };
    adv_fields.num_uuids16 = 1;
    adv_fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Error setting adv fields; rc=%d", rc);
        return;
    }

    // 4. 开始广播
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // 可连接
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // 可发现

    rc = ble_gap_adv_start(g_ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Error starting advertisement; rc=%d", rc);
    } else {
        ESP_LOGI(BLE_TAG, "Advertising Started! Name: %s", name);
    }
}

/* --- 蓝牙事件回调 --- */
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            g_is_connected = true;
            g_ble_conn_handle = event->connect.conn_handle;
            ESP_LOGI(BLE_TAG, "PC/Web Client Connected");
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            g_is_connected = false;
            ESP_LOGE(BLE_TAG, "Disconnected! Reason: 0x%x", event->disconnect.reason);
            ble_app_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_app_advertise();
            break;
    }
    return 0;
}

/* --- 协议栈同步回调 (只有确定了地址才能广播) --- */
static void ble_on_stack_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &g_ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(BLE_TAG, "Error determining address type; rc=%d", rc);
        return;
    }
    ble_app_advertise();
}

/* --- 外部调用的任务：每 250ms 向网页推送一次遥测数据 --- */
void task_ble_telemetry_tx(void *pv) {
    while (1) {
        if (g_is_connected) {
            uint8_t pkg[16];
            // 保持 100 倍缩放以保留两位小数
            uint16_t vo = (uint16_t)(g_sys_cfg.vout * 100);
            uint16_t io = (uint16_t)(g_sys_cfg.iout * 100);
            uint16_t vi = (uint16_t)(g_sys_cfg.vin * 100);
            uint16_t ii = (uint16_t)(g_sys_cfg.iin * 100);

            pkg[0] = vo >> 8; pkg[1] = vo & 0xFF;
            pkg[2] = io >> 8; pkg[3] = io & 0xFF;
            pkg[4] = vi >> 8; pkg[5] = vi & 0xFF;
            pkg[6] = ii >> 8; pkg[7] = ii & 0xFF;
            pkg[8] = (uint8_t)g_sys_cfg.efficiency;
            // 状态位：低 4 位存储开关，高 4 位可以存储当前的模式索引(方便网页同步显示)
            pkg[9] = (g_sys_cfg.is_output_active & 0x0F) | ((g_sys_cfg.work_mode_index & 0x0F) << 4);

            pkg[10] = (uint8_t)g_sys_cfg.work_mode_index; // 将旋钮改动的模式发给网页
            pkg[11] = (uint8_t)g_sys_cfg.frequency_index; // 将旋钮改动的频率发给网页

            // --- 新增：同步设定值 (12-15字节) ---
            uint16_t sv = (uint16_t)(g_sys_cfg.target_voltage * 100);
            uint16_t si = (uint16_t)(g_sys_cfg.current_limit * 100);
            pkg[12] = sv >> 8; pkg[13] = sv & 0xFF;
            pkg[14] = si >> 8; pkg[15] = si & 0xFF;

            struct os_mbuf *om = ble_hs_mbuf_from_flat(pkg, sizeof(pkg));
            if (om) ble_gattc_notify_custom(g_ble_conn_handle, g_ble_telem_attr_handle, om);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 5Hz 刷新
    }
}

/* --- 协议栈运行任务 --- */
void nimble_host_task(void *param) {
    ESP_LOGI(BLE_TAG, "BLE Host Task Started");
    nimble_port_run(); // 阻塞运行
    nimble_port_deinit();
}

/* --- 初始化入口 --- */
void init_ble_stack(void) {
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_TAG, "Failed to init nimble %d", ret);
        return;
    }

    // 初始化 GAP/GATT 服务
    ble_svc_gap_device_name_set("MP4201_PWR");
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // 注册自定义服务表
    ble_gatts_count_cfg(g_ble_gatt_svcs);
    ble_gatts_add_svcs(g_ble_gatt_svcs);

    // 设置同步回调（触发广播的关键）
    ble_hs_cfg.sync_cb = ble_on_stack_sync;

    // 创建 NimBLE 主任务
    xTaskCreatePinnedToCore(nimble_host_task, "NimBLE", 8192, NULL, 5, NULL, 0);
}