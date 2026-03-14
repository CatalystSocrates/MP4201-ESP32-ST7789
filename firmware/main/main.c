#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lvgl.h"
#include "st7789_driver.h"
#include "MP4201.h"
#include "MP4201Driver.h"
#include "IIC_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include <encoder.h>
#include "MCP4725.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>

static const char *TAG = "MP4201_APP";

/* --- 硬件控制接口声明 (通过外部 DAC 修改 MP4201 引脚电压) --- */
void MP4201_SET_MODE_DAC(int mode_index);
void MP4201_SET_FREQ_DAC(int freq_index);

void lv_port_disp_init(void);

extern void init_ble_stack(void);
extern void task_ble_telemetry_tx(void *pv);
extern bool g_is_connected;

/* --- 系统控制模式枚举：定义当前旋钮控制的是哪个参数 --- */
typedef enum {
    CTRL_MODE_VOLTAGE = 0,    // 调节电压
    CTRL_MODE_CURRENT,        // 调节限流
    CTRL_MODE_WORK_MODE,      // 切换工作模式 (PFM/FCCM等)
    CTRL_MODE_FREQUENCY,      // 切换开关频率
    CTRL_MODE_MAX
} app_ctrl_mode_t;

/* --- UI 静态文本词条 --- */
static const char* STR_WORK_MODES[] = {"PFM(FSS)","PFM(noFSS)","FCCM(FSS)","FCCM(noFSS)"};
static const char* STR_FREQUENCIES[] = {"200kHz", "400kHz", "600kHz", "1MHz"};

/* --- 全局配置结构体：存储所有关键运行参数 --- */
typedef struct {
    float vin, iin, pin;        // 输入电压、电流、功率
    float vout, iout, pout;     // 输出电压、电流、功率
    float efficiency;           // 转换效率
    float target_voltage;       // 用户设定的目标电压
    float current_limit;        // 用户设定的限流值
    bool is_output_active;      // 输出开关状态 (ON/OFF)
    int32_t work_mode_index;    // 当前模式索引
    int32_t frequency_index;    // 当前频率索引
} app_config_t;

app_config_t g_sys_cfg = {
    .target_voltage = 12.0f, 
    .current_limit = 2.0f, 
    .is_output_active = false,
    .work_mode_index = 0, 
    .frequency_index = 0
};

/* --- 运行控制全局变量 --- */
static app_ctrl_mode_t g_curr_ctrl_mode = CTRL_MODE_VOLTAGE; // 当前焦点位置
static int g_step_index = 0;   // 数字调节步进索引 (0:百位/十位, 1:个位, 2:十分位)
static bool g_is_editing = false; // 是否处于“编辑模式”（按下旋钮进入）

/* --- NVS 存储控制变量 (采用防抖策略，减少 Flash 擦写) --- */
static const char* NVS_STORAGE_NAME = "sys_cfg";
bool g_config_is_dirty = false;      // 脏数据标记：标记配置已被修改，需要保存
uint32_t g_last_mod_timestamp = 0;    // 最后一次修改的时间戳

/* --- LVGL UI 对象句柄 --- */
static lv_obj_t *ui_lbl_vout, *ui_lbl_iout, *ui_lbl_pout; // 顶部大字实时数据
static lv_obj_t *ui_lbl_vin, *ui_lbl_iin, *ui_lbl_pin;   // 右侧小字输入数据
static lv_obj_t *ui_lbl_set_v, *ui_lbl_set_i, *ui_lbl_status, *ui_lbl_eff; // 设定值及状态
static lv_obj_t *ui_lbl_mode_state, *ui_lbl_freq_state; // 底部常驻模式/频率显示
static lv_obj_t *ui_popup_options; // 修改模式/频率时弹出的半透明选择框
static lv_obj_t *ui_cursor_bar;    // 调节数字时的橙色下划线光标
static lv_obj_t *ui_chart_monitor; // 实时波形图表
static lv_chart_series_t *ui_ser_voltage; // 图表：电压曲线
static lv_chart_series_t *ui_ser_current; // 图表：电流曲线

static lv_obj_t *ui_lbl_ble_state; // 专门的蓝牙状态标签
extern bool g_is_connected;        // 引用 ble_subs.c 中的连接状态

/* ================== 核心逻辑实现 ================== */

/**
 * @brief 更新 UI 位选光标的位置
 */
void update_ui_cursor_pos() {
    // 如果焦点在模式或频率选择上，隐藏数字光标
    if (g_curr_ctrl_mode >= CTRL_MODE_WORK_MODE) {
        lv_obj_add_flag(ui_cursor_bar, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(ui_cursor_bar, LV_OBJ_FLAG_HIDDEN);
    
    // 对应显示文本中数字的 X 轴偏移像素 (需根据字体微调)
    const int x_offsets[] = {38, 48, 61}; 
    lv_obj_t *parent_lbl = (g_curr_ctrl_mode == CTRL_MODE_VOLTAGE) ? ui_lbl_set_v : ui_lbl_set_i;
    lv_obj_align_to(ui_cursor_bar, parent_lbl, LV_ALIGN_BOTTOM_LEFT, x_offsets[g_step_index], 0);
}

/**
 * @brief 从 MP4201 芯片同步实时遥测数据
 */
void sync_telemetry_data(void) {
    get_mp4201_all_read_data_info(&MP4201, 5); // 读取5次取平均
    g_sys_cfg.vin  = MP4201.read_info.Vin_read;
    g_sys_cfg.iin  = MP4201.read_info.Iin_read;
    g_sys_cfg.vout = MP4201.read_info.Vout_read;
    g_sys_cfg.iout = MP4201.read_info.Iout_read;
    g_sys_cfg.pin  = g_sys_cfg.vin * g_sys_cfg.iin;
    g_sys_cfg.pout = g_sys_cfg.vout * g_sys_cfg.iout;
    g_sys_cfg.efficiency = (g_sys_cfg.pin > 0.1f) ? (g_sys_cfg.pout / g_sys_cfg.pin * 100.0f) : 0;
}

/* ================== NVS 断电保存存储逻辑 ================== */

void load_settings_from_nvs(void) {
    nvs_handle_t nvs_h;
    if (nvs_open(NVS_STORAGE_NAME, NVS_READONLY, &nvs_h) != ESP_OK) {
        ESP_LOGW(TAG, "首次启动: 使用默认参数");
        return;
    }
    uint32_t v_bits, i_bits;
    // NVS 不直存 float，通过字节拷贝还原
    if (nvs_get_u32(nvs_h, "target_v", &v_bits) == ESP_OK) memcpy(&g_sys_cfg.target_voltage, &v_bits, 4);
    if (nvs_get_u32(nvs_h, "limit_i", &i_bits) == ESP_OK) memcpy(&g_sys_cfg.current_limit, &i_bits, 4);
    nvs_get_i32(nvs_h, "mode_idx", &g_sys_cfg.work_mode_index);
    nvs_get_i32(nvs_h, "freq_idx", &g_sys_cfg.frequency_index);
    nvs_close(nvs_h);
}

void save_settings_to_nvs(void) {
    nvs_handle_t nvs_h;
    if (nvs_open(NVS_STORAGE_NAME, NVS_READWRITE, &nvs_h) != ESP_OK) return;
    uint32_t v_bits, i_bits;
    memcpy(&v_bits, &g_sys_cfg.target_voltage, 4);
    memcpy(&i_bits, &g_sys_cfg.current_limit, 4);
    nvs_set_u32(nvs_h, "target_v", v_bits);
    nvs_set_u32(nvs_h, "limit_i", i_bits);
    nvs_set_i32(nvs_h, "mode_idx", g_sys_cfg.work_mode_index);
    nvs_set_i32(nvs_h, "freq_idx", g_sys_cfg.frequency_index);
    nvs_commit(nvs_h); // 提交修改
    nvs_close(nvs_h);
    ESP_LOGI(TAG, "配置已自动保存到 Flash");
}

/* ================== UI 界面构建 ================== */

void setup_power_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    // 初始化三种字体样式：小、中、大
    static lv_style_t st_sm, st_lg, st_md;
    lv_style_init(&st_sm); lv_style_set_text_font(&st_sm, &lv_font_montserrat_14); lv_style_set_text_color(&st_sm, lv_color_hex(0xFFFFFF));
    lv_style_init(&st_lg); lv_style_set_text_font(&st_lg, &lv_font_montserrat_26); lv_style_set_text_color(&st_lg, lv_color_hex(0xFFFFFF));
    lv_style_init(&st_md); lv_style_set_text_font(&st_md, &lv_font_montserrat_16); lv_style_set_text_color(&st_md, lv_color_hex(0xFFFFFF));

    // 顶部标题
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "MP4201 POWER STATION");
    lv_obj_add_style(title, &st_md, 0);
    lv_obj_set_style_text_color(title, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

    // 实时遥测大字标签 (Vo, Io, Po)
    ui_lbl_vout = lv_label_create(scr); lv_obj_add_style(ui_lbl_vout, &st_lg, 0); lv_obj_align(ui_lbl_vout, LV_ALIGN_TOP_LEFT, 5, 30);
    ui_lbl_iout = lv_label_create(scr); lv_obj_add_style(ui_lbl_iout, &st_lg, 0); lv_obj_align(ui_lbl_iout, LV_ALIGN_TOP_LEFT, 5, 55);
    ui_lbl_pout = lv_label_create(scr); lv_obj_add_style(ui_lbl_pout, &st_lg, 0); lv_obj_align(ui_lbl_pout, LV_ALIGN_TOP_LEFT, 5, 80);

    // 输入端小字标签 (Vi, Ii, Pi)
    ui_lbl_vin = lv_label_create(scr); lv_obj_add_style(ui_lbl_vin, &st_sm, 0); lv_obj_align(ui_lbl_vin, LV_ALIGN_TOP_RIGHT, -5, 35);
    ui_lbl_iin = lv_label_create(scr); lv_obj_add_style(ui_lbl_iin, &st_sm, 0); lv_obj_align(ui_lbl_iin, LV_ALIGN_TOP_RIGHT, -5, 55);
    ui_lbl_pin = lv_label_create(scr); lv_obj_add_style(ui_lbl_pin, &st_sm, 0); lv_obj_align(ui_lbl_pin, LV_ALIGN_TOP_RIGHT, -5, 75);
    ui_lbl_eff = lv_label_create(scr); lv_obj_add_style(ui_lbl_eff, &st_sm, 0); lv_obj_align(ui_lbl_eff, LV_ALIGN_TOP_RIGHT, -5, 95);

    // 设定值标签 (SetV, SetI)
    ui_lbl_set_v = lv_label_create(scr); lv_obj_add_style(ui_lbl_set_v, &st_md, 0); lv_obj_align(ui_lbl_set_v, LV_ALIGN_BOTTOM_LEFT, 10, -48);
    ui_lbl_set_i = lv_label_create(scr); lv_obj_add_style(ui_lbl_set_i, &st_md, 0); lv_obj_align(ui_lbl_set_i, LV_ALIGN_BOTTOM_LEFT, 10, -30);

    // 状态栏
    ui_lbl_mode_state = lv_label_create(scr); lv_obj_add_style(ui_lbl_mode_state, &st_sm, 0); lv_obj_align(ui_lbl_mode_state, LV_ALIGN_BOTTOM_RIGHT, -10, -30);
    ui_lbl_freq_state = lv_label_create(scr); lv_obj_add_style(ui_lbl_freq_state, &st_sm, 0); lv_obj_align(ui_lbl_freq_state, LV_ALIGN_BOTTOM_RIGHT, -10, -48);

    

    // 弹出式选择框样式
    ui_popup_options = lv_label_create(scr);
    lv_obj_add_style(ui_popup_options, &st_md, 0);
    lv_obj_set_style_bg_opa(ui_popup_options, LV_OPA_80, 0);
    lv_obj_set_style_bg_color(ui_popup_options, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_color(ui_popup_options, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_border_width(ui_popup_options, 2, 0);
    lv_obj_set_style_pad_all(ui_popup_options, 10, 0);
    lv_obj_align(ui_popup_options, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(ui_popup_options, LV_OBJ_FLAG_HIDDEN); // 默认隐藏

    ui_lbl_status = lv_label_create(scr); lv_obj_add_style(ui_lbl_status, &st_lg, 0); lv_obj_align(ui_lbl_status, LV_ALIGN_BOTTOM_MID, 0, -4);
    
     // 1. 创建蓝牙状态标签
    ui_lbl_ble_state = lv_label_create(scr);
    
    // 2. 【关键】强制指定使用内置的 Montserrat 字体（它包含了符号集）
    // 确保你的 menuconfig 中开启了对应尺寸的字体
    lv_obj_set_style_text_font(ui_lbl_ble_state, &lv_font_montserrat_14, 0); 
    
    // 3. 设置文本
    lv_label_set_text(ui_lbl_ble_state, LV_SYMBOL_BLUETOOTH); 
    
    // 4. 调整位置（放在 Vi 标签上方）
    lv_obj_align(ui_lbl_ble_state, LV_ALIGN_BOTTOM_RIGHT, 0, -4); 
    
    // 5. 设置初始颜色（灰色）
    lv_obj_set_style_text_color(ui_lbl_ble_state, lv_color_hex(0x444444), 0);

    ui_cursor_bar = lv_obj_create(scr);
    lv_obj_set_size(ui_cursor_bar, 8, 1);
    lv_obj_set_style_bg_color(ui_cursor_bar, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_set_style_border_width(ui_cursor_bar, 0, -10);

    
}

void setup_chart(void) {
    lv_obj_t *scr = lv_scr_act();
    ui_chart_monitor = lv_chart_create(scr);
    lv_obj_set_size(ui_chart_monitor, 230, 60);
    lv_obj_align(ui_chart_monitor, LV_ALIGN_CENTER, 0, 20);
    
    lv_chart_set_type(ui_chart_monitor, LV_CHART_TYPE_LINE);
    lv_obj_set_style_bg_color(ui_chart_monitor, lv_color_hex(0x000000), 0);
    lv_chart_set_div_line_count(ui_chart_monitor, 3, 5); // 设置网格线
    lv_chart_set_point_count(ui_chart_monitor, 50);     // 缓冲区点数
    lv_chart_set_range(ui_chart_monitor, LV_CHART_AXIS_PRIMARY_Y, 0, 100);

    // 添加两条曲线数据系列
    ui_ser_voltage = lv_chart_add_series(ui_chart_monitor, lv_palette_main(LV_PALETTE_YELLOW), LV_CHART_AXIS_PRIMARY_Y);
    ui_ser_current = lv_chart_add_series(ui_chart_monitor, lv_palette_main(LV_PALETTE_CYAN), LV_CHART_AXIS_PRIMARY_Y);
    lv_obj_set_style_size(ui_chart_monitor, 0, LV_PART_INDICATOR); // 隐藏圆点，只留线
}

/* ================== FreeRTOS 任务实现 ================== */

/**
 * @brief GUI 刷新任务：负责所有 UI 部件的实时更新
 */
void task_gui_refresh(void *pv) {
    while (1) {
        lv_tick_inc(50); 
        
        // 更新显示数值
        lv_label_set_text_fmt(ui_lbl_vout, "Vo:%02d.%03dV", (int)g_sys_cfg.vout, (int)(g_sys_cfg.vout * 1000) % 1000);
        lv_label_set_text_fmt(ui_lbl_iout, "Io:%02d.%03dA", (int)g_sys_cfg.iout, (int)(g_sys_cfg.iout * 1000) % 1000);
        lv_label_set_text_fmt(ui_lbl_pout, "Po:%03d.%01dW", (int)g_sys_cfg.pout, (int)(g_sys_cfg.pout * 10) % 10);

        lv_label_set_text_fmt(ui_lbl_vin,  "Vi:%02d.%02dV", (int)g_sys_cfg.vin, (int)(g_sys_cfg.vin * 10) % 10);
        lv_label_set_text_fmt(ui_lbl_iin,  "Ii:%02d.%02dA", (int)g_sys_cfg.iin, (int)(g_sys_cfg.iin * 1000) % 1000);
        lv_label_set_text_fmt(ui_lbl_pin,  "Pi:%03d.%01dW", (int)g_sys_cfg.pin, (int)(g_sys_cfg.pin * 10) % 10);

        lv_label_set_text_fmt(ui_lbl_set_v, "SetV:%02d.%01dV", (int)g_sys_cfg.target_voltage, (int)(g_sys_cfg.target_voltage * 10) % 10);
        lv_label_set_text_fmt(ui_lbl_set_i, "SetI:%02d.%01dA", (int)g_sys_cfg.current_limit, (int)(g_sys_cfg.current_limit * 10) % 10);

        lv_label_set_text_fmt(ui_lbl_mode_state, "%s", STR_WORK_MODES[g_sys_cfg.work_mode_index]);
        lv_label_set_text_fmt(ui_lbl_freq_state, "FREQ: %s", STR_FREQUENCIES[g_sys_cfg.frequency_index]);

        lv_label_set_text_fmt(ui_lbl_eff, "Eff:%d%%", (int)g_sys_cfg.efficiency);
        
        // 根据焦点位置高亮颜色
        lv_obj_set_style_text_color(ui_lbl_set_v, (g_curr_ctrl_mode == CTRL_MODE_VOLTAGE) ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_hex(0x888888), 0);
        lv_obj_set_style_text_color(ui_lbl_set_i, (g_curr_ctrl_mode == CTRL_MODE_CURRENT) ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_hex(0x888888), 0);
        lv_obj_set_style_text_color(ui_lbl_mode_state, (g_curr_ctrl_mode == CTRL_MODE_WORK_MODE) ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_hex(0x888888), 0);
        lv_obj_set_style_text_color(ui_lbl_freq_state, (g_curr_ctrl_mode == CTRL_MODE_FREQUENCY) ? lv_palette_main(LV_PALETTE_ORANGE) : lv_color_hex(0x888888), 0);

        // 图表曲线绘制 (归一化显示)
        lv_chart_set_next_value(ui_chart_monitor, ui_ser_voltage, (int)(g_sys_cfg.vout * 100 / (g_sys_cfg.target_voltage + 1)));
        lv_chart_set_next_value(ui_chart_monitor, ui_ser_current, (int)(g_sys_cfg.iout * 100 / (g_sys_cfg.current_limit + 0.1)));

        // 弹出选项逻辑：只有处于编辑模式且焦点在模式/频率上才显示
        if (g_is_editing && g_curr_ctrl_mode >= CTRL_MODE_WORK_MODE) {
            lv_obj_clear_flag(ui_popup_options, LV_OBJ_FLAG_HIDDEN);
            char buf[128] = "";
            const char** opts = (g_curr_ctrl_mode == CTRL_MODE_WORK_MODE) ? STR_WORK_MODES : STR_FREQUENCIES;
            int curr_idx = (g_curr_ctrl_mode == CTRL_MODE_WORK_MODE) ? g_sys_cfg.work_mode_index : g_sys_cfg.frequency_index;
            snprintf(buf, 128, "SELECT %s:\n", (g_curr_ctrl_mode == CTRL_MODE_WORK_MODE) ? "MODE" : "FREQ");
            for(int i=0; i<4; i++) {
                char tmp[32]; snprintf(tmp, 32, "%s %s\n", (curr_idx == i ? ">" : " "), opts[i]);
                strcat(buf, tmp);
            }
            lv_label_set_text(ui_popup_options, buf);
        } else {
            lv_obj_add_flag(ui_popup_options, LV_OBJ_FLAG_HIDDEN);
        }

        if (g_is_connected) {
            // 已连接：亮蓝色
            lv_obj_set_style_text_color(ui_lbl_ble_state, lv_palette_main(LV_PALETTE_BLUE), 0);
        } else {
            // 未连接：深灰色
            lv_obj_set_style_text_color(ui_lbl_ble_state, lv_color_hex(0x333333), 0);
        }

        lv_label_set_text_fmt(ui_lbl_status, "%s", g_sys_cfg.is_output_active ? "ACTIVE" : "OFF");
        lv_obj_set_style_text_color(ui_lbl_status, g_sys_cfg.is_output_active ? lv_palette_main(LV_PALETTE_GREEN) : lv_palette_main(LV_PALETTE_RED), 0);

    
        update_ui_cursor_pos();
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

/**
 * @brief 编码器处理任务：负责用户输入及逻辑切换
 */
void task_encoder_handler(void *arg) {
    QueueHandle_t eq = xQueueCreate(10, sizeof(rotary_encoder_event_t));
    rotary_encoder_init(eq);
    rotary_encoder_t re_h = {.pin_a = 14, .pin_b = 13, .pin_btn = 9};
    rotary_encoder_add(&re_h);
    rotary_encoder_event_t e;

    while (1) {
        if (xQueueReceive(eq, &e, portMAX_DELAY) == pdTRUE) {
            // 长按：控制输出开关
            if (e.type == RE_ET_BTN_LONG_PRESSED) {
                g_sys_cfg.is_output_active = !g_sys_cfg.is_output_active;
                mp4201_operation_set(&MP4201, g_sys_cfg.is_output_active);
                continue; 
            }
            // 短按：进入/退出编辑状态
            if (e.type == RE_ET_BTN_CLICKED) { g_is_editing = !g_is_editing; continue; }

            if (e.type == RE_ET_CHANGED) {
                if (!g_is_editing) {
                    // 非编辑模式：在 8 个焦点位之间导航 (V3个位, I3个位, 频率, 模式)
                    static int focus_idx = 0; focus_idx = (focus_idx + e.diff + 8) % 8;
                    if (focus_idx <= 2) { g_curr_ctrl_mode = CTRL_MODE_VOLTAGE; g_step_index = focus_idx; }
                    else if (focus_idx <= 5) { g_curr_ctrl_mode = CTRL_MODE_CURRENT; g_step_index = focus_idx - 3; }
                    else if (focus_idx == 6) { g_curr_ctrl_mode = CTRL_MODE_FREQUENCY; }
                    else { g_curr_ctrl_mode = CTRL_MODE_WORK_MODE; }
                } else {
                    // 编辑模式：修改具体参数，并标记脏数据用于 NVS 保存
                    g_config_is_dirty = true; g_last_mod_timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
                    if (g_curr_ctrl_mode == CTRL_MODE_VOLTAGE) {
                        float step = (g_step_index == 0 ? 10.0f : (g_step_index == 1 ? 1.0f : 0.1f));
                        g_sys_cfg.target_voltage = fmax(0.8f, fmin(60.0f, g_sys_cfg.target_voltage + e.diff * step));
                        mp4201_vout_set(&MP4201, g_sys_cfg.target_voltage);
                    } else if (g_curr_ctrl_mode == CTRL_MODE_CURRENT) {
                        float step = (g_step_index == 0 ? 10.0f : (g_step_index == 1 ? 1.0f : 0.1f));
                        g_sys_cfg.current_limit = fmax(0.1f, g_sys_cfg.current_limit + e.diff * step);
                        mp4201_iout_oc_fault_limit_set(&MP4201, g_sys_cfg.current_limit);
                    } else if (g_curr_ctrl_mode == CTRL_MODE_WORK_MODE) {
                        g_sys_cfg.work_mode_index = (g_sys_cfg.work_mode_index + e.diff + 4) % 4;
                        MP4201_SET_MODE_DAC(g_sys_cfg.work_mode_index);
                        g_config_is_dirty = true; // 标记保存
                    } else if (g_curr_ctrl_mode == CTRL_MODE_FREQUENCY) {
                        g_sys_cfg.frequency_index = (g_sys_cfg.frequency_index + e.diff + 4) % 4;
                        MP4201_SET_FREQ_DAC(g_sys_cfg.frequency_index);
                        g_config_is_dirty = true; // 标记保存
                    }
                }
            }
        }
    }
}

/**
 * @brief NVS 监控任务：当配置超过 5 秒未修改时，执行 Flash 写入
 */
void task_nvs_monitor(void *pv) {
    while (1) {
        if (g_config_is_dirty && (pdTICKS_TO_MS(xTaskGetTickCount()) - g_last_mod_timestamp > 5000)) {
            save_settings_to_nvs(); g_config_is_dirty = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_read(void *pvParameters) {
    while (1) {
        sync_telemetry_data();
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

void app_main(void) {
    // 初始化硬件
    i2c_master_init(); 
    MP4201_Init(&MP4201);

    sync_telemetry_data();
    if(g_sys_cfg.vin == 0) MP4201_Init(&MP4201);

    // 初始化存储
    if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }

    init_ble_stack();

    load_settings_from_nvs();

    // 关键步骤：恢复断电前的参数到硬件寄存器
    mp4201_vout_set(&MP4201, g_sys_cfg.target_voltage);
    mp4201_iout_oc_fault_limit_set(&MP4201, g_sys_cfg.current_limit);
    MP4201_SET_MODE_DAC(g_sys_cfg.work_mode_index);
    MP4201_SET_FREQ_DAC(g_sys_cfg.frequency_index);

    // 初始化图形引擎
    lv_init(); lv_port_disp_init(); setup_chart(); setup_power_ui();

    // 启动 FreeRTOS 任务 (分配不同优先级和核心)
    xTaskCreatePinnedToCore(task_encoder_handler, "encoder", 4096, NULL, 15, NULL, 1);
    xTaskCreatePinnedToCore(task_gui_refresh,     "gui",     8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(task_read,            "read",    4096, NULL, 5,  NULL, 1);
    xTaskCreate(task_nvs_monitor, "nvs_svc", 2048, NULL, 2, NULL);
    xTaskCreate(task_ble_telemetry_tx, "ble_tx", 4096, NULL, 12, NULL);
}