/*
 * A320 trackpad HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven version (minimal modification)
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT avago_a320

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include "trackpad_led.h"
#include "a320.h"

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

/* ========= ⭐ A320 专用 Work Queue ========= */
#define A320_WORKQ_STACK_SIZE 2048
#define A320_WORKQ_PRIORITY 5

/* ========= ⭐ NEW: I2C Mutex ========= */
static struct k_mutex a320_i2c_mutex;

K_THREAD_STACK_DEFINE(a320_workq_stack, A320_WORKQ_STACK_SIZE);
static struct k_work_q a320_workq;

/* ========================================================================= */
/* 鼠标与滚轮可调参数 (已映射至 Kconfig，用户可在 .conf 中配置)                 */
/* ========================================================================= */

// --- 滚轮方向配置 ---
#define SCROLL_X_DIR (-CONFIG_A320_SCROLL_X_DIR)
#define SCROLL_Y_DIR CONFIG_A320_SCROLL_Y_DIR

// --- 滚轮灵敏度与粒度配置 ---
#define SCROLL_DEADZONE CONFIG_A320_SCROLL_DEADZONE
#define SCROLL_INPUT_MAX CONFIG_A320_SCROLL_INPUT_MAX
#define SCROLL_DIVISOR_SLOW CONFIG_A320_SCROLL_DIVISOR_SLOW
#define SCROLL_DIVISOR_FAST CONFIG_A320_SCROLL_DIVISOR_FAST

// --- Arrow key threshold / divisor ---
#define ARROW_DEADZONE CONFIG_A320_SCROLL_DEADZONE
#define ARROW_INPUT_MAX 128
#define ARROW_DIVISOR_SLOW CONFIG_A320_SCROLL_DIVISOR_SLOW
#define ARROW_DIVISOR_FAST CONFIG_A320_SCROLL_DIVISOR_FAST

// --- 防误触锁定比例配置 ---
#define DOMINANT_NUMERATOR CONFIG_A320_DOMINANT_NUMERATOR
#define DOMINANT_DENOMINATOR CONFIG_A320_DOMINANT_DENOMINATOR

// --- 鼠标指针基础配置 (Kconfig 为整数百分比，这里除以 100 转为浮点数) ---
#define MOUSE_BASE_SPEED (CONFIG_A320_MOUSE_BASE_SPEED_PERCENT / 100.0f)
#define MOUSE_SENS_BASE (CONFIG_A320_MOUSE_SENS_BASE_PERCENT / 100.0f)
#define MOUSE_SENS_STEP (CONFIG_A320_MOUSE_SENS_STEP_PERCENT / 100.0f)

/* ========= Motion GPIO ========= */

#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 5
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

/* ========= A320 常量 ========= */
#define A320_I2C_ADDR 0x3B
#define A320_PACKET_LEN 3

#define SLOW_KEY_MULTIPLIER 0.5f
#define TOUCH_IDLE_TIMEOUT 50 // 30~80ms 看手感
/* ========= Watch Dog ========= */
static uint32_t last_activity_time = 0;
#define A320_WDT_TIMEOUT 200
/* ========= 全局状态 ========= */
static bool arrow_key_pressed = false;
static bool slow_key_pressed = false;
static bool last_arrow_key_pressed = false;
static bool last_slow_key_pressed = false;
uint32_t last_packet_time = 0;
static bool touched = false;
static uint32_t last_touch_time = 0;

/* ==== HID indicators ==== */
/* ========= Direction + precision override key listener ========= */
static int special_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev)
        return 0;
    if (ev->position == 34) {
        arrow_key_pressed = ev->state;
        LOG_INF("Arrow position=34 %s", arrow_key_pressed ? "PRESSED" : "RELEASED");
    }

    /* Holding position 36 temporarily switches the default scroll surface to
     * the existing half-speed precision pointer mode. */
    if (ev->position == 36) {
        slow_key_pressed = ev->state;
        LOG_INF("precision pointer position=36 %s", slow_key_pressed ? "PRESSED" : "RELEASED");
    }

    return 0;
}
ZMK_LISTENER(a320_special_key_listener, special_key_listener_cb);
ZMK_SUBSCRIPTION(a320_special_key_listener, zmk_position_state_changed);

struct a320_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct a320_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work; // ⭐ 新增
    uint32_t last_packet_time;
    int16_t scroll_residue_x;
    int16_t scroll_residue_y;
    int16_t arrow_residue_x;
    int16_t arrow_residue_y;
};

/* ========= I2C 读取（加锁版） ========= */
static int a320_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct a320_config *cfg = dev->config;
    uint8_t buf[A320_PACKET_LEN] = {0};
    uint8_t reg = 0x82;

    int ret = 0;

    k_mutex_lock(&a320_i2c_mutex, K_FOREVER);

    ret = i2c_write_dt(&cfg->i2c, &reg, 1);
    if (ret < 0)
        goto out;

    ret = i2c_burst_read_dt(&cfg->i2c, 0x82, buf, sizeof(buf));
    if (ret < 0)
        goto out;

    *dx = (int8_t)buf[1];
    *dy = -(int8_t)buf[2];

out:
    k_mutex_unlock(&a320_i2c_mutex);
    return ret;
}

/* ========= ★ 抽象复用：滚轮单轴处理函数 ========= */
static inline void process_scroll_axis(const struct device *dev, int16_t delta, int16_t *residue,
                                       uint16_t input_code, int8_t dir_mult) {
    int abs_delta = abs(delta);

    // ★ 不清零，保持连续性
    if (abs_delta <= SCROLL_DEADZONE) {
        return;
    }

    if (abs_delta > SCROLL_INPUT_MAX) {
        abs_delta = SCROLL_INPUT_MAX;
    }

    // ★ 非线性 divisor（更丝滑）
    float t = (float)abs_delta / SCROLL_INPUT_MAX;
    t = t * t;

    float f_div = SCROLL_DIVISOR_SLOW - (SCROLL_DIVISOR_SLOW - SCROLL_DIVISOR_FAST) * t;

    int divisor = (int)f_div;
    if (divisor < 1)
        divisor = 1;

    *residue += (delta * dir_mult);

    int16_t scroll_ticks = *residue / divisor;
    if (scroll_ticks != 0) {
        input_report_rel(dev, input_code, scroll_ticks, true, K_NO_WAIT);
        *residue %= divisor;
    }

    // ★ 阻尼（关键）
    *residue = (*residue * 3) / 4;
}

static inline void process_arrow_axis(const struct device *dev, int16_t delta, int16_t *residue,
                                      uint16_t key_neg, uint16_t key_pos) {

    int abs_delta = abs(delta);

    if (abs_delta <= ARROW_DEADZONE) {
        return;
    }

    if (abs_delta > ARROW_INPUT_MAX) {
        abs_delta = ARROW_INPUT_MAX;
    }

    // ★ 非线性 divisor（更丝滑）
    float t = (float)abs_delta / SCROLL_INPUT_MAX;
    t = t * t;

    float f_div = SCROLL_DIVISOR_SLOW - (SCROLL_DIVISOR_SLOW - SCROLL_DIVISOR_FAST) * t;

    int divisor = (int)f_div;
    if (divisor < 1)
        divisor = 1;

    *residue += delta; // 替换掉 dir_mult
    int16_t arrow_ticks = *residue / divisor;
    if (arrow_ticks != 0) {
        uint16_t key = (arrow_ticks > 0) ? key_pos : key_neg;

        // 触发 key press + release（脉冲）
        input_report_key(dev, key, 1, true, K_FOREVER);
        input_report_key(dev, key, 0, true, K_FOREVER);

        *residue %= divisor;
    }

    // 阻尼（防止漂移）
    *residue = (*residue * 3) / 4;
}

static void a320_work_cb(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, work);
    const struct device *dev = data->dev;

    uint32_t now = k_uptime_get_32();

    /* ========= WATCHDOG ========= */
    if (now - last_activity_time > A320_WDT_TIMEOUT) {
        LOG_WRN("A320 watchdog recovery");

        data->scroll_residue_x = 0;
        data->scroll_residue_y = 0;
        data->arrow_residue_x = 0;
        data->arrow_residue_y = 0;

        last_arrow_key_pressed = arrow_key_pressed;
        last_slow_key_pressed = slow_key_pressed;

        touched = false;
        return;
    }

    int8_t packet_dx = 0, packet_dy = 0;

    /* ========= ⭐ NEW: DRAIN MODE ========= */
    int16_t total_dx = 0;
    int16_t total_dy = 0;
    bool got_data = false;

    while (1) {
        int ret = a320_read_packet(dev, &packet_dx, &packet_dy);

        if (ret != 0) {
            break;
        }

        /* 防止异常空包 */
        if (packet_dx == 0 && packet_dy == 0) {
            break;
        }

        total_dx += packet_dx;
        total_dy += packet_dy;
        got_data = true;
    }

    /* ========= ⭐ TOUCH TIME TRACK ========= */
    if (got_data) {
        last_touch_time = now;
        touched = true;
    }

    /* ========= ⭐ TOUCH RELEASE 判定（关键修复） ========= */
    if (!got_data) {
        if (now - last_touch_time > TOUCH_IDLE_TIMEOUT) { // 30~80ms 可调
            touched = false;
        }
        return;
    }

    int16_t dx = total_dx;
    int16_t dy = total_dy;

    /* ========= 默认滚动 / 精细指针 / 方向键模式 ========= */
    bool just_enter_arrow = arrow_key_pressed && !last_arrow_key_pressed;
    bool just_enter_scroll = !arrow_key_pressed && !slow_key_pressed &&
                             (last_arrow_key_pressed || last_slow_key_pressed);

    if (arrow_key_pressed) {

        if (just_enter_arrow) {
            data->arrow_residue_x = 0;
            data->arrow_residue_y = 0;
        }

        int abs_dx = abs(dx);
        int abs_dy = abs(dy);

        if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
            dx = 0;
        } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
            dy = 0;
        } else {
            dx = 0;
            dy = 0;
        }

        process_arrow_axis(dev, dx, &data->arrow_residue_x, INPUT_BTN_1, INPUT_BTN_0);

        process_arrow_axis(dev, dy, &data->arrow_residue_y, INPUT_BTN_3, INPUT_BTN_2);
    } else if (!slow_key_pressed) {

        if (just_enter_scroll) {
            data->scroll_residue_x = 0;
            data->scroll_residue_y = 0;
        }

        int abs_dx = abs(dx);
        int abs_dy = abs(dy);

        if (abs_dy * DOMINANT_DENOMINATOR > abs_dx * DOMINANT_NUMERATOR) {
            dx = 0;
        } else if (abs_dx * DOMINANT_DENOMINATOR > abs_dy * DOMINANT_NUMERATOR) {
            dy = 0;
        } else {
            dx = 0;
            dy = 0;
        }

        process_scroll_axis(dev, -1 * dx, &data->scroll_residue_x, INPUT_REL_HWHEEL, SCROLL_X_DIR);

        process_scroll_axis(dev, -1 * dy, &data->scroll_residue_y, INPUT_REL_WHEEL, SCROLL_Y_DIR);
    } else {

        uint8_t a320_led_brt = indicator_tp_get_last_valid_brightness();
        float a320_factor = 0.4f + 0.01f * a320_led_brt;

        float fx = dx * 3 / 4 * a320_factor * SLOW_KEY_MULTIPLIER;
        float fy = dy * 3 / 4 * a320_factor * SLOW_KEY_MULTIPLIER;

        input_report_rel(dev, INPUT_REL_X, (int)fx, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, (int)fy, true, K_NO_WAIT);
    }

    last_arrow_key_pressed = arrow_key_pressed;
    last_slow_key_pressed = slow_key_pressed;
    data->last_packet_time = now;
}

/* ========= GPIO ISR ========= */
static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb_data);

    last_activity_time = k_uptime_get_32();

    /* ⭐ 防止 work 堆积 */
    k_work_submit_to_queue(&a320_workq, &data->work);
}

bool tp_is_touched(void) {
    return touched && (k_uptime_get_32() - last_touch_time) <= TOUCH_IDLE_TIMEOUT;
}

static void a320_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, enable_irq_work);
    const struct device *dev = data->dev;
    const struct a320_config *cfg = dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    LOG_INF("A320 IRQ enabled (delayed)");
}
/* ========= 初始化 ========= */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c))
        return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->motion_gpio))
        return -ENODEV;

    /* ⭐ 初始化 mutex */
    k_mutex_init(&a320_i2c_mutex);

    data->dev = dev;

    k_work_init(&data->work, a320_work_cb);

    /* ⭐ 启动 workqueue */
    k_work_queue_start(&a320_workq, a320_workq_stack, K_THREAD_STACK_SIZEOF(a320_workq_stack),
                       A320_WORKQ_PRIORITY, NULL);

    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);

    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);

    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);

    k_work_init_delayable(&data->enable_irq_work, a320_enable_irq_work_cb);
    k_work_schedule(&data->enable_irq_work, K_MSEC(200));

    LOG_INF("A320 Driver Initialized (I2C mutex enabled)");
    return 0;
}

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static const struct a320_config a320_config_##inst = {                                         \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                   \
                        .pin = MOTION_GPIO_PIN,                                                    \
                        .dt_flags = MOTION_GPIO_FLAGS},                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_config_##inst,           \
                          POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE);
