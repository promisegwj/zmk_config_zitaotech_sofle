/*
 * bbtrackball_input_handler.c
 * BB Trackball FULL interrupt-driven version
 *
 * + Dedicated workqueue version (NO system workqueue)
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_bbtrackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>

#include <math.h>

LOG_MODULE_REGISTER(bbtrackball_input_handler, LOG_LEVEL_INF);

/* =========================================================
 * Workqueue config  ⭐⭐⭐新增
 * ========================================================= */

#define BBTRACKBALL_WORKQ_STACK_SIZE 2048
#define BBTRACKBALL_WORKQ_PRIORITY 5

K_THREAD_STACK_DEFINE(bbtrackball_workq_stack, BBTRACKBALL_WORKQ_STACK_SIZE);
static struct k_work_q bbtrackball_work_q;

/* =========================================================
 * GPIO Pins
 * ========================================================= */

#define DOWN_GPIO_PIN 9
#define LEFT_GPIO_PIN 12
#define UP_GPIO_PIN 5
#define RIGHT_GPIO_PIN 27

#define GPIO0_DEV DT_NODELABEL(gpio0)
#define GPIO1_DEV DT_NODELABEL(gpio1)

/* =========================================================
 * Config
 * ========================================================= */

#define SCROLL_STEP 1
#define SCROLL_ACCEL_BASE 1.08f
#define SCROLL_ACCEL_SCALE 45.0f
#define SCROLL_DELTA_MAX 8

/* =========================================================
 * Runtime State
 * ========================================================= */

static struct k_spinlock acc_lock;

static int dx_acc = 0;
static int dy_acc = 0;

static uint32_t last_move_time = 0;

/* =========================================================
 * GPIO Input Description
 * ========================================================= */

typedef struct {
    const struct device *gpio_dev;
    int pin;
    int last_state;
    uint32_t last_time;
    int sign;
} DirInput;

static DirInput dir_inputs[] = {
    {DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, 0, -1},
    {DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, 0, +1},
    {DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, 0, -1},
    {DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, 0, +1},
};

/* ========================================================= */

struct bbtrackball_data;

/* ========================================================= */

struct bb_gpio_cb {
    struct gpio_callback cb;
    struct bbtrackball_data *parent;
};

struct bbtrackball_data {
    const struct device *dev;
    struct k_work work;
    struct bb_gpio_cb gpio_cbs[ARRAY_SIZE(dir_inputs)];
};

/* ========================================================= */

bool trackball_is_active(void) { return (k_uptime_get_32() - last_move_time) < 40; }

/* ========================================================= */

static void report_scroll(const struct device *dev, int dx, int dy) {
    input_report_rel(dev, INPUT_REL_HWHEEL, -dx, false, K_NO_WAIT);
    input_report_rel(dev, INPUT_REL_WHEEL, dy, true, K_NO_WAIT);
}

static int scroll_delta_from_interval(uint32_t delta_ms) {
    if (delta_ms == 0) {
        delta_ms = 1;
    }

    float speed_factor = SCROLL_ACCEL_SCALE / (float)delta_ms;
    float mult = powf(SCROLL_ACCEL_BASE, speed_factor);
    int delta = (int)roundf(SCROLL_STEP * mult);

    if (delta < 1) {
        return 1;
    }

    if (delta > SCROLL_DELTA_MAX) {
        return SCROLL_DELTA_MAX;
    }

    return delta;
}

/* =========================================================
 * GPIO interrupt callback
 * ========================================================= */

static void dir_edge_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {

    struct bb_gpio_cb *wrapper = CONTAINER_OF(cb, struct bb_gpio_cb, cb);
    struct bbtrackball_data *data = wrapper->parent;

    for (size_t i = 0; i < ARRAY_SIZE(dir_inputs); i++) {

        DirInput *d = &dir_inputs[i];

        if ((dev == d->gpio_dev) && (pins & BIT(d->pin))) {

            int val = gpio_pin_get(dev, d->pin);

            if (val != d->last_state) {

                uint32_t now = k_uptime_get_32();
                uint32_t delta_ms = now - d->last_time;
                int delta_px = scroll_delta_from_interval(delta_ms);

                k_spinlock_key_t key = k_spin_lock(&acc_lock);
                if (i < 2) {
                    dx_acc += d->sign * delta_px;
                } else {
                    dy_acc += d->sign * delta_px;
                }
                k_spin_unlock(&acc_lock, key);

                d->last_state = val;
                d->last_time = now;

                k_work_submit_to_queue(&bbtrackball_work_q, &data->work);
            }
        }
    }
}

/* =========================================================
 * Work Handler
 * ========================================================= */

static void bbtrackball_work_handler(struct k_work *work) {

    struct bbtrackball_data *data = CONTAINER_OF(work, struct bbtrackball_data, work);
    const struct device *dev = data->dev;

    while (1) {
        k_spinlock_key_t key = k_spin_lock(&acc_lock);

        int dx = dx_acc;
        int dy = dy_acc;

        dx_acc = 0;
        dy_acc = 0;

        k_spin_unlock(&acc_lock, key);

        if (dx == 0 && dy == 0) {
            return;
        }

        last_move_time = k_uptime_get_32();
        report_scroll(dev, dx, dy);
    }
}

/* =========================================================
 * Init
 * ========================================================= */

static int bbtrackball_init(const struct device *dev) {

    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball");

    data->dev = dev;

    /* ⭐ 启动独立 workqueue */
    k_work_queue_start(&bbtrackball_work_q, bbtrackball_workq_stack,
                       K_THREAD_STACK_SIZEOF(bbtrackball_workq_stack), BBTRACKBALL_WORKQ_PRIORITY,
                       NULL);

    k_work_init(&data->work, bbtrackball_work_handler);

    for (size_t i = 0; i < ARRAY_SIZE(dir_inputs); i++) {

        DirInput *d = &dir_inputs[i];

        gpio_pin_configure(d->gpio_dev, d->pin, GPIO_INPUT | GPIO_PULL_UP);

        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);
        d->last_time = k_uptime_get_32();

        data->gpio_cbs[i].parent = data;

        gpio_init_callback(&data->gpio_cbs[i].cb, dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &data->gpio_cbs[i].cb);

        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, GPIO_INT_EDGE_BOTH);
    }

    return 0;
}

/* ========================================================= */

#define BBTRACKBALL_INIT_PRIORITY CONFIG_INPUT_INIT_PRIORITY

#define BBTRACKBALL_DEFINE(inst)                                                                   \
    static struct bbtrackball_data bbtrackball_data_##inst;                                        \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, bbtrackball_init, NULL, &bbtrackball_data_##inst,                  \
                          NULL, POST_KERNEL, BBTRACKBALL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BBTRACKBALL_DEFINE);
