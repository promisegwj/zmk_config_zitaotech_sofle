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
#include <zephyr/sys/atomic.h>

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

#define SCROLL_EDGE_IMPULSE 2
#define SCROLL_DRAIN_INTERVAL_MS 8
#define SCROLL_MEDIUM_BACKLOG_THRESHOLD 3
#define SCROLL_FAST_BACKLOG_THRESHOLD 5
#define SCROLL_REPORT_MAX_PER_TICK 3

/* Diagnostic: disable vertical latch/repeat synthetic output. */
#define BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT 1

#define AUTO_SCROLL_TRIGGER_EDGES 8
#define AUTO_SCROLL_TRIGGER_WINDOW_MS 280
#define AUTO_SCROLL_REPEAT_DIVISOR 2
#define AUTO_SCROLL_REPEAT_STEP 1

#define BBTRACKBALL_EDGE_LOG_LIMIT 8
#define BBTRACKBALL_REPORT_LOG_LIMIT 12
#define BBTRACKBALL_AUTO_LOG_LIMIT 4
#define BBTRACKBALL_AUTO_REPORT_LOG_LIMIT 6

/* =========================================================
 * Runtime State
 * ========================================================= */

static struct k_spinlock acc_lock;
static atomic_t scroll_work_active;

static int dx_acc = 0;
static int dy_acc = 0;

#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
static int auto_scroll_dir = 0;
static int vertical_streak_dir = 0;
static uint8_t vertical_streak_count = 0;
static uint32_t vertical_streak_last_time = 0;
static uint8_t auto_scroll_repeat_divider = 0;
static uint8_t auto_scroll_enter_log_count = 0;
static uint8_t auto_scroll_stop_log_count = 0;
static uint8_t auto_scroll_repeat_log_count = 0;
#endif

static uint32_t last_move_time = 0;

/* =========================================================
 * GPIO Input Description
 * ========================================================= */

typedef struct {
    const char *name;
    const struct device *gpio_dev;
    int pin;
    int last_state;
    uint32_t last_time;
    int sign;
    uint8_t edge_log_count;
} DirInput;

static DirInput dir_inputs[] = {
    {"LEFT", DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, 0, -1, 0},
    {"RIGHT", DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, 0, +1, 0},
    {"UP", DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, 0, -1, 0},
    {"DOWN", DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, 0, +1, 0},
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
    static uint8_t report_log_count;

    if (report_log_count < BBTRACKBALL_REPORT_LOG_LIMIT && (dx != 0 || dy != 0)) {
        LOG_INF("BBtrackball report dx=%d hwheel=%d dy=%d wheel=%d", dx, -dx, dy, dy);
        report_log_count++;
    }

    input_report_rel(dev, INPUT_REL_HWHEEL, -dx, false, K_NO_WAIT);
    input_report_rel(dev, INPUT_REL_WHEEL, dy, true, K_NO_WAIT);
}

static void submit_scroll_work(struct bbtrackball_data *data) {
    if (atomic_cas(&scroll_work_active, 0, 1)) {
        int ret = k_work_submit_to_queue(&bbtrackball_work_q, &data->work);

        if (ret < 0) {
            atomic_set(&scroll_work_active, 0);
            LOG_WRN("BBtrackball scroll work submit failed: %d", ret);
        }
    }
}

static int take_scroll_tick_delta(int *value) {
    int max_delta = 1;

    if (*value >= SCROLL_FAST_BACKLOG_THRESHOLD || *value <= -SCROLL_FAST_BACKLOG_THRESHOLD) {
        max_delta = SCROLL_REPORT_MAX_PER_TICK;
    } else if (*value >= SCROLL_MEDIUM_BACKLOG_THRESHOLD ||
               *value <= -SCROLL_MEDIUM_BACKLOG_THRESHOLD) {
        max_delta = 2;
    }

    if (*value > 0) {
        int delta = *value < max_delta ? *value : max_delta;
        *value -= delta;
        return delta;
    }

    if (*value < 0) {
        int delta = *value > -max_delta ? *value : -max_delta;
        *value -= delta;
        return delta;
    }

    return 0;
}

#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
static bool process_vertical_auto_latch(int dir, uint32_t now, bool *auto_entered,
                                        bool *auto_stopped) {
    *auto_entered = false;
    *auto_stopped = false;

    if (auto_scroll_dir != 0) {
        if (dir == -auto_scroll_dir) {
            auto_scroll_dir = 0;
            vertical_streak_dir = 0;
            vertical_streak_count = 0;
            vertical_streak_last_time = now;
            auto_scroll_repeat_divider = 0;
            *auto_stopped = true;
            return false;
        }

        vertical_streak_dir = dir;
        vertical_streak_count = 0;
        vertical_streak_last_time = now;
        auto_scroll_repeat_divider = 0;
        return true;
    }

    if (vertical_streak_dir == dir &&
        (now - vertical_streak_last_time) <= AUTO_SCROLL_TRIGGER_WINDOW_MS) {
        if (vertical_streak_count < AUTO_SCROLL_TRIGGER_EDGES) {
            vertical_streak_count++;
        }
    } else {
        vertical_streak_dir = dir;
        vertical_streak_count = 1;
    }

    vertical_streak_last_time = now;

    if (vertical_streak_count >= AUTO_SCROLL_TRIGGER_EDGES) {
        auto_scroll_dir = dir;
        vertical_streak_count = 0;
        auto_scroll_repeat_divider = 0;
        *auto_entered = true;
    }

    return true;
}
#endif

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
                int signed_impulse = d->sign * SCROLL_EDGE_IMPULSE;
                bool apply_impulse = true;
#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
                bool auto_entered = false;
                bool auto_stopped = false;
                bool log_auto_enter = false;
                bool log_auto_stop = false;
#endif

                k_spinlock_key_t key = k_spin_lock(&acc_lock);
                if (i < 2) {
                    dx_acc += signed_impulse;
                } else {
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
                    dy_acc += signed_impulse;
#else
                    apply_impulse = process_vertical_auto_latch(d->sign, now, &auto_entered,
                                                                &auto_stopped);
                    if (apply_impulse) {
                        dy_acc += signed_impulse;
                    }

                    if (auto_entered &&
                        auto_scroll_enter_log_count < BBTRACKBALL_AUTO_LOG_LIMIT) {
                        auto_scroll_enter_log_count++;
                        log_auto_enter = true;
                    }

                    if (auto_stopped && auto_scroll_stop_log_count < BBTRACKBALL_AUTO_LOG_LIMIT) {
                        auto_scroll_stop_log_count++;
                        log_auto_stop = true;
                    }
#endif
                }
                k_spin_unlock(&acc_lock, key);

                if (d->edge_log_count < BBTRACKBALL_EDGE_LOG_LIMIT) {
                    LOG_INF("BBtrackball edge %s gpio=%s.%d state=%d delta_ms=%u impulse=%d signed=%d applied=%d",
                            d->name, d->gpio_dev->name, d->pin, val, delta_ms, SCROLL_EDGE_IMPULSE,
                            signed_impulse, apply_impulse ? 1 : 0);
                    d->edge_log_count++;
                }

#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
                if (log_auto_enter) {
                    LOG_INF("BBtrackball auto-scroll enter dir=%d trigger_edges=%d window_ms=%d",
                            d->sign, AUTO_SCROLL_TRIGGER_EDGES, AUTO_SCROLL_TRIGGER_WINDOW_MS);
                }

                if (log_auto_stop) {
                    LOG_INF("BBtrackball auto-scroll stop dir=%d by=%s", -d->sign, d->name);
                }
#endif

                d->last_state = val;
                d->last_time = now;

                submit_scroll_work(data);
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
        k_sleep(K_MSEC(SCROLL_DRAIN_INTERVAL_MS));

        k_spinlock_key_t key = k_spin_lock(&acc_lock);

        int dx = take_scroll_tick_delta(&dx_acc);
        int dy = take_scroll_tick_delta(&dy_acc);
#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        bool auto_active = auto_scroll_dir != 0;
        bool log_auto_report = false;

        if (dx == 0 && dy == 0 && auto_active) {
            auto_scroll_repeat_divider++;
            if (auto_scroll_repeat_divider >= AUTO_SCROLL_REPEAT_DIVISOR) {
                auto_scroll_repeat_divider = 0;
                dy = auto_scroll_dir * AUTO_SCROLL_REPEAT_STEP;

                if (auto_scroll_repeat_log_count < BBTRACKBALL_AUTO_REPORT_LOG_LIMIT) {
                    auto_scroll_repeat_log_count++;
                    log_auto_report = true;
                }
            }
        }
#endif

        k_spin_unlock(&acc_lock, key);

#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        if (log_auto_report) {
            LOG_INF("BBtrackball auto-scroll repeat dy=%d", dy);
        }
#endif

        if (dx != 0 || dy != 0) {
            last_move_time = k_uptime_get_32();
            report_scroll(dev, dx, dy);
            continue;
        }

#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        if (auto_active) {
            continue;
        }
#endif

        atomic_set(&scroll_work_active, 0);

        key = k_spin_lock(&acc_lock);
        bool has_pending = dx_acc != 0 || dy_acc != 0;
#if !BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        has_pending = has_pending || auto_scroll_dir != 0;
#endif
        k_spin_unlock(&acc_lock, key);

        if (!has_pending) {
            return;
        }

        atomic_set(&scroll_work_active, 1);
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
        d->edge_log_count = 0;

        LOG_INF("BBtrackball input %s gpio=%s.%d initial=%d sign=%d", d->name,
                d->gpio_dev->name, d->pin, d->last_state, d->sign);

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
