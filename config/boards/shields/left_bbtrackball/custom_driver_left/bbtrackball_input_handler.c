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
#define SCROLL_DRAIN_INTERVAL_MS 6
#define SCROLL_MEDIUM_BACKLOG_THRESHOLD 3
#define SCROLL_FAST_BACKLOG_THRESHOLD 5
#define SCROLL_REPORT_MAX_PER_TICK 3
#define VERTICAL_EDGE_DEBOUNCE_MS 2
#define VERTICAL_TAIL_BASE_MS 96
#define VERTICAL_TAIL_EXTEND_MS 16
#define VERTICAL_TAIL_MAX_MS 160
#define VERTICAL_TAIL_INITIAL_BUDGET 2
#define VERTICAL_TAIL_ADD_BUDGET 2
#define VERTICAL_TAIL_MAX_BUDGET 6
#define VERTICAL_TAIL_PUMP_DIVISOR 2
#define VERTICAL_TAIL_TRIGGER_WINDOW_MS 200
#define VERTICAL_TAIL_TRIGGER_EDGES 2
#define SMALL_MOTION_ASSIST_WINDOW_MS VERTICAL_TAIL_TRIGGER_WINDOW_MS
#define SMALL_MOTION_ASSIST_STEP 1
#define SCROLL_REAL_QUEUE_SIZE 32

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

#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
typedef struct {
    int values[SCROLL_REAL_QUEUE_SIZE];
    uint8_t head;
    uint8_t count;
} ScrollQueue;

static ScrollQueue horizontal_real_queue;
static ScrollQueue vertical_real_queue;
#else
static int dx_acc = 0;
static int dy_acc = 0;
#endif
static int vertical_tail_dir = 0;
static uint8_t vertical_tail_budget = 0;
static uint8_t vertical_tail_pump_divider = 0;
static uint32_t vertical_tail_until = 0;

#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
static int vertical_tail_candidate_dir = 0;
static uint8_t vertical_tail_candidate_count = 0;
static uint32_t vertical_tail_candidate_last_time = 0;
static int vertical_small_motion_assist_dir = 0;
static bool vertical_small_motion_assist_disabled = false;
static uint32_t vertical_small_motion_assist_edge_time = 0;
#endif

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

#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
static bool scroll_queue_has_pending(const ScrollQueue *queue) {
    return queue->count > 0;
}

static bool same_scroll_direction(int a, int b) {
    return (a > 0 && b > 0) || (a < 0 && b < 0);
}

static void enqueue_scroll_delta(ScrollQueue *queue, int delta) {
    if (queue->count > 0) {
        uint8_t tail = (queue->head + queue->count - 1) % ARRAY_SIZE(queue->values);
        if (same_scroll_direction(queue->values[tail], delta)) {
            queue->values[tail] += delta;
            return;
        }
    }

    if (queue->count == ARRAY_SIZE(queue->values)) {
        /* Bound stale backlog while preserving the newest direction changes. */
        queue->head = (queue->head + 1) % ARRAY_SIZE(queue->values);
        queue->count--;
    }

    uint8_t tail = (queue->head + queue->count) % ARRAY_SIZE(queue->values);
    queue->values[tail] = delta;
    queue->count++;
}

static int take_scroll_queue_delta(ScrollQueue *queue) {
    if (!scroll_queue_has_pending(queue)) {
        return 0;
    }

    int *front = &queue->values[queue->head];
    int delta = take_scroll_tick_delta(front);

    if (*front == 0) {
        queue->head = (queue->head + 1) % ARRAY_SIZE(queue->values);
        queue->count--;
        if (queue->count == 0) {
            queue->head = 0;
        }
    }

    return delta;
}

static void reset_vertical_tail_candidate(void) {
    vertical_tail_candidate_dir = 0;
    vertical_tail_candidate_count = 0;
    vertical_tail_candidate_last_time = 0;
}

static void arm_vertical_small_motion_assist(int dir, uint32_t now) {
    vertical_small_motion_assist_dir = dir;
    vertical_small_motion_assist_disabled = false;
    vertical_small_motion_assist_edge_time = now;
}

static void reset_vertical_small_motion_assist(void) {
    vertical_small_motion_assist_dir = 0;
    vertical_small_motion_assist_disabled = false;
    vertical_small_motion_assist_edge_time = 0;
}

static void update_vertical_small_motion_assist(int dir, uint32_t now) {
    if (vertical_small_motion_assist_dir == 0 ||
        dir == -vertical_small_motion_assist_dir ||
        (now - vertical_small_motion_assist_edge_time) > SMALL_MOTION_ASSIST_WINDOW_MS) {
        arm_vertical_small_motion_assist(dir, now);
        return;
    }

    if (dir == vertical_small_motion_assist_dir) {
        vertical_small_motion_assist_disabled = true;
        vertical_small_motion_assist_edge_time = now;
    }
}

static bool vertical_small_motion_assist_due(uint32_t now) {
    return vertical_small_motion_assist_dir != 0 &&
           !vertical_small_motion_assist_disabled &&
           (int32_t)(now - vertical_small_motion_assist_edge_time -
                     SMALL_MOTION_ASSIST_WINDOW_MS) >= 0;
}

static bool vertical_small_motion_assist_waiting(uint32_t now) {
    return vertical_small_motion_assist_dir != 0 &&
           !vertical_small_motion_assist_disabled &&
           !vertical_small_motion_assist_due(now);
}

static int take_vertical_small_motion_assist_delta(uint32_t now) {
    if (!vertical_small_motion_assist_due(now)) {
        return 0;
    }

    int delta = vertical_small_motion_assist_dir * SMALL_MOTION_ASSIST_STEP;
    reset_vertical_small_motion_assist();
    return delta;
}
#endif

static bool vertical_tail_is_live(uint32_t now) {
    return vertical_tail_dir != 0 && (int32_t)(vertical_tail_until - now) > 0;
}

static void clear_vertical_tail(void) {
    vertical_tail_dir = 0;
    vertical_tail_budget = 0;
    vertical_tail_pump_divider = 0;
    vertical_tail_until = 0;
}

static void refresh_vertical_tail(int dir, uint32_t now) {
    if (vertical_tail_dir != dir || !vertical_tail_is_live(now)) {
        vertical_tail_dir = dir;
        vertical_tail_budget = VERTICAL_TAIL_INITIAL_BUDGET;
    } else {
        uint8_t boosted = vertical_tail_budget + VERTICAL_TAIL_ADD_BUDGET;
        vertical_tail_budget =
            boosted > VERTICAL_TAIL_MAX_BUDGET ? VERTICAL_TAIL_MAX_BUDGET : boosted;
    }

    vertical_tail_pump_divider = 0;

    uint32_t tail_ms = VERTICAL_TAIL_BASE_MS;
    if (vertical_tail_budget > VERTICAL_TAIL_INITIAL_BUDGET) {
        tail_ms += (vertical_tail_budget - VERTICAL_TAIL_INITIAL_BUDGET) *
                   VERTICAL_TAIL_EXTEND_MS;
    }

    if (tail_ms > VERTICAL_TAIL_MAX_MS) {
        tail_ms = VERTICAL_TAIL_MAX_MS;
    }

    vertical_tail_until = now + tail_ms;
}

#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
static bool should_refresh_vertical_tail(int dir, uint32_t now) {
    if ((vertical_tail_dir != 0 && dir == -vertical_tail_dir) ||
        (vertical_tail_candidate_dir != 0 && dir == -vertical_tail_candidate_dir)) {
        clear_vertical_tail();
        reset_vertical_tail_candidate();
    }

    if (vertical_tail_candidate_dir == dir &&
        (now - vertical_tail_candidate_last_time) <= VERTICAL_TAIL_TRIGGER_WINDOW_MS) {
        if (vertical_tail_candidate_count < VERTICAL_TAIL_TRIGGER_EDGES) {
            vertical_tail_candidate_count++;
        }
    } else {
        vertical_tail_candidate_dir = dir;
        vertical_tail_candidate_count = 1;
    }

    vertical_tail_candidate_last_time = now;

    return vertical_tail_candidate_count >= VERTICAL_TAIL_TRIGGER_EDGES;
}
#endif

static int take_vertical_tail_delta(uint32_t now) {
    if (!vertical_tail_is_live(now)) {
        clear_vertical_tail();
        return 0;
    }

    if (vertical_tail_budget == 0) {
        return 0;
    }

    vertical_tail_pump_divider++;
    if (vertical_tail_pump_divider < VERTICAL_TAIL_PUMP_DIVISOR) {
        return 0;
    }

    vertical_tail_pump_divider = 0;
    vertical_tail_budget--;

    return vertical_tail_dir;
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

            bool is_vertical = i >= 2;
            int val = is_vertical ? 1 : gpio_pin_get(dev, d->pin);

            if (is_vertical || val != d->last_state) {

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

                if (is_vertical && delta_ms < VERTICAL_EDGE_DEBOUNCE_MS) {
                    continue;
                }

                k_spinlock_key_t key = k_spin_lock(&acc_lock);
                if (i < 2) {
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
                    enqueue_scroll_delta(&horizontal_real_queue, signed_impulse);
#else
                    dx_acc += signed_impulse;
#endif
                } else {
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
                    enqueue_scroll_delta(&vertical_real_queue, signed_impulse);
                    update_vertical_small_motion_assist(d->sign, now);
                    if (should_refresh_vertical_tail(d->sign, now)) {
                        refresh_vertical_tail(d->sign, now);
                    }
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

                if (!is_vertical) {
                    d->last_state = val;
                }
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

#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        int dx = take_scroll_queue_delta(&horizontal_real_queue);
        int dy = take_scroll_queue_delta(&vertical_real_queue);
#else
        int dx = take_scroll_tick_delta(&dx_acc);
        int dy = take_scroll_tick_delta(&dy_acc);
#endif
        uint32_t now = k_uptime_get_32();
        bool tail_active = vertical_tail_is_live(now);

        if (dy == 0 && tail_active
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
            && !scroll_queue_has_pending(&vertical_real_queue)
#endif
        ) {
            dy = take_vertical_tail_delta(now);
        }
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        if (dy == 0 && !tail_active && !scroll_queue_has_pending(&vertical_real_queue)) {
            dy = take_vertical_small_motion_assist_delta(now);
        }
#endif
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
            last_move_time = now;
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
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        bool has_pending = scroll_queue_has_pending(&horizontal_real_queue) ||
                           scroll_queue_has_pending(&vertical_real_queue);
#else
        bool has_pending = dx_acc != 0 || dy_acc != 0;
#endif
        uint32_t pending_now = k_uptime_get_32();
        has_pending = has_pending || vertical_tail_is_live(pending_now);
#if BBTRACKBALL_DIAG_DISABLE_VERTICAL_AUTO_REPEAT
        has_pending = has_pending ||
                      vertical_small_motion_assist_waiting(pending_now) ||
                      vertical_small_motion_assist_due(pending_now);
#endif
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
        bool is_vertical = i >= 2;
        gpio_flags_t input_flags = GPIO_INPUT | GPIO_PULL_UP;
        gpio_flags_t interrupt_flags = is_vertical ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_EDGE_BOTH;

        if (is_vertical) {
            input_flags |= GPIO_ACTIVE_LOW;
        }

        gpio_pin_configure(d->gpio_dev, d->pin, input_flags);

        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);
        d->last_time = k_uptime_get_32();
        d->edge_log_count = 0;

        LOG_INF("BBtrackball input %s gpio=%s.%d initial=%d sign=%d", d->name,
                d->gpio_dev->name, d->pin, d->last_state, d->sign);

        data->gpio_cbs[i].parent = data;

        gpio_init_callback(&data->gpio_cbs[i].cb, dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &data->gpio_cbs[i].cb);

        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, interrupt_flags);
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
