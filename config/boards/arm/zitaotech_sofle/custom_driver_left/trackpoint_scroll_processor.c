/*
 * Central-side TrackPoint scroll gate for ZitaoTech Sofle.
 *
 * The right half is a split peripheral, so it must not query keymap state
 * directly. This input processor runs on the left/central half and converts
 * TrackPoint X/Y input into scroll only when the central keymap state allows it.
 */

#define DT_DRV_COMPAT zitaotech_trackpoint_scroll_processor

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/input_processor.h>
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/hid.h>

LOG_MODULE_REGISTER(zitaotech_trackpoint_scroll, LOG_LEVEL_INF);

#define TRACKPOINT_SCROLL_POSITION 61
#define HID_INDICATORS_CAPS_LOCK (1 << 1)

/* TrackPoint movement is already scaled by the peripheral before it reaches
 * this central processor, so keep this divisor much lower than the raw driver
 * scroll divisor.
 */
#define TRACKPOINT_SCROLL_DIVISOR 8

struct trackpoint_scroll_processor_data {
    int16_t h_remainder;
    int16_t v_remainder;
    bool was_active;
};

static bool trackpoint_scroll_pressed;
static zmk_hid_indicators_t current_indicators;

static bool trackpoint_scroll_active(void) {
    if (current_indicators & HID_INDICATORS_CAPS_LOCK) {
        return true;
    }

    return trackpoint_scroll_pressed;
}

static int16_t scroll_ticks(int16_t *remainder, int32_t value, int8_t direction) {
    *remainder += value * direction;

    int16_t ticks = *remainder / TRACKPOINT_SCROLL_DIVISOR;
    if (ticks != 0) {
        *remainder %= TRACKPOINT_SCROLL_DIVISOR;
    }

    return ticks;
}

static int trackpoint_scroll_handle_event(const struct device *dev, struct input_event *event,
                                          uint32_t param1, uint32_t param2,
                                          struct zmk_input_processor_state *state) {
    struct trackpoint_scroll_processor_data *data = dev->data;
    bool active = trackpoint_scroll_active();

    (void)param1;
    (void)param2;
    (void)state;

    if (!active) {
        data->h_remainder = 0;
        data->v_remainder = 0;
        data->was_active = false;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (!data->was_active) {
        data->h_remainder = 0;
        data->v_remainder = 0;
        data->was_active = true;
    }

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    switch (event->code) {
    case INPUT_REL_X:
        event->code = INPUT_REL_HWHEEL;
        event->value = scroll_ticks(&data->h_remainder, event->value, 1);
        break;
    case INPUT_REL_Y:
        event->code = INPUT_REL_WHEEL;
        event->value = scroll_ticks(&data->v_remainder, event->value, -1);
        break;
    default:
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->value == 0 && !event->sync) {
        return ZMK_INPUT_PROC_STOP;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static int trackpoint_scroll_position_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    if (ev && ev->position == TRACKPOINT_SCROLL_POSITION) {
        trackpoint_scroll_pressed = ev->state;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

static int trackpoint_scroll_hid_cb(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);

    if (ev) {
        current_indicators = ev->indicators;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(trackpoint_scroll_position_listener, trackpoint_scroll_position_cb);
ZMK_SUBSCRIPTION(trackpoint_scroll_position_listener, zmk_position_state_changed);

ZMK_LISTENER(trackpoint_scroll_hid_listener, trackpoint_scroll_hid_cb);
ZMK_SUBSCRIPTION(trackpoint_scroll_hid_listener, zmk_hid_indicators_changed);

static const struct zmk_input_processor_driver_api trackpoint_scroll_driver_api = {
    .handle_event = trackpoint_scroll_handle_event,
};

#define TRACKPOINT_SCROLL_PROCESSOR_INST(n)                                                         \
    static struct trackpoint_scroll_processor_data trackpoint_scroll_data_##n = {};                  \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &trackpoint_scroll_data_##n, NULL, POST_KERNEL,            \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &trackpoint_scroll_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_SCROLL_PROCESSOR_INST)
