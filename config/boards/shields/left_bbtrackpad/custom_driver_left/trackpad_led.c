/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include <zmk/endpoints.h>
#include <zmk/hid_indicators.h>
#include "trackpad_led.h"

#define HID_INDICATORS_CAPS_LOCK (1 << 1)

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_trackpad_led),
             "CONFIG_ZMK_TRACKPAD_LED enabled but no zmk,trackpad_led chosen node found");

static const struct device *const led_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_trackpad_led));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define INDICATOR_LED_NUM_LEDS (DT_NUM_CHILD(DT_CHOSEN(zmk_trackpad_led)))

#define BRT_MIN 10
#define BRT_MAX 100
#define BRT_LOW 20
#define BRT_STEP 5

#define ANIMATION_INTERVAL_MS 20
#define POLLING_INTERVAL_MS 5

static struct k_work_delayable polling_work;
static struct k_work_delayable animation_work;

static bool capslock_on = false;
static bool animation_increasing = true;
static uint8_t brightness = BRT_MIN;

static uint8_t last_valid_brt = BRT_MAX;
static bool usb_mode = false;

static void set_led_brightness(uint8_t level) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED device not ready");
        return;
    }
    for (int i = 0; i < INDICATOR_LED_NUM_LEDS; i++) {
        int err = led_set_brightness(led_dev, i, level);
        if (err < 0) {
            LOG_ERR("Failed to set LED[%d] brightness: %d", i, err);
        }
    }
}

static void animation_work_handler(struct k_work *work) {
    if (!capslock_on)
        return;

    if (animation_increasing) {
        brightness += BRT_STEP;
        if (brightness >= BRT_MAX) {
            brightness = BRT_MAX;
            animation_increasing = false;
        }
    } else {
        brightness -= BRT_STEP;
        if (brightness <= BRT_LOW) {
            brightness = BRT_LOW;
            animation_increasing = true;
        }
    }

    set_led_brightness(brightness);
    k_work_reschedule(&animation_work, K_MSEC(ANIMATION_INTERVAL_MS));
}

static void polling_work_handler(struct k_work *work) {
    enum zmk_transport transport = zmk_endpoints_selected().transport;
    bool current_capslock = (zmk_hid_indicators_get_current_profile() & HID_INDICATORS_CAPS_LOCK);
    bool current_usb_mode = transport == ZMK_TRANSPORT_USB;

    /* CapsLock breathing has priority over the steady USB indicator. */
    if (current_capslock != capslock_on) {
        capslock_on = current_capslock;
        if (capslock_on) {
            brightness = BRT_MIN;
            animation_increasing = true;
            k_work_reschedule(&animation_work, K_NO_WAIT);
        } else {
            k_work_cancel_delayable(&animation_work);
            set_led_brightness(current_usb_mode ? BRT_MAX : 0);
        }
    }

    /* Normal state: USB is steadily on, BLE is off. Input activity is ignored. */
    if (!capslock_on && current_usb_mode != usb_mode) {
        set_led_brightness(current_usb_mode ? BRT_MAX : 0);
        LOG_INF("Trackpad LED transport mode: %s", current_usb_mode ? "USB steady" : "BLE off");
    }
    usb_mode = current_usb_mode;

    k_work_reschedule(&polling_work, K_MSEC(POLLING_INTERVAL_MS));
}

uint8_t indicator_tp_get_last_valid_brightness(void) { return last_valid_brt; }

static int indicator_tp_init(void) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("LED indicator_tp device not ready");
        return -ENODEV;
    }

    set_led_brightness(0);
    usb_mode = false;
    capslock_on = false;

    k_work_init_delayable(&polling_work, polling_work_handler);
    k_work_init_delayable(&animation_work, animation_work_handler);

    k_work_reschedule(&polling_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(indicator_tp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
