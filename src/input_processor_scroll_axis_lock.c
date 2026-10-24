/*
 * Locks scrolling to one axis per gesture, so a vertical scroll does not wobble sideways
 * (and vice versa). Mirrors the dominant-axis behaviour of the trackpad's native scroll.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_axis_lock

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum scroll_axis {
    SCROLL_AXIS_NONE,
    SCROLL_AXIS_V,
    SCROLL_AXIS_H,
};

struct zip_scroll_axis_lock_config {
    int32_t threshold;
    int32_t release_ms;
};

struct zip_scroll_axis_lock_data {
    // HWHEEL waiting for its paired WHEEL
    int32_t pending_h;
    bool has_pending_h;

    enum scroll_axis axis;
    // Movement accumulated while the axis is still undecided
    int32_t acc_h;
    int32_t acc_v;
    int64_t last_time;
};

static int scroll_axis_lock_handle_event(const struct device *dev, struct input_event *event,
                                         uint32_t param1, uint32_t param2,
                                         struct zmk_input_processor_state *state) {
    const struct zip_scroll_axis_lock_config *cfg = dev->config;
    struct zip_scroll_axis_lock_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // HWHEEL arrives first (without sync); hold it until WHEEL completes the report
    if (event->code == INPUT_REL_HWHEEL) {
        data->pending_h = event->value;
        data->has_pending_h = true;
        event->value = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code != INPUT_REL_WHEEL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t h = data->has_pending_h ? data->pending_h : 0;
    int32_t v = event->value;
    data->has_pending_h = false;

    int64_t now = k_uptime_get();
    if (now - data->last_time > cfg->release_ms) {
        data->axis = SCROLL_AXIS_NONE;
        data->acc_h = 0;
        data->acc_v = 0;
    }
    if (h != 0 || v != 0) {
        data->last_time = now;
    }

    if (data->axis == SCROLL_AXIS_NONE) {
        data->acc_h += abs(h);
        data->acc_v += abs(v);
        if (MAX(data->acc_h, data->acc_v) < cfg->threshold) {
            // Too little movement to tell the direction yet
            event->value = 0;
            return ZMK_INPUT_PROC_CONTINUE;
        }
        data->axis = data->acc_h > data->acc_v ? SCROLL_AXIS_H : SCROLL_AXIS_V;
    }

    // WHEEL carries the sync, so a horizontal lock re-codes it instead of the held HWHEEL
    if (data->axis == SCROLL_AXIS_H) {
        event->code = INPUT_REL_HWHEEL;
        event->value = (int16_t)CLAMP(h, INT16_MIN, INT16_MAX);
    }
    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api scroll_axis_lock_driver_api = {
    .handle_event = scroll_axis_lock_handle_event,
};

#define ZIP_SCROLL_AXIS_LOCK_INST(n)                                                               \
    static const struct zip_scroll_axis_lock_config zip_scroll_axis_lock_config_##n = {            \
        .threshold = DT_INST_PROP(n, threshold),                                                   \
        .release_ms = DT_INST_PROP(n, release_ms),                                                 \
    };                                                                                             \
    static struct zip_scroll_axis_lock_data zip_scroll_axis_lock_data_##n;                         \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &zip_scroll_axis_lock_data_##n,                           \
                          &zip_scroll_axis_lock_config_##n, POST_KERNEL,                           \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &scroll_axis_lock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_SCROLL_AXIS_LOCK_INST)
