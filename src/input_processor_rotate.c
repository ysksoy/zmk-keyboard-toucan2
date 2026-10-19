/*
 * Rotates relative X/Y movement by a fixed angle to compensate for a tilted trackpad.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_rotate

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Fixed-point scale for sin/cos
#define ROT_SCALE 1024

struct zip_rotate_config {
    int32_t degrees;
    int32_t carry_timeout_ms;
};

struct zip_rotate_data {
    int32_t cos_val; // cos * ROT_SCALE
    int32_t sin_val; // sin * ROT_SCALE

    // Raw X waiting for its paired Y
    int32_t pending_x;
    bool has_pending_x;

    // Rotated X emitted on the next X event
    int32_t carry_x;
    int64_t carry_time;

    // Sub-unit remainders (in ROT_SCALE units) so slow movement is not lost
    int32_t rem_x;
    int32_t rem_y;
};

// sin/cos via Taylor series; only run at init, avoids depending on libm
static double rot_sin(double r) {
    double term = r, sum = r, r2 = r * r;
    for (int n = 1; n < 10; n++) {
        term *= -r2 / ((2 * n) * (2 * n + 1));
        sum += term;
    }
    return sum;
}

static double rot_cos(double r) {
    double term = 1.0, sum = 1.0, r2 = r * r;
    for (int n = 1; n < 10; n++) {
        term *= -r2 / ((2 * n - 1) * (2 * n));
        sum += term;
    }
    return sum;
}

static int32_t take_scaled(int32_t *acc) {
    int32_t out = *acc / ROT_SCALE;
    *acc -= out * ROT_SCALE;
    return out;
}

static int rotate_handle_event(const struct device *dev, struct input_event *event,
                               uint32_t param1, uint32_t param2,
                               struct zmk_input_processor_state *state) {
    const struct zip_rotate_config *cfg = dev->config;
    struct zip_rotate_data *data = dev->data;

    if (event->type != INPUT_EV_REL || cfg->degrees == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        data->pending_x = event->value;
        data->has_pending_x = true;

        int32_t out = 0;
        if (data->carry_x != 0 &&
            k_uptime_get() - data->carry_time <= cfg->carry_timeout_ms) {
            out = data->carry_x;
        }
        data->carry_x = 0;
        event->value = (int16_t)CLAMP(out, INT16_MIN, INT16_MAX);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        int32_t x = data->has_pending_x ? data->pending_x : 0;
        int32_t y = event->value;
        data->has_pending_x = false;

        // Screen coordinates (Y down); positive degrees undo a counter-clockwise tilt.
        //   x' =  x*cos + y*sin
        //   y' = -x*sin + y*cos
        data->rem_x += x * data->cos_val + y * data->sin_val;
        data->rem_y += -x * data->sin_val + y * data->cos_val;

        data->carry_x += take_scaled(&data->rem_x);
        data->carry_time = k_uptime_get();
        event->value = (int16_t)CLAMP(take_scaled(&data->rem_y), INT16_MIN, INT16_MAX);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api rotate_driver_api = {
    .handle_event = rotate_handle_event,
};

static int rotate_init(const struct device *dev) {
    const struct zip_rotate_config *cfg = dev->config;
    struct zip_rotate_data *data = dev->data;

    double rad = (double)cfg->degrees * 3.14159265358979 / 180.0;
    data->cos_val = (int32_t)(rot_cos(rad) * ROT_SCALE + 0.5);
    double s = rot_sin(rad) * ROT_SCALE;
    data->sin_val = (int32_t)(s >= 0 ? s + 0.5 : s - 0.5);

    LOG_DBG("Rotate %d deg: cos=%d sin=%d (/%d)", cfg->degrees, data->cos_val, data->sin_val,
            ROT_SCALE);
    return 0;
}

#define ZIP_ROTATE_INST(n)                                                                         \
    BUILD_ASSERT(DT_INST_PROP(n, rotation_degrees) >= -90 &&                                       \
                     DT_INST_PROP(n, rotation_degrees) <= 90,                                      \
                 "rotation-degrees must be between -90 and 90");                                   \
    static const struct zip_rotate_config zip_rotate_config_##n = {                                \
        .degrees = DT_INST_PROP(n, rotation_degrees),                                              \
        .carry_timeout_ms = DT_INST_PROP(n, carry_timeout_ms),                                     \
    };                                                                                             \
    static struct zip_rotate_data zip_rotate_data_##n;                                             \
    DEVICE_DT_INST_DEFINE(n, rotate_init, NULL, &zip_rotate_data_##n, &zip_rotate_config_##n,      \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &rotate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_ROTATE_INST)
