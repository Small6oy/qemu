/*
 * Stepper motor on GPIO pads
 *
 * A unipolar stepper wired the way a camera's pan and tilt motors are:
 * each coil's end on a GPIO pad (through a driver array such as a ULN2003
 * or a transistor stage), no controller between. "phase0" to "phase3" are
 * the GPIO numbers (port * 32 + pad) of the four coils in the order the
 * excitation table below energises them; the rotor moves one step forward
 * (position + 1) each time the table index goes up.
 *
 * Four coils, as the vendors' drivers excite them (a half-step table of
 * eight patterns, one or two coils on):
 *
 *   index  0    1    2    3    4    5    6    7
 *   0123   1000 1100 0100 0110 0010 0011 0001 1001
 *
 * A move of one index is a half step; the full-step sequences (indexes 1,
 * 3, 5, 7 or 0, 2, 4, 6) move two. Position counts half steps. With
 * "phase2" and "phase3" left at -1 the two pads are a quadrature pair, one
 * full step to each of 10, 11, 01, 00 (position counts those). All coils
 * off holds the rotor where it is. A pattern off the table, or a jump of
 * more than the sequence's two steps, is counted in "glitches" and the
 * rotor stays put (a real rotor would stall or slip).
 *
 * The pads are sampled "settle-us" after the last change: a driver sets
 * its four pins one after another, and no rotor answers a few hundred
 * nanoseconds of the table's in-between patterns.
 *
 * The rotor travels 0 to "range" (0: no stop, either way), from "start" at
 * reset, and a move into the end stops stalls: position stays, "stalls"
 * counts it, as a motor driven against its mechanical stop loses steps.
 * "min-pin" and "max-pin" are limit switches on pads of the same
 * controller, closed (the level "limit-level", 0 by default, active low with the pull-up the
 * drivers enable) while the rotor is within "limit-margin" steps of the
 * end, read through the pad the way the guest reads a button.
 *
 * The pads go to the SoC's GPIO controller: the one linked as "gpio", or
 * the only one there is. A pad takes one device: two motors need eight
 * different pads. The position is read (and set, to place the rotor) on
 * the "position" property and traced as gpio_stepper_move.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/qapi-builtin-type-infos.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/gpio/ingenic-gpio.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_GPIO_STEPPER "gpio-stepper"
OBJECT_DECLARE_SIMPLE_TYPE(GpioStepperState, GPIO_STEPPER)

#define GS_COILS 4

struct GpioStepperState {
    DeviceState parent_obj;

    DeviceState *gpio;
    int32_t phase[GS_COILS];
    int32_t min_pin, max_pin;
    int32_t range, start, margin;
    uint32_t settle_us;
    uint32_t limit_level;

    unsigned nphases;
    uint32_t pads;          /* the level each coil's pad drives, bit 0 first */
    int32_t idx;            /* table index of the last pattern, -1 unknown */
    int32_t pos;
    uint64_t moved, stalls, glitches;
    bool min_on, max_on;
    QEMUTimer *timer;
    qemu_irq min_pull, max_pull;
};

/* Coil 0 is the table's high bit: 1000 is index 0 */
static const uint8_t gs_half[8] = { 0x8, 0xc, 0x4, 0x6, 0x2, 0x3, 0x1, 0x9 };
static const uint8_t gs_quad[4] = { 0x2, 0x3, 0x1, 0x0 };

static unsigned gs_len(const GpioStepperState *s)
{
    return s->nphases == 4 ? 8 : 4;
}

static int gs_lookup(const GpioStepperState *s, unsigned pattern)
{
    const uint8_t *tab = s->nphases == 4 ? gs_half : gs_quad;

    for (unsigned i = 0; i < gs_len(s); i++) {
        if (tab[i] == pattern) {
            return i;
        }
    }
    return -1;
}

static void gs_switches(GpioStepperState *s)
{
    bool lo = s->min_pin >= 0 && s->pos <= s->margin;
    bool hi = s->max_pin >= 0 && s->range > 0 &&
              s->pos >= s->range - s->margin;

    if (s->min_pull && lo != s->min_on) {
        qemu_set_irq(s->min_pull, lo ? s->limit_level : !s->limit_level);
    }
    if (s->max_pull && hi != s->max_on) {
        qemu_set_irq(s->max_pull, hi ? s->limit_level : !s->limit_level);
    }
    if (lo != s->min_on || hi != s->max_on) {
        trace_gpio_stepper_limit(DEVICE(s)->id ?: "", s->pos, lo, hi);
    }
    s->min_on = lo;
    s->max_on = hi;
}

static void gs_settle(void *opaque)
{
    GpioStepperState *s = opaque;
    unsigned n = gs_len(s), pattern = 0;
    int idx, step, target;

    /* coil 0 is the table's high bit */
    for (unsigned i = 0; i < s->nphases; i++) {
        pattern |= extract32(s->pads, i, 1) << (s->nphases - 1 - i);
    }
    idx = gs_lookup(s, pattern);

    if (idx < 0) {
        if (pattern != 0 && pattern != BIT(s->nphases) - 1) {
            s->glitches++;
        }
        return;
    }
    if (s->idx < 0) {
        s->idx = idx;
        trace_gpio_stepper_move(DEVICE(s)->id ?: "", s->pos, idx, 0);
        return;
    }
    step = (idx - s->idx + n) % n;
    if (step > n / 2) {
        step -= n;
    }
    if (step == n / 2 || step > (s->nphases == 4 ? 2 : 1) ||
        step < -(s->nphases == 4 ? 2 : 1)) {
        s->glitches++;
        return;
    }
    s->idx = idx;
    target = s->pos + step;
    if (s->range > 0 && (target < 0 || target > s->range)) {
        s->stalls++;
        trace_gpio_stepper_stall(DEVICE(s)->id ?: "", s->pos, step);
        return;
    }
    s->pos = target;
    s->moved += abs(step);
    trace_gpio_stepper_move(DEVICE(s)->id ?: "", s->pos, idx, step);
    gs_switches(s);
}

static void gs_pad(void *opaque, int line, int level)
{
    GpioStepperState *s = opaque;

    s->pads = deposit32(s->pads, line, 1, level);
    timer_mod(s->timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) +
                        s->settle_us);
}

static void gs_get_pos(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    int64_t pos = GPIO_STEPPER(obj)->pos;

    visit_type_int(v, name, &pos, errp);
}

static void gs_set_pos(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    GpioStepperState *s = GPIO_STEPPER(obj);
    int64_t pos;

    if (!visit_type_int(v, name, &pos, errp)) {
        return;
    }
    if (s->range > 0 && (pos < 0 || pos > s->range)) {
        error_setg(errp, "position out of range 0..%d", s->range);
        return;
    }
    s->pos = pos;
    gs_switches(s);
}

#define GS_COUNTER(fn, field)                                                \
    static void fn(Object *obj, Visitor *v, const char *name, void *opaque,  \
                   Error **errp)                                             \
    {                                                                        \
        uint64_t val = GPIO_STEPPER(obj)->field;                             \
                                                                             \
        visit_type_uint64(v, name, &val, errp);                              \
    }
GS_COUNTER(gs_get_moved, moved)
GS_COUNTER(gs_get_stalls, stalls)
GS_COUNTER(gs_get_glitches, glitches)

static void gs_reset_hold(Object *obj, ResetType type)
{
    GpioStepperState *s = GPIO_STEPPER(obj);

    timer_del(s->timer);
    s->pads = MAKE_64BIT_MASK(0, s->nphases);
    s->idx = -1;
    s->pos = s->start;
    s->moved = s->stalls = s->glitches = 0;
    s->min_on = s->max_on = false;
    if (s->min_pull) {
        qemu_set_irq(s->min_pull, !s->limit_level);
    }
    if (s->max_pull) {
        qemu_set_irq(s->max_pull, !s->limit_level);
    }
    gs_switches(s);
}

static void gs_realize(DeviceState *dev, Error **errp)
{
    GpioStepperState *s = GPIO_STEPPER(dev);
    bool ambiguous;
    uint32_t pins;

    if (!s->gpio) {
        Object *o = object_resolve_path_type("", TYPE_INGENIC_GPIO,
                                             &ambiguous);

        if (!o || ambiguous) {
            error_setg(errp, "no single Ingenic GPIO controller: "
                       "set the gpio link");
            return;
        }
        s->gpio = DEVICE(o);
    }
    if (!object_dynamic_cast(OBJECT(s->gpio), TYPE_INGENIC_GPIO)) {
        error_setg(errp, "gpio is not an %s", TYPE_INGENIC_GPIO);
        return;
    }
    pins = INGENIC_GPIO(s->gpio)->nr_ports * 32;

    if (s->phase[0] < 0 || s->phase[1] < 0) {
        error_setg(errp, "phase0 and phase1 are needed");
        return;
    }
    if ((s->phase[2] < 0) != (s->phase[3] < 0)) {
        error_setg(errp, "phase2 and phase3 go together: four coils or two");
        return;
    }
    s->nphases = s->phase[2] < 0 ? 2 : 4;
    for (int i = 0; i < s->nphases; i++) {
        if (s->phase[i] >= (int32_t)pins) {
            error_setg(errp, "phase%d is not a GPIO under %u", i, pins);
            return;
        }
        for (int j = 0; j < i; j++) {
            if (s->phase[i] == s->phase[j]) {
                error_setg(errp, "phase%d and phase%d are the same pad",
                           j, i);
                return;
            }
        }
    }
    if (s->range < 0 || s->start < 0 || s->margin < 0 ||
        (s->range > 0 && s->start > s->range)) {
        error_setg(errp, "start must be within 0..range, margin not "
                   "negative");
        return;
    }
    if (s->min_pin >= (int32_t)pins || s->max_pin >= (int32_t)pins) {
        error_setg(errp, "a limit pin is not a GPIO under %u", pins);
        return;
    }
    if (s->max_pin >= 0 && s->range == 0) {
        error_setg(errp, "max-pin needs a range");
        return;
    }
    if (s->limit_level > 1) {
        error_setg(errp, "limit-level is 0 or 1");
        return;
    }

    s->timer = timer_new_us(QEMU_CLOCK_VIRTUAL, gs_settle, s);
    qdev_init_gpio_in_named(dev, gs_pad, "pads", s->nphases);
    for (int i = 0; i < s->nphases; i++) {
        qdev_connect_gpio_out_named(s->gpio, "pad", s->phase[i],
                                    qdev_get_gpio_in_named(dev, "pads", i));
    }
    if (s->min_pin >= 0) {
        s->min_pull = qdev_get_gpio_in_named(s->gpio, "pull", s->min_pin);
    }
    if (s->max_pin >= 0) {
        s->max_pull = qdev_get_gpio_in_named(s->gpio, "pull", s->max_pin);
    }
}

static void gs_init(Object *obj)
{
    object_property_add(obj, "position", "int",
                        &int_type_info, gs_get_pos,
                        gs_set_pos, NULL, NULL);
    object_property_add(obj, "moved", "uint64",
                        &uint64_type_info, gs_get_moved,
                        NULL, NULL, NULL);
    object_property_add(obj, "stalls", "uint64",
                        &uint64_type_info, gs_get_stalls,
                        NULL, NULL, NULL);
    object_property_add(obj, "glitches", "uint64",
                        &uint64_type_info, gs_get_glitches,
                        NULL, NULL, NULL);
}

static const Property gs_props[] = {
    DEFINE_PROP_LINK("gpio", GpioStepperState, gpio, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_INT32("phase0", GpioStepperState, phase[0], -1),
    DEFINE_PROP_INT32("phase1", GpioStepperState, phase[1], -1),
    DEFINE_PROP_INT32("phase2", GpioStepperState, phase[2], -1),
    DEFINE_PROP_INT32("phase3", GpioStepperState, phase[3], -1),
    DEFINE_PROP_INT32("min-pin", GpioStepperState, min_pin, -1),
    DEFINE_PROP_INT32("max-pin", GpioStepperState, max_pin, -1),
    DEFINE_PROP_UINT32("limit-level", GpioStepperState, limit_level, 0),
    DEFINE_PROP_INT32("limit-margin", GpioStepperState, margin, 0),
    DEFINE_PROP_INT32("range", GpioStepperState, range, 0),
    DEFINE_PROP_INT32("start", GpioStepperState, start, 0),
    DEFINE_PROP_UINT32("settle-us", GpioStepperState, settle_us, 20),
};

static const VMStateDescription vmstate_gs = {
    .name = TYPE_GPIO_STEPPER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(pads, GpioStepperState),
        VMSTATE_INT32(idx, GpioStepperState),
        VMSTATE_INT32(pos, GpioStepperState),
        VMSTATE_UINT64(moved, GpioStepperState),
        VMSTATE_UINT64(stalls, GpioStepperState),
        VMSTATE_UINT64(glitches, GpioStepperState),
        VMSTATE_BOOL(min_on, GpioStepperState),
        VMSTATE_BOOL(max_on, GpioStepperState),
        VMSTATE_TIMER_PTR(timer, GpioStepperState),
        VMSTATE_END_OF_LIST()
    },
};

static void gs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Stepper motor on GPIO pads";
    dc->realize = gs_realize;
    dc->bus_type = TYPE_SYSTEM_BUS;
    dc->vmsd = &vmstate_gs;
    rc->phases.hold = gs_reset_hold;
    device_class_set_props(dc, gs_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo gs_info = {
    .name = TYPE_GPIO_STEPPER,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(GpioStepperState),
    .instance_init = gs_init,
    .class_init = gs_class_init,
};

static void gs_register_types(void)
{
    type_register_static(&gs_info);
}

type_init(gs_register_types)
