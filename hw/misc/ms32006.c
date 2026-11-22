/*
 * MS32006 and MS32007 two-channel stepper motor driver (I2C)
 *
 * Eight-bit registers behind a pointer that steps on each byte, as the
 * vendor sample_motor_i2c driver uses the chip at 7-bit address 0x10:
 *
 *   0x00       bit 0 run (0 resets the chip), bit 2 pause A, bit 1 pause B,
 *              bit 7 four-phase five-wire motor (0: two-phase four-wire)
 *   0x01 0x02  channel A speed: 14 bits (0x01 low, 0x02 bits 5:0), bit 7 of
 *              0x02 the half-step mode
 *   0x03 0x04  channel A steps: 12 bits (0x03 low, 0x04 bits 3:0), bit 6
 *              counter-clockwise, bit 7 output enable
 *   0x05-0x08  the same for channel B
 *   0x09       bit 7 starts A, bit 6 starts B, bits 3:2 the IR-cut bridge
 *   0x0b-0x0e  state: the steps A and B still have to make, low then high
 *   0x0f       clock select (the MS32007 takes the internal clock)
 *
 * A started channel makes its steps one by one at "fclk / (speed * 128)" a
 * second (fclk is "fclk" Hz, 24 MHz as the T23 gives it on a pad, the
 * driver's ms32006_fclk), then stops; clearing the output enable bit in the
 * steps register stops it at once, a pause bit holds it. Positions are the
 * "position-a" and "position-b" properties, clockwise positive. A channel
 * runs between 0 and "range" steps (0: no stop) and a step into a stop is lost.
 *
 * Only the driver's use is known; register meanings are inferred from it, no
 * datasheet was available, and the state registers' layout is an assumption.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/qapi-builtin-type-infos.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_MS32006 "ms32006"
OBJECT_DECLARE_SIMPLE_TYPE(Ms32006State, MS32006)

typedef struct Ms32006Chan {
    uint16_t left;
    bool run;
    int64_t pos;
    uint64_t stalls;
    QEMUTimer *timer;
} Ms32006Chan;

struct Ms32006State {
    I2CSlave parent_obj;

    uint32_t fclk;
    int32_t range;
    uint8_t reg[16];
    uint8_t pointer;
    bool first;
    Ms32006Chan ch[2];
};

static unsigned ms_base(int c)
{
    return c ? 5 : 1;
}

static bool ms_active(const Ms32006State *s, int c)
{
    return s->ch[c].run && s->ch[c].left && (s->reg[0] & 1) &&
           !(s->reg[0] & (c ? 2 : 4));
}

static void ms_arm(Ms32006State *s, int c)
{
    unsigned b = ms_base(c);
    unsigned speed = s->reg[b] | (s->reg[b + 1] & 0x3f) << 8;
    int64_t period;

    if (!ms_active(s, c)) {
        timer_del(s->ch[c].timer);
        return;
    }
    period = (int64_t)(MAX(speed, 1)) * 128 * NANOSECONDS_PER_SECOND /
             MAX(s->fclk, 1);
    timer_mod(s->ch[c].timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period);
}

static void ms_tick(Ms32006State *s, int c)
{
    unsigned b = ms_base(c);
    bool ccw = s->reg[b + 3] & 0x40;
    Ms32006Chan *ch = &s->ch[c];
    int64_t target = ch->pos + (ccw ? -1 : 1);

    if (!ms_active(s, c)) {
        return;
    }
    if (s->range > 0 && (target < 0 || target > s->range)) {
        ch->stalls++;
        trace_ms32006_stall(c, ch->pos);
    } else {
        ch->pos = target;
        trace_ms32006_move(c, target, ch->left - 1);
    }
    if (--ch->left == 0) {
        ch->run = false;
    }
    ms_arm(s, c);
}

static void ms_tick_a(void *opaque)
{
    ms_tick(opaque, 0);
}

static void ms_tick_b(void *opaque)
{
    ms_tick(opaque, 1);
}

static void ms_write(Ms32006State *s, unsigned addr, uint8_t v)
{
    s->reg[addr & 15] = v;
    trace_ms32006_write(addr & 15, v);
    for (int c = 0; c < 2; c++) {
        unsigned b = ms_base(c);

        if (addr == b + 3 && !(v & 0x80)) {
            s->ch[c].run = false;
        }
    }
    if (addr == 9) {
        for (int c = 0; c < 2; c++) {
            unsigned b = ms_base(c);

            if (v & (c ? 0x40 : 0x80)) {
                s->ch[c].left = s->reg[b + 2] | (s->reg[b + 3] & 15) << 8;
                s->ch[c].run = (s->reg[b + 3] & 0x80) && s->ch[c].left;
            }
        }
    }
    if (addr == 0 && !(v & 1)) {
        for (int c = 0; c < 2; c++) {
            s->ch[c].run = false;
            s->ch[c].left = 0;
        }
    }
    for (int c = 0; c < 2; c++) {
        ms_arm(s, c);
    }
}

static uint8_t ms_read(Ms32006State *s, unsigned addr)
{
    switch (addr & 15) {
    case 0x0b:
        return s->ch[0].left & 0xff;
    case 0x0c:
        return s->ch[0].left >> 8;
    case 0x0d:
        return s->ch[1].left & 0xff;
    case 0x0e:
        return s->ch[1].left >> 8;
    default:
        return s->reg[addr & 15];
    }
}

static int ms_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        MS32006(i2c)->first = true;
    }
    return 0;
}

static uint8_t ms_recv(I2CSlave *i2c)
{
    Ms32006State *s = MS32006(i2c);

    return ms_read(s, s->pointer++);
}

static int ms_send(I2CSlave *i2c, uint8_t data)
{
    Ms32006State *s = MS32006(i2c);

    if (s->first) {
        s->pointer = data;
        s->first = false;
    } else {
        ms_write(s, s->pointer++, data);
    }
    return 0;
}

static void ms_get_pos(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    int64_t pos = MS32006(obj)->ch[GPOINTER_TO_INT(opaque)].pos;

    visit_type_int(v, name, &pos, errp);
}

static void ms_get_stalls(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    uint64_t n = MS32006(obj)->ch[GPOINTER_TO_INT(opaque)].stalls;

    visit_type_uint64(v, name, &n, errp);
}

static void ms_init(Object *obj)
{
    object_property_add(obj, "position-a", "int",
                        &int_type_info, ms_get_pos,
                        NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(obj, "position-b", "int",
                        &int_type_info, ms_get_pos,
                        NULL, NULL, GINT_TO_POINTER(1));
    object_property_add(obj, "stalls-a", "uint64",
                        &uint64_type_info, ms_get_stalls,
                        NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(obj, "stalls-b", "uint64",
                        &uint64_type_info, ms_get_stalls,
                        NULL, NULL, GINT_TO_POINTER(1));
}

static void ms_realize(DeviceState *dev, Error **errp)
{
    Ms32006State *s = MS32006(dev);

    s->ch[0].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_tick_a, s);
    s->ch[1].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_tick_b, s);
}

static void ms_reset_hold(Object *obj, ResetType type)
{
    Ms32006State *s = MS32006(obj);

    memset(s->reg, 0, sizeof(s->reg));
    s->pointer = 0;
    s->first = true;
    for (int c = 0; c < 2; c++) {
        timer_del(s->ch[c].timer);
        s->ch[c].left = 0;
        s->ch[c].run = false;
        s->ch[c].pos = 0;
        s->ch[c].stalls = 0;
    }
}

static const Property ms_props[] = {
    DEFINE_PROP_UINT32("fclk", Ms32006State, fclk, 24000000),
    DEFINE_PROP_INT32("range", Ms32006State, range, 0),
};

static const VMStateDescription vmstate_ms_chan = {
    .name = "ms32006-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(left, Ms32006Chan),
        VMSTATE_BOOL(run, Ms32006Chan),
        VMSTATE_INT64(pos, Ms32006Chan),
        VMSTATE_UINT64(stalls, Ms32006Chan),
        VMSTATE_TIMER_PTR(timer, Ms32006Chan),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_ms = {
    .name = TYPE_MS32006,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Ms32006State),
        VMSTATE_UINT8_ARRAY(reg, Ms32006State, 16),
        VMSTATE_UINT8(pointer, Ms32006State),
        VMSTATE_BOOL(first, Ms32006State),
        VMSTATE_STRUCT_ARRAY(ch, Ms32006State, 2, 1, vmstate_ms_chan,
                             Ms32006Chan),
        VMSTATE_END_OF_LIST()
    },
};

static void ms_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ms_realize;
    dc->vmsd = &vmstate_ms;
    dc->desc = "MS32006 stepper motor driver";
    device_class_set_props(dc, ms_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    rc->phases.hold = ms_reset_hold;
    k->event = ms_event;
    k->recv = ms_recv;
    k->send = ms_send;
}

static const TypeInfo ms_info = {
    .name = TYPE_MS32006,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ms32006State),
    .instance_init = ms_init,
    .class_init = ms_class_init,
};

static void ms_register_types(void)
{
    type_register_static(&ms_info);
}

type_init(ms_register_types)
