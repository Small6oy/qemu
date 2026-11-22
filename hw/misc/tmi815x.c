/*
 * TMI8152 and TMI8160 (the TMI8153) micro-stepping motor drivers (SPI)
 *
 * The TMI8152 shares its pinout, its SPI and its five channels with the
 * TMI8150B, whose datasheet (V0.3, 2022.02) says: four channels drive two
 * stepper motors (X and Y) in six modes, the default being the one where a
 * target number of steps and a phase are set (Function description, p.5),
 * with 64, 128, 256 or 512 micro steps (p.9); the fifth channel drives the
 * IR-cut coil (p.10: with IREN high, IRINA high and IRINB low is forwards,
 * the reverse backwards, both high brake, both low coast); CS is active
 * low, 5 MHz at most (SPI Logic Description, p.7). The register map and the
 * SPI frame are in figures the text does not carry, and the TMI8160, which
 * the T41's BOM calls the TMI8153, has no datasheet in the archive: the
 * registers below are the SDK drivers' (sample_motor4 for the TMI8152,
 * sample_motor5, which names the TMI8160 manual, for the other).
 *
 * The address byte with bit 7 set writes the next byte there, without it
 * the next byte read comes back; the bits go MSB first. Each channel has a
 * control register (bit 7 enables the channel, 0x07 and 0x1d are the manual
 * modes the driver sets), a mode register (bits 7:4 the step division, bits
 * 3:0 the output: TMI8152 0x5 clockwise, 0xa counter-clockwise, 0xf brake,
 * 0 open; TMI8160 0x1, 0x2, 0x3 and 0), clock dividers, a revolution counter
 * (16 bits, low then high) and the count to run to (16 bits). The global
 * register (0x02) enables the chip with bit 7. The IR-cut register (0x11,
 * 0x10 on the TMI8160) has IREN in bit 7, IRINA in bit 2 and IRINB in bit 3,
 * as the driver's 0x86 and 0x8a write them against the table above; its
 * state is the "ircut" property.
 *
 *            TMI8152                       TMI8160
 *   channel  1       2                      1       2
 *   control  0x04    0x14                   0x20    0x40
 *   mode     0x05    0x15                   0x22    0x42
 *   counter  0x08/09 0x18/19                0x2a/2b 0x4a/4b
 *   run to   0x0b/0c 0x1b/1c                0x2e/2f 0x4e/4f
 *   IR-cut   0x11                           0x10
 *
 * An enabled channel in a drive mode moves one step per "step-us" while the
 * chip is enabled, until the counter equals the count to run to, then
 * holds. The TMI8152's counter counts the steps made in either direction
 * and its driver writes the count as it is; the TMI8160's counts up
 * clockwise and down counter-clockwise, and its driver writes 65536 minus
 * the count for counter-clockwise: that is how the two are modelled, from
 * the drivers (the clock dividers and the step-us time per step are not
 * derived from them). Writing the counter restarts the count. The
 * "position-1" and "position-2" properties are the rotors' net steps,
 * clockwise positive.
 *
 * Wiring, from the T23 RD_T23_EEL V1.1 schematic (TMI8153) and the PIKE's
 * note that such a driver can replace the MS32006: CS, SCLK, SDI and SDO on
 * the SoC's SSI, OUT1A to OUT2B the horizontal motor, OUT3A to OUT4B the
 * vertical one, IROA and IROB the IR-cut coil, an EN pin.
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
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_TMI815X "tmi815x"
#define TYPE_TMI8152 "tmi8152"
#define TYPE_TMI8160 "tmi8160"
#define TYPE_TMI8153 "tmi8153"
OBJECT_DECLARE_TYPE(Tmi815xState, Tmi815xClass, TMI815X)

typedef struct Tmi815xLayout {
    uint8_t ctl, mode, cnt, set, stride, ircut;
    uint8_t cw, ccw;
    bool signed_count;
} Tmi815xLayout;

static const Tmi815xLayout lay8152 = {
    .ctl = 0x04, .mode = 0x05, .cnt = 0x08, .set = 0x0b, .stride = 0x10,
    .ircut = 0x11, .cw = 0x5, .ccw = 0xa,
};
static const Tmi815xLayout lay8153 = {
    .ctl = 0x20, .mode = 0x22, .cnt = 0x2a, .set = 0x2e, .stride = 0x20,
    .ircut = 0x10, .cw = 0x1, .ccw = 0x2, .signed_count = true,
};

struct Tmi815xClass {
    SSIPeripheralClass parent_class;
    const Tmi815xLayout *lay;
};

typedef struct Tmi815xChan {
    uint16_t count, target;
    int64_t pos;
    QEMUTimer *timer;
} Tmi815xChan;

struct Tmi815xState {
    SSIPeripheral parent_obj;

    uint32_t step_us;
    uint8_t reg[256];
    unsigned nbytes;
    uint8_t cmd;
    Tmi815xChan ch[2];
};

static int tmi_dir(const Tmi815xState *s, int c)
{
    const Tmi815xLayout *l = TMI815X_GET_CLASS(s)->lay;
    uint8_t mode = s->reg[l->mode + c * l->stride] & 0xf;

    return mode == l->cw ? 1 : mode == l->ccw ? -1 : 0;
}

static bool tmi_running(const Tmi815xState *s, int c)
{
    const Tmi815xLayout *l = TMI815X_GET_CLASS(s)->lay;

    return (s->reg[0x02] & 0x80) && (s->reg[l->ctl + c * l->stride] & 0x80) &&
           tmi_dir(s, c) && s->ch[c].count != s->ch[c].target;
}

static void tmi_kick(Tmi815xState *s, int c)
{
    if (tmi_running(s, c)) {
        if (!timer_pending(s->ch[c].timer)) {
            timer_mod(s->ch[c].timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->step_us * 1000LL);
        }
    } else {
        timer_del(s->ch[c].timer);
    }
}

static void tmi_tick(Tmi815xState *s, int c)
{
    const Tmi815xLayout *l = TMI815X_GET_CLASS(s)->lay;
    Tmi815xChan *ch = &s->ch[c];
    int dir = tmi_dir(s, c);

    if (!tmi_running(s, c)) {
        return;
    }
    ch->pos += dir;
    ch->count += (l->signed_count && dir < 0) ? -1 : 1;
    s->reg[l->cnt + c * l->stride] = ch->count;
    s->reg[l->cnt + c * l->stride + 1] = ch->count >> 8;
    trace_tmi815x_move(c + 1, ch->pos, ch->count, ch->target);
    tmi_kick(s, c);
}

static void tmi_tick0(void *opaque)
{
    tmi_tick(opaque, 0);
}

static void tmi_tick1(void *opaque)
{
    tmi_tick(opaque, 1);
}

static void tmi_sync(Tmi815xState *s)
{
    const Tmi815xLayout *l = TMI815X_GET_CLASS(s)->lay;

    for (int c = 0; c < 2; c++) {
        unsigned o = c * l->stride;

        s->ch[c].target = s->reg[l->set + o] | s->reg[l->set + o + 1] << 8;
        tmi_kick(s, c);
    }
}

static const char *tmi_ircut_state(uint8_t v)
{
    bool a = v & 4, b = v & 8;

    if (!(v & 0x80)) {
        return "off";
    }
    return a && b ? "brake" : a ? "forwards" : b ? "backwards" : "coast";
}

static void tmi_write(Tmi815xState *s, unsigned addr, uint8_t v)
{
    const Tmi815xLayout *l = TMI815X_GET_CLASS(s)->lay;

    s->reg[addr] = v;
    trace_tmi815x_write(addr, v);
    if (addr == l->ircut) {
        trace_tmi815x_ircut(tmi_ircut_state(v));
    }
    for (int c = 0; c < 2; c++) {
        unsigned o = l->cnt + c * l->stride;

        if (addr == o || addr == o + 1) {
            s->ch[c].count = s->reg[o] | s->reg[o + 1] << 8;
        }
    }
    if (addr == 0x02 && !(v & 0x80)) {
        for (int c = 0; c < 2; c++) {
            s->ch[c].count = 0;
            s->reg[l->cnt + c * l->stride] = 0;
            s->reg[l->cnt + c * l->stride + 1] = 0;
        }
    }
    tmi_sync(s);
}

static uint8_t tmi_read(Tmi815xState *s, unsigned addr)
{
    const Tmi815xLayout *l = TMI815X_GET_CLASS(s)->lay;

    for (int c = 0; c < 2; c++) {
        unsigned o = l->cnt + c * l->stride;

        if (addr == o || addr == o + 1) {
            return s->ch[c].count >> (8 * (addr - o));
        }
    }
    return s->reg[addr];
}

static int tmi_set_cs(SSIPeripheral *dev, bool select)
{
    if (select) {
        TMI815X(dev)->nbytes = 0;
    }
    return 0;
}

static uint32_t tmi_transfer(SSIPeripheral *dev, uint32_t val)
{
    Tmi815xState *s = TMI815X(dev);
    unsigned n = s->nbytes++;

    if (n == 0) {
        s->cmd = val;
        return 0;
    }
    if (s->cmd & 0x80) {
        if (n == 1) {
            tmi_write(s, s->cmd & 0x7f, val);
        }
        return 0;
    }
    return n == 1 ? tmi_read(s, s->cmd) : 0;
}

static void tmi_get_pos(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    int64_t pos = TMI815X(obj)->ch[GPOINTER_TO_INT(opaque)].pos;

    visit_type_int(v, name, &pos, errp);
}

static char *tmi_get_ircut(Object *obj, Error **errp)
{
    Tmi815xState *s = TMI815X(obj);

    return g_strdup(tmi_ircut_state(
                        s->reg[TMI815X_GET_CLASS(s)->lay->ircut]));
}

static void tmi_init(Object *obj)
{
    object_property_add_str(obj, "ircut", tmi_get_ircut, NULL);
    object_property_add(obj, "position-1", "int",
                        &int_type_info, tmi_get_pos,
                        NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(obj, "position-2", "int",
                        &int_type_info, tmi_get_pos,
                        NULL, NULL, GINT_TO_POINTER(1));
}

static void tmi_realize(SSIPeripheral *dev, Error **errp)
{
    Tmi815xState *s = TMI815X(dev);

    s->ch[0].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tmi_tick0, s);
    s->ch[1].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tmi_tick1, s);
}

static void tmi_reset_hold(Object *obj, ResetType type)
{
    Tmi815xState *s = TMI815X(obj);

    memset(s->reg, 0, sizeof(s->reg));
    s->nbytes = 0;
    for (int c = 0; c < 2; c++) {
        timer_del(s->ch[c].timer);
        s->ch[c].count = s->ch[c].target = 0;
        s->ch[c].pos = 0;
    }
}

static const Property tmi_props[] = {
    DEFINE_PROP_UINT32("step-us", Tmi815xState, step_us, 500),
};

static const VMStateDescription vmstate_tmi_chan = {
    .name = "tmi815x-chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(count, Tmi815xChan),
        VMSTATE_UINT16(target, Tmi815xChan),
        VMSTATE_INT64(pos, Tmi815xChan),
        VMSTATE_TIMER_PTR(timer, Tmi815xChan),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_tmi = {
    .name = TYPE_TMI815X,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(reg, Tmi815xState, 256),
        VMSTATE_UINT32(nbytes, Tmi815xState),
        VMSTATE_UINT8(cmd, Tmi815xState),
        VMSTATE_STRUCT_ARRAY(ch, Tmi815xState, 2, 1, vmstate_tmi_chan,
                             Tmi815xChan),
        VMSTATE_END_OF_LIST()
    },
};

static void tmi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    k->realize = tmi_realize;
    k->transfer = tmi_transfer;
    k->set_cs = tmi_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    rc->phases.hold = tmi_reset_hold;
    dc->vmsd = &vmstate_tmi;
    device_class_set_props(dc, tmi_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void tmi8152_class_init(ObjectClass *klass, const void *data)
{
    TMI815X_CLASS(klass)->lay = &lay8152;
    DEVICE_CLASS(klass)->desc = "TMI8152 motor driver";
}

static void tmi8160_class_init(ObjectClass *klass, const void *data)
{
    TMI815X_CLASS(klass)->lay = &lay8153;
    DEVICE_CLASS(klass)->desc = "TMI8160 motor driver";
}

static void tmi8153_class_init(ObjectClass *klass, const void *data)
{
    DEVICE_CLASS(klass)->desc = "TMI8153 (the TMI8160's older name)";
}

static const TypeInfo tmi_types[] = {
    {
        .name = TYPE_TMI815X,
        .parent = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(Tmi815xState),
        .instance_init = tmi_init,
        .class_size = sizeof(Tmi815xClass),
        .class_init = tmi_class_init,
        .abstract = true,
    }, {
        .name = TYPE_TMI8152,
        .parent = TYPE_TMI815X,
        .class_init = tmi8152_class_init,
    }, {
        .name = TYPE_TMI8160,
        .parent = TYPE_TMI815X,
        .class_init = tmi8160_class_init,
    }, {
        .name = TYPE_TMI8153,
        .parent = TYPE_TMI8160,
        .class_init = tmi8153_class_init,
    },
};

DEFINE_TYPES(tmi_types)
