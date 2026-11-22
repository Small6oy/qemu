/*
 * MS419xx lens motor driver (SPI), two PWM stepper channels
 *
 * What the vendor sample_motor3 driver uses of the chip: 16-bit registers
 * at 6-bit addresses, written as the address byte then the value low and
 * high byte, read as the address byte with bit 6 set then two bytes back,
 * chip select active high, bits LSB first on the wire. The registers it
 * writes, and no others are modelled (a write to any register is kept and
 * reads back):
 *
 *   0x20 0x22 0x27         set up once, kept
 *   0x23 0x28              PWM duty of channel AB and CD
 *   0x24 0x29              drive of AB and CD: bits 7:0 the micro steps a
 *                          VD period moves (PSUM; the driver uses 8), bit 8
 *                          the direction, bits 15:9 kept (the driver writes
 *                          0x3408 for its left or down, 0x3508 for its right
 *                          or up and 0x3000 stopped: bit 8 counts the
 *                          position up)
 *   0x25 0x2a              pulse period of AB and CD, kept
 *
 * The VD pin ("vd-pin", a GPIO of the SoC's controller, the driver's
 * VDFZ) latches the drive registers: on each falling edge each channel
 * moves PSUM micro steps in its direction. The driver counts one step per
 * VD, so a step is PSUM = 8 micro steps. The "reset-pin" clears the
 * registers while it is high (the driver pulses it high then leaves it low).
 * A channel runs between 0 and "range" micro steps (0: no stops); a move into
 * a stop is lost and counted in "stalls-ab" and "stalls-cd". The positions
 * are the "position-ab" and "position-cd" properties, in micro steps.
 *
 * Only the driver's use is known: the register meanings above are inferred
 * from it, no datasheet was available, and the chip's 0x00 to 0x0f lens
 * registers (iris, zoom, focus DC drive) are not modelled.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/qapi-builtin-type-infos.h"
#include "qapi/visitor.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/ingenic-gpio.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_MS419XX "ms419xx"
OBJECT_DECLARE_SIMPLE_TYPE(Ms419xxState, MS419XX)

#define MS_REGS     64
#define MS_DRIVE(ch) ((ch) ? 0x29 : 0x24)

struct Ms419xxState {
    SSIPeripheral parent_obj;

    DeviceState *gpio;
    int32_t vd_pin, reset_pin;
    int32_t range;
    bool lsb_first;

    uint16_t reg[MS_REGS];
    unsigned nbytes;
    uint8_t cmd, lo;
    bool vd;
    int64_t pos[2];
    uint64_t stalls[2];
};

static uint8_t ms_wire(const Ms419xxState *s, uint8_t b)
{
    return s->lsb_first ? revbit8(b) : b;
}

static void ms_vd_fall(Ms419xxState *s)
{
    for (int ch = 0; ch < 2; ch++) {
        uint16_t v = s->reg[MS_DRIVE(ch)];
        int psum = v & 0xff;
        int64_t target = s->pos[ch] + (v & BIT(8) ? psum : -psum);

        if (!psum) {
            continue;
        }
        if (s->range > 0 && (target < 0 || target > s->range)) {
            s->stalls[ch]++;
            trace_ms419xx_stall(ch, s->pos[ch]);
            continue;
        }
        s->pos[ch] = target;
        trace_ms419xx_move(ch, target, !(v & BIT(8)), psum);
    }
}

static void ms_reset_regs(Ms419xxState *s)
{
    memset(s->reg, 0, sizeof(s->reg));
    s->nbytes = 0;
}

static void ms_vd(void *opaque, int line, int level)
{
    Ms419xxState *s = opaque;

    if (s->vd && !level) {
        ms_vd_fall(s);
    }
    s->vd = level;
}

static void ms_rst(void *opaque, int line, int level)
{
    if (level) {
        ms_reset_regs(opaque);
    }
}

static int ms_set_cs(SSIPeripheral *dev, bool select)
{
    Ms419xxState *s = MS419XX(dev);

    if (select) {
        s->nbytes = 0;
    }
    return 0;
}

static uint32_t ms_transfer(SSIPeripheral *dev, uint32_t val)
{
    Ms419xxState *s = MS419XX(dev);
    uint8_t b = ms_wire(s, val), out = 0;
    unsigned n = s->nbytes++, addr = s->cmd & (MS_REGS - 1);

    if (n == 0) {
        s->cmd = b;
        return 0;
    }
    if (s->cmd & 0x40) {
        if (n <= 2) {
            out = s->reg[addr] >> (8 * (n - 1));
        }
        return ms_wire(s, out);
    }
    if (n == 1) {
        s->lo = b;
    } else if (n == 2) {
        s->reg[addr] = s->lo | b << 8;
        trace_ms419xx_write(addr, s->reg[addr]);
    }
    return 0;
}

static void ms_get_pos(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    int64_t pos = MS419XX(obj)->pos[GPOINTER_TO_INT(opaque)];

    visit_type_int(v, name, &pos, errp);
}

static void ms_get_stalls(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    uint64_t n = MS419XX(obj)->stalls[GPOINTER_TO_INT(opaque)];

    visit_type_uint64(v, name, &n, errp);
}

static void ms_realize(SSIPeripheral *dev, Error **errp)
{
    Ms419xxState *s = MS419XX(dev);
    DeviceState *d = DEVICE(dev);
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
    if (s->vd_pin >= (int32_t)pins || s->reset_pin >= (int32_t)pins ||
        s->range < 0) {
        error_setg(errp, "vd-pin and reset-pin are GPIOs under %u, range "
                   "is not negative", pins);
        return;
    }
    qdev_init_gpio_in_named(d, ms_vd, "vd", 1);
    qdev_init_gpio_in_named(d, ms_rst, "reset", 1);
    if (s->vd_pin >= 0) {
        qdev_connect_gpio_out_named(s->gpio, "pad", s->vd_pin,
                                    qdev_get_gpio_in_named(d, "vd", 0));
    }
    if (s->reset_pin >= 0) {
        qdev_connect_gpio_out_named(s->gpio, "pad", s->reset_pin,
                                    qdev_get_gpio_in_named(d, "reset", 0));
    }
    object_property_add(OBJECT(d), "position-ab", "int",
                        &int_type_info, ms_get_pos,
                        NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(OBJECT(d), "position-cd", "int",
                        &int_type_info, ms_get_pos,
                        NULL, NULL, GINT_TO_POINTER(1));
    object_property_add(OBJECT(d), "stalls-ab", "uint64",
                        &uint64_type_info, ms_get_stalls,
                        NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(OBJECT(d), "stalls-cd", "uint64",
                        &uint64_type_info, ms_get_stalls,
                        NULL, NULL, GINT_TO_POINTER(1));
}

static void ms_reset_hold(Object *obj, ResetType type)
{
    Ms419xxState *s = MS419XX(obj);

    ms_reset_regs(s);
    s->vd = true;
    s->pos[0] = s->pos[1] = 0;
    s->stalls[0] = s->stalls[1] = 0;
}

static const Property ms_props[] = {
    DEFINE_PROP_LINK("gpio", Ms419xxState, gpio, TYPE_DEVICE, DeviceState *),
    DEFINE_PROP_INT32("vd-pin", Ms419xxState, vd_pin, -1),
    DEFINE_PROP_INT32("reset-pin", Ms419xxState, reset_pin, -1),
    DEFINE_PROP_INT32("range", Ms419xxState, range, 0),
    DEFINE_PROP_BOOL("lsb-first", Ms419xxState, lsb_first, true),
};

static const VMStateDescription vmstate_ms = {
    .name = TYPE_MS419XX,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(reg, Ms419xxState, MS_REGS),
        VMSTATE_UINT32(nbytes, Ms419xxState),
        VMSTATE_UINT8(cmd, Ms419xxState),
        VMSTATE_UINT8(lo, Ms419xxState),
        VMSTATE_BOOL(vd, Ms419xxState),
        VMSTATE_INT64_ARRAY(pos, Ms419xxState, 2),
        VMSTATE_UINT64_ARRAY(stalls, Ms419xxState, 2),
        VMSTATE_END_OF_LIST()
    },
};

static void ms_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    k->realize = ms_realize;
    k->transfer = ms_transfer;
    k->set_cs = ms_set_cs;
    k->cs_polarity = SSI_CS_HIGH;
    rc->phases.hold = ms_reset_hold;
    dc->desc = "MS419xx lens motor driver";
    dc->vmsd = &vmstate_ms;
    device_class_set_props(dc, ms_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo ms_info = {
    .name = TYPE_MS419XX,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(Ms419xxState),
    .class_init = ms_class_init,
};

static void ms_register_types(void)
{
    type_register_static(&ms_info);
}

type_init(ms_register_types)
