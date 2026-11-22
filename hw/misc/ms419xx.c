/*
 * MS419xx lens motor driver (SPI): two micro-stepping motor drivers
 *
 * Follows RuiMeng's MS41929 datasheet (V1.2, 2015.08.10; the MS41939's
 * V-series sheet has the same 0x20 to 0x2c map): Serial Interface
 * Specifications (p.12) for the frame, the Register Map and List (pp.12 to
 * 14) and the Register detail description (pp.15 to 31) for the bits.
 *
 * The frame is 24 bits, sent LSB first with chip select high: the address
 * A5 to A0, C0 (1 reads) and C1, then D15 to D0 as low byte and high byte.
 * Registers (the reset values are the datasheet's "Initial Value"; bits it
 * calls reserved are not kept):
 *
 *   0x0b  D10 MODESEL_FZ (0: VD_FZ acts on its rising edge, 1: falling),
 *         D8 TESTEN1
 *   0x20  D14:13 PWMRES, D12:8 PWMMODE, D7:0 DT1, the start point wait
 *         (n * 8192 / OSCIN; 0 stops the other registers being updated);
 *         resets to 0x3c0a
 *   0x21  D8 TESTEN2, D4:0 FZTEST
 *   0x22  D13:8 PHMODAB, D7:0 DT2A, the start point excitation wait; 0x0003
 *   0x23  PPWB, PPWA (peak pulse width)
 *   0x24  D13:12 MICROAB (3: 64 divisions, 2: 128, 0 and 1: 256), D11 LEDB,
 *         D10 ENDISAB (1: outputs on), D9 BRAKEAB, D8 CCWCWAB (0 forward,
 *         1 reverse), D7:0 PSUMAB, the step count per VD
 *   0x25  INTCTAB, the step cycle (0: the position is held)
 *   0x27 to 0x2a  the same for the second motor (CD)
 *   0x2c  D2 SEL, D1 IN1, D0 IN2, the infrared rejector bridge
 *
 * Each active VD_FZ edge starts a move after DT1 and DT2 (both nonzero):
 * PSUM * 8 sine-wave 256ths of a cycle, 3 * INTCT / OSCIN each (24 * INTCT *
 * PSUM / OSCIN in all, as the datasheet's p.19 formula has it), whatever
 * the number of divisions; what has not been done when the next edge
 * comes is cancelled (p.17). The rotor moves only with ENDIS set and BRAKE
 * clear and INTCT not 0. The position of each motor is a property, in
 * 256ths of a cycle: 64 are a full step of a four-phase motor, the 8
 * PSUM units the vendor's sample_motor3 counts as one step. A motor runs
 * between 0 and "range" (0: no stops), and a move into a stop is lost.
 *
 * "vd-pin" is the VD_FZ input, a GPIO of the SoC's controller (the sample's
 * VDFZ, PB31); "reset-pin" holds the registers at their initial values while
 * at "reset-level". The datasheet's RSTB is active low, but the sample's
 * driver leaves its pin low after a high pulse and works, so the board must
 * invert it: the default is high.
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
#include "qemu/timer.h"
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
#define MS_INTCT(ch) ((ch) ? 0x2a : 0x25)
#define MS_DT2(ch)   ((ch) ? 0x27 : 0x22)

typedef struct Ms419xxMove {
    QEMUTimer *timer;       /* the move's end */
    QEMUTimer *wait;        /* the start point waits' end */
    int64_t start_ns;
    int64_t unit_ns;
    int32_t units;
    int32_t done;
    int8_t dir;
    bool active;
} Ms419xxMove;

struct Ms419xxState {
    SSIPeripheral parent_obj;

    DeviceState *gpio;
    int32_t vd_pin, reset_pin;
    int32_t range;
    uint32_t reset_level;
    uint32_t oscin;
    bool lsb_first;

    uint16_t reg[MS_REGS];
    unsigned nbytes;
    uint8_t cmd, lo;
    bool vd;
    int64_t pos[2];
    uint64_t stalls[2];
    Ms419xxMove mv[2];
};

/* The bits each register keeps, 0 for an address the chip has none at */
static uint16_t ms_mask(unsigned addr)
{
    switch (addr) {
    case 0x0b:
        return 0x0500;
    case 0x20:
        return 0x7fff;
    case 0x21:
        return 0x011f;
    case 0x22:
    case 0x27:
        return 0x3fff;
    case 0x23:
    case 0x28:
    case 0x25:
    case 0x2a:
        return 0xffff;
    case 0x24:
    case 0x29:
        return 0x3fff;
    case 0x2c:
        return 0x0007;
    default:
        return 0;
    }
}

static uint8_t ms_wire(const Ms419xxState *s, uint8_t b)
{
    return s->lsb_first ? revbit8(b) : b;
}

static void ms_apply(Ms419xxState *s, int ch, int n)
{
    Ms419xxMove *m = &s->mv[ch];
    int64_t target;

    if (n <= 0) {
        return;
    }
    target = s->pos[ch] + m->dir * n;
    m->done += n;
    if (s->range > 0 && (target < 0 || target > s->range)) {
        s->stalls[ch]++;
        trace_ms419xx_stall(ch, s->pos[ch]);
        return;
    }
    s->pos[ch] = target;
    trace_ms419xx_move(ch, target, m->dir < 0, n);
}

/* What the move has done by now (or all of it, when it is over) */
static void ms_progress(Ms419xxState *s, int ch, bool over)
{
    Ms419xxMove *m = &s->mv[ch];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int n = over ? m->units - m->done :
            MIN((now - m->start_ns) / m->unit_ns, m->units) - m->done;

    ms_apply(s, ch, n);
}

static void ms_begin(Ms419xxState *s, int ch)
{
    Ms419xxMove *m = &s->mv[ch];
    uint16_t v = s->reg[MS_DRIVE(ch)];
    unsigned psum = v & 0xff, intct = s->reg[MS_INTCT(ch)];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!psum || !intct || !(v & BIT(10)) || (v & BIT(9))) {
        m->active = false;
        return;
    }
    m->active = true;
    m->dir = v & BIT(8) ? -1 : 1;
    m->units = psum * 8;
    m->done = 0;
    m->start_ns = now;
    m->unit_ns = MAX(muldiv64(3ull * intct, NANOSECONDS_PER_SECOND,
                              s->oscin), 1);
    timer_mod(m->timer, now + m->unit_ns * m->units);
}

static void ms_move_done(Ms419xxState *s, int ch)
{
    if (s->mv[ch].active) {
        ms_progress(s, ch, true);
        s->mv[ch].active = false;
    }
}

static void ms_done0(void *opaque)
{
    ms_move_done(opaque, 0);
}

static void ms_done1(void *opaque)
{
    ms_move_done(opaque, 1);
}

/* The start point waits over: the registers are taken and the motor runs */
static void ms_start0(void *opaque)
{
    ms_begin(opaque, 0);
}

static void ms_start1(void *opaque)
{
    ms_begin(opaque, 1);
}

static void ms_vd_edge(Ms419xxState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned dt1 = s->reg[0x20] & 0xff;

    for (int ch = 0; ch < 2; ch++) {
        unsigned dt2 = s->reg[MS_DT2(ch)] & 0xff;
        Ms419xxMove *m = &s->mv[ch];

        if (m->active) {
            /* what has not run by this edge is cancelled */
            timer_del(m->timer);
            ms_progress(s, ch, false);
            m->active = false;
        }
        timer_del(m->wait);
        if (dt1 && dt2) {
            timer_mod(m->wait, now + muldiv64((uint64_t)(dt1 + dt2) * 8192,
                                              NANOSECONDS_PER_SECOND,
                                              s->oscin));
        }
    }
}

static void ms_reset_regs(Ms419xxState *s)
{
    memset(s->reg, 0, sizeof(s->reg));
    s->reg[0x20] = 0x3c0a;
    s->reg[0x22] = 0x0003;
    s->reg[0x27] = 0x0003;
    s->nbytes = 0;
    for (int ch = 0; ch < 2; ch++) {
        timer_del(s->mv[ch].timer);
        timer_del(s->mv[ch].wait);
        s->mv[ch].active = false;
    }
}

static void ms_vd(void *opaque, int line, int level)
{
    Ms419xxState *s = opaque;
    bool falling_edge = s->reg[0x0b] & BIT(10);

    if (!!level != s->vd && (falling_edge ? !level : level)) {
        ms_vd_edge(s);
    }
    s->vd = !!level;
}

static void ms_rst(void *opaque, int line, int level)
{
    Ms419xxState *s = opaque;

    if (level == s->reset_level) {
        ms_reset_regs(s);
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
        s->reg[addr] = (s->lo | b << 8) & ms_mask(addr);
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
        s->range < 0 || s->reset_level > 1 || !s->oscin) {
        error_setg(errp, "vd-pin and reset-pin are GPIOs under %u, range "
                   "is not negative, reset-level is 0 or 1, oscin is set",
                   pins);
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
    s->mv[0].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_done0, s);
    s->mv[1].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_done1, s);
    s->mv[0].wait = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_start0, s);
    s->mv[1].wait = timer_new_ns(QEMU_CLOCK_VIRTUAL, ms_start1, s);
    object_property_add(OBJECT(d), "position-ab", "int", &int_type_info,
                        ms_get_pos, NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(OBJECT(d), "position-cd", "int", &int_type_info,
                        ms_get_pos, NULL, NULL, GINT_TO_POINTER(1));
    object_property_add(OBJECT(d), "stalls-ab", "uint64", &uint64_type_info,
                        ms_get_stalls, NULL, NULL, GINT_TO_POINTER(0));
    object_property_add(OBJECT(d), "stalls-cd", "uint64", &uint64_type_info,
                        ms_get_stalls, NULL, NULL, GINT_TO_POINTER(1));
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
    DEFINE_PROP_UINT32("reset-level", Ms419xxState, reset_level, 1),
    DEFINE_PROP_UINT32("oscin", Ms419xxState, oscin, 27000000),
    DEFINE_PROP_INT32("range", Ms419xxState, range, 0),
    DEFINE_PROP_BOOL("lsb-first", Ms419xxState, lsb_first, true),
};

static const VMStateDescription vmstate_ms_move = {
    .name = "ms419xx-move",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, Ms419xxMove),
        VMSTATE_TIMER_PTR(wait, Ms419xxMove),
        VMSTATE_INT64(start_ns, Ms419xxMove),
        VMSTATE_INT64(unit_ns, Ms419xxMove),
        VMSTATE_INT32(units, Ms419xxMove),
        VMSTATE_INT32(done, Ms419xxMove),
        VMSTATE_INT8(dir, Ms419xxMove),
        VMSTATE_BOOL(active, Ms419xxMove),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_ms = {
    .name = TYPE_MS419XX,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(reg, Ms419xxState, MS_REGS),
        VMSTATE_UINT32(nbytes, Ms419xxState),
        VMSTATE_UINT8(cmd, Ms419xxState),
        VMSTATE_UINT8(lo, Ms419xxState),
        VMSTATE_BOOL(vd, Ms419xxState),
        VMSTATE_INT64_ARRAY(pos, Ms419xxState, 2),
        VMSTATE_UINT64_ARRAY(stalls, Ms419xxState, 2),
        VMSTATE_STRUCT_ARRAY(mv, Ms419xxState, 2, 1, vmstate_ms_move,
                             Ms419xxMove),
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
