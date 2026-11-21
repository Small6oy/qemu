/*
 * NXP PCF8563 I2C real-time clock/calendar
 *
 * Follows the PCF8563 data sheet (Rev. 11.1): sections 8.2 (registers),
 * 8.3.2 (flags and INT), 8.4 (time and date), 8.5 (setting and reading
 * the time), 8.6 (alarm), 8.8 (timer), 8.10 (STOP) and 8.11 (reset). The
 * HYM8563 uses the same registers at the same address (0x51).
 *
 * The time is the host clock (or the -rtc clock) plus an offset that a
 * write of the time registers sets, like hw/rtc/ds1338.c. The registers
 * keep their values over a machine reset, as the part is not reset by the
 * SoC (Table 27); "vl" gives the VL flag it starts with.
 *
 * Modelled: BCD time with the century flag, the alarm (checked as the
 * minute increments, so it fires once per match, never for a time set to
 * match), the countdown timer in its four clocks with the INT pulse
 * widths of Table 7, STOP, and the INT pin as an active-low output
 * ("int") that "int-pin" wires to a pad of the SoC's GPIO controller.
 * Not modelled: CLKOUT (the FE and FD bits read back), EXT_CLK test mode,
 * the POR override, the half second after STOP is released.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/gpio/ingenic-gpio.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/bcd.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/rtc.h"
#include "system/system.h"

#define TYPE_PCF8563 "pcf8563"
OBJECT_DECLARE_SIMPLE_TYPE(PCF8563State, PCF8563)

#define R_CS1       0x00
#define R_CS2       0x01
#define R_SEC       0x02
#define R_MIN       0x03
#define R_HOUR      0x04
#define R_DAY       0x05
#define R_WDAY      0x06
#define R_MON       0x07
#define R_YEAR      0x08
#define R_AL_MIN    0x09
#define R_AL_HOUR   0x0a
#define R_AL_DAY    0x0b
#define R_AL_WDAY   0x0c
#define R_CLKOUT    0x0d
#define R_TCTRL     0x0e
#define R_TIMER     0x0f

#define CS1_STOP    0x20
#define CS1_MASK    0xa8        /* TEST1, STOP, TESTC */
#define CS2_TI_TP   0x10
#define CS2_AF      0x08
#define CS2_TF      0x04
#define CS2_AIE     0x02
#define CS2_TIE     0x01
#define SEC_VL      0x80
#define MON_C       0x80
#define AE          0x80
#define TCTRL_TE    0x80

struct PCF8563State {
    I2CSlave parent_obj;

    int64_t offset;
    struct tm frozen;           /* the time while STOP is set */
    uint8_t cs1, cs2;
    uint8_t alarm[4];
    uint8_t clkout, tctrl, tval;
    uint8_t wday_off;
    bool vl;
    bool vl_init;
    bool pulse;
    uint8_t ptr;
    bool addr_byte;
    bool dirty;
    uint8_t buf[7];             /* R_SEC to R_YEAR as the transfer sees them */
    int64_t cd_start;           /* virtual ns the countdown (re)started */
    uint8_t cd_n;               /* the count it started from */

    QEMUTimer *minute_timer;
    QEMUTimer *cd_timer;
    QEMUTimer *pulse_timer;
    qemu_irq int_out;

    int32_t int_pin;
    DeviceState *gpio;
    Notifier done;
};

/* The day of the week of a date, 0 for Sunday */
static int weekday_of(int year, int mon, int day)
{
    static const int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

    if (mon < 3) {
        year--;
    }
    return (year + year / 4 - year / 100 + year / 400 + t[mon - 1] + day) % 7;
}

/*
 * The offset that makes the clock read tm now. qemu_timedate_diff()
 * measures against the host clock, qemu_get_timedate() against rtc_clock:
 * they differ under -rtc clock=vm, so take both from the same instant.
 */
static int64_t offset_for(const struct tm *tm)
{
    struct tm now, want = *tm;

    qemu_get_timedate(&now, 0);
    return qemu_timedate_diff(&want) - qemu_timedate_diff(&now);
}

static void now_tm(PCF8563State *s, struct tm *tm, int64_t extra)
{
    if (s->cs1 & CS1_STOP) {
        *tm = s->frozen;
    } else {
        qemu_get_timedate(tm, s->offset + extra);
    }
}

static unsigned wday_of_tm(PCF8563State *s, const struct tm *tm)
{
    return (weekday_of(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday) +
            s->wday_off) % 7;
}

static void snapshot(PCF8563State *s)
{
    struct tm tm;
    int year;

    now_tm(s, &tm, 0);
    year = tm.tm_year + 1900;
    s->buf[0] = to_bcd(tm.tm_sec) | (s->vl ? SEC_VL : 0);
    s->buf[1] = to_bcd(tm.tm_min);
    s->buf[2] = to_bcd(tm.tm_hour);
    s->buf[3] = to_bcd(tm.tm_mday);
    s->buf[4] = wday_of_tm(s, &tm);
    /* C is set for the 2000s: the century flips as the years overflow */
    s->buf[5] = to_bcd(tm.tm_mon + 1) | ((year / 100) % 2 == 0 ? MON_C : 0);
    s->buf[6] = to_bcd(year % 100);
}

static int get_bcd(uint8_t v, uint8_t mask, int lo, int hi)
{
    int n = from_bcd(v & mask);

    return n < lo ? lo : n > hi ? hi : n;
}

static void match_arm(PCF8563State *s);

static void commit(PCF8563State *s)
{
    struct tm tm = { 0 };
    int year;

    if (!s->dirty) {
        return;
    }
    s->dirty = false;
    tm.tm_sec = get_bcd(s->buf[0], 0x7f, 0, 59);
    tm.tm_min = get_bcd(s->buf[1], 0x7f, 0, 59);
    tm.tm_hour = get_bcd(s->buf[2], 0x3f, 0, 23);
    tm.tm_mday = get_bcd(s->buf[3], 0x3f, 1, 31);
    tm.tm_mon = get_bcd(s->buf[5], 0x1f, 1, 12) - 1;
    year = get_bcd(s->buf[6], 0xff, 0, 99) +
           (s->buf[5] & MON_C ? 2000 : 1900);
    tm.tm_year = year - 1900;
    s->wday_off = ((s->buf[4] & 7) % 7 + 7 -
                   weekday_of(year, tm.tm_mon + 1, tm.tm_mday)) % 7;
    s->vl = s->buf[0] & SEC_VL;
    if (s->cs1 & CS1_STOP) {
        s->frozen = tm;
    } else {
        s->offset = offset_for(&tm);
    }
    match_arm(s);
}

/* INT is open-drain: low while active, released otherwise */
static void update_int(PCF8563State *s)
{
    bool active = ((s->cs2 & CS2_AIE) && (s->cs2 & CS2_AF)) ||
        ((s->cs2 & CS2_TIE) &&
         ((s->cs2 & CS2_TI_TP) ? s->pulse : (s->cs2 & CS2_TF)));

    qemu_set_irq(s->int_out, !active);
}

static bool alarm_match(PCF8563State *s, const struct tm *tm)
{
    bool any = false;

    if (!(s->alarm[0] & AE)) {
        any = true;
        if (from_bcd(s->alarm[0] & 0x7f) != tm->tm_min) {
            return false;
        }
    }
    if (!(s->alarm[1] & AE)) {
        any = true;
        if (from_bcd(s->alarm[1] & 0x3f) != tm->tm_hour) {
            return false;
        }
    }
    if (!(s->alarm[2] & AE)) {
        any = true;
        if (from_bcd(s->alarm[2] & 0x3f) != tm->tm_mday) {
            return false;
        }
    }
    if (!(s->alarm[3] & AE)) {
        any = true;
        if ((s->alarm[3] & 7) != wday_of_tm(s, tm)) {
            return false;
        }
    }
    return any;
}

/* The alarm is only compared as the time increments, minute by minute */
static void match_arm(PCF8563State *s)
{
    struct tm tm;
    int64_t now;

    if ((s->cs1 & CS1_STOP) ||
        ((s->alarm[0] & s->alarm[1] & s->alarm[2] & s->alarm[3]) & AE)) {
        timer_del(s->minute_timer);
        return;
    }
    now = qemu_clock_get_ns(rtc_clock);
    now_tm(s, &tm, 0);
    timer_mod(s->minute_timer,
              now + (60 - tm.tm_sec) * NANOSECONDS_PER_SECOND -
              now % NANOSECONDS_PER_SECOND);
}

static void minute_tick(void *opaque)
{
    PCF8563State *s = opaque;
    struct tm cur, prev;

    now_tm(s, &cur, 0);
    now_tm(s, &prev, -60);
    if (alarm_match(s, &cur) && !alarm_match(s, &prev)) {
        s->cs2 |= CS2_AF;
        update_int(s);
    }
    match_arm(s);
}

static const uint32_t td_hz_x60[4] = { 4096 * 60, 64 * 60, 60, 1 };

static int64_t cd_ticks_ns(PCF8563State *s, int64_t ticks)
{
    return muldiv64(ticks * 60, NANOSECONDS_PER_SECOND,
                    td_hz_x60[s->tctrl & 3]);
}

static void cd_start(PCF8563State *s)
{
    s->cd_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->cd_n = s->tval;
    if ((s->tctrl & TCTRL_TE) && s->cd_n) {
        timer_mod(s->cd_timer, s->cd_start + cd_ticks_ns(s, s->cd_n));
    } else {
        timer_del(s->cd_timer);
    }
}

static unsigned cd_remaining(PCF8563State *s)
{
    int64_t el = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->cd_start;
    int64_t ticks = muldiv64(el, td_hz_x60[s->tctrl & 3],
                             NANOSECONDS_PER_SECOND) / 60;

    return ticks >= s->cd_n ? 0 : s->cd_n - ticks;
}

static void pulse_end(void *opaque)
{
    PCF8563State *s = opaque;

    s->pulse = false;
    update_int(s);
}

static void cd_expire(void *opaque)
{
    PCF8563State *s = opaque;
    /* Table 7: INT pulse width in microseconds x 1000 per source clock */
    static const int64_t n1[4] = { 122070, 7812500, 15625000, 15625000 };
    static const int64_t ngt1[4] = { 244140, 15625000, 15625000, 15625000 };
    int64_t w = s->cd_n == 1 ? n1[s->tctrl & 3] : ngt1[s->tctrl & 3];

    s->cs2 |= CS2_TF;
    if (s->cs2 & CS2_TI_TP) {
        s->pulse = true;
        timer_mod(s->pulse_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + w);
    }
    update_int(s);
    s->cd_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->cd_timer, s->cd_start + cd_ticks_ns(s, s->cd_n));
}

static uint8_t reg_read(PCF8563State *s, uint8_t r)
{
    switch (r) {
    case R_CS1:
        return s->cs1;
    case R_CS2:
        return s->cs2;
    case R_SEC ... R_YEAR:
        return s->buf[r - R_SEC];
    case R_AL_MIN ... R_AL_WDAY:
        return s->alarm[r - R_AL_MIN];
    case R_CLKOUT:
        return s->clkout;
    case R_TCTRL:
        return s->tctrl;
    default:
        return (s->tctrl & TCTRL_TE) ? cd_remaining(s) : s->tval;
    }
}

static void reg_write(PCF8563State *s, uint8_t r, uint8_t v)
{
    switch (r) {
    case R_CS1:
        if ((v & CS1_STOP) && !(s->cs1 & CS1_STOP)) {
            now_tm(s, &s->frozen, 0);
            s->cs1 = v & CS1_MASK;
            match_arm(s);
        } else if (!(v & CS1_STOP) && (s->cs1 & CS1_STOP)) {
            s->cs1 = v & CS1_MASK;
            s->offset = offset_for(&s->frozen);
            match_arm(s);
        } else {
            s->cs1 = v & CS1_MASK;
        }
        break;
    case R_CS2:
        /* AF and TF: a written 0 clears, a written 1 leaves */
        s->cs2 = (v & (CS2_TI_TP | CS2_AIE | CS2_TIE)) |
                 (s->cs2 & v & (CS2_AF | CS2_TF));
        update_int(s);
        break;
    case R_SEC ... R_YEAR:
        s->buf[r - R_SEC] = v;
        s->dirty = true;
        break;
    case R_AL_MIN:
        s->alarm[0] = v;
        match_arm(s);
        break;
    case R_AL_HOUR:
        s->alarm[1] = v & 0xbf;
        match_arm(s);
        break;
    case R_AL_DAY:
        s->alarm[2] = v & 0xbf;
        match_arm(s);
        break;
    case R_AL_WDAY:
        s->alarm[3] = v & 0x87;
        match_arm(s);
        break;
    case R_CLKOUT:
        s->clkout = v & 0x83;
        break;
    case R_TCTRL:
        s->tctrl = v & 0x83;
        cd_start(s);
        break;
    default:
        s->tval = v;
        if (s->tctrl & TCTRL_TE) {
            cd_start(s);
        }
        break;
    }
}

static int pcf8563_event(I2CSlave *i2c, enum i2c_event event)
{
    PCF8563State *s = PCF8563(i2c);

    switch (event) {
    case I2C_START_RECV:
        /* The counters are read in one go from a copy taken here */
        commit(s);
        snapshot(s);
        break;
    case I2C_START_SEND:
        s->addr_byte = true;
        snapshot(s);
        break;
    case I2C_FINISH:
        commit(s);
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t pcf8563_recv(I2CSlave *i2c)
{
    PCF8563State *s = PCF8563(i2c);
    uint8_t r = reg_read(s, s->ptr);

    s->ptr = (s->ptr + 1) & 0xf;
    return r;
}

static int pcf8563_send(I2CSlave *i2c, uint8_t data)
{
    PCF8563State *s = PCF8563(i2c);

    if (s->addr_byte) {
        s->ptr = data & 0xf;
        s->addr_byte = false;
    } else {
        reg_write(s, s->ptr, data);
        s->ptr = (s->ptr + 1) & 0xf;
    }
    return 0;
}

static void pcf8563_done(Notifier *n, void *data)
{
    PCF8563State *s = container_of(n, PCF8563State, done);

    if (!s->gpio) {
        bool ambiguous;
        Object *o = object_resolve_path_type("", TYPE_INGENIC_GPIO,
                                             &ambiguous);

        s->gpio = o && !ambiguous ? DEVICE(o) : NULL;
    }
    if (s->gpio && s->int_pin >= 0 &&
        s->int_pin < INGENIC_GPIO(s->gpio)->nr_ports * 32) {
        qdev_connect_gpio_out_named(DEVICE(s), "int", 0,
                                    qdev_get_gpio_in_named(s->gpio, "pull",
                                                           s->int_pin));
        update_int(s);
    }
}

static void pcf8563_reset(DeviceState *dev)
{
    PCF8563State *s = PCF8563(dev);

    s->ptr = 0;
    s->addr_byte = false;
    s->dirty = false;
    update_int(s);
}

static void pcf8563_realize(DeviceState *dev, Error **errp)
{
    PCF8563State *s = PCF8563(dev);

    if (!I2C_SLAVE(dev)->address) {
        I2C_SLAVE(dev)->address = 0x51;
    }
    s->minute_timer = timer_new_ns(rtc_clock, minute_tick, s);
    s->cd_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, cd_expire, s);
    s->pulse_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pulse_end, s);
    qdev_init_gpio_out_named(dev, &s->int_out, "int", 1);

    /* Power-on values, Table 27; the time starts as the host's */
    s->cs1 = 0x08;
    s->alarm[0] = s->alarm[1] = s->alarm[2] = s->alarm[3] = AE;
    s->clkout = 0x80;
    s->tctrl = 0x03;
    s->vl = s->vl_init;
    s->done.notify = pcf8563_done;
    if (phase_check(PHASE_MACHINE_READY)) {
        pcf8563_done(&s->done, NULL);
    } else {
        qemu_add_machine_init_done_notifier(&s->done);
    }
}

static int pcf8563_post_load(void *opaque, int version_id)
{
    PCF8563State *s = opaque;

    match_arm(s);
    if ((s->tctrl & TCTRL_TE) && s->cd_n) {
        timer_mod(s->cd_timer, s->cd_start + cd_ticks_ns(s, s->cd_n));
    }
    update_int(s);
    return 0;
}

static const VMStateDescription pcf8563_vmstate = {
    .name = "pcf8563",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = pcf8563_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, PCF8563State),
        VMSTATE_INT64(offset, PCF8563State),
        VMSTATE_INT32(frozen.tm_sec, PCF8563State),
        VMSTATE_INT32(frozen.tm_min, PCF8563State),
        VMSTATE_INT32(frozen.tm_hour, PCF8563State),
        VMSTATE_INT32(frozen.tm_mday, PCF8563State),
        VMSTATE_INT32(frozen.tm_mon, PCF8563State),
        VMSTATE_INT32(frozen.tm_year, PCF8563State),
        VMSTATE_UINT8(cs1, PCF8563State),
        VMSTATE_UINT8(cs2, PCF8563State),
        VMSTATE_UINT8_ARRAY(alarm, PCF8563State, 4),
        VMSTATE_UINT8(clkout, PCF8563State),
        VMSTATE_UINT8(tctrl, PCF8563State),
        VMSTATE_UINT8(tval, PCF8563State),
        VMSTATE_UINT8(wday_off, PCF8563State),
        VMSTATE_BOOL(vl, PCF8563State),
        VMSTATE_BOOL(pulse, PCF8563State),
        VMSTATE_UINT8(ptr, PCF8563State),
        VMSTATE_BOOL(addr_byte, PCF8563State),
        VMSTATE_BOOL(dirty, PCF8563State),
        VMSTATE_UINT8_ARRAY(buf, PCF8563State, 7),
        VMSTATE_INT64(cd_start, PCF8563State),
        VMSTATE_UINT8(cd_n, PCF8563State),
        VMSTATE_END_OF_LIST()
    }
};

static const Property pcf8563_props[] = {
    DEFINE_PROP_BOOL("vl", PCF8563State, vl_init, false),
    DEFINE_PROP_INT32("int-pin", PCF8563State, int_pin, -1),
    DEFINE_PROP_LINK("gpio", PCF8563State, gpio, TYPE_DEVICE, DeviceState *),
};

static void pcf8563_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = pcf8563_realize;
    dc->vmsd = &pcf8563_vmstate;
    device_class_set_legacy_reset(dc, pcf8563_reset);
    device_class_set_props(dc, pcf8563_props);
    k->event = pcf8563_event;
    k->recv = pcf8563_recv;
    k->send = pcf8563_send;
}

static const TypeInfo pcf8563_info = {
    .name = TYPE_PCF8563,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(PCF8563State),
    .class_init = pcf8563_class_init,
};

static void pcf8563_register_types(void)
{
    type_register_static(&pcf8563_info);
}

type_init(pcf8563_register_types)
