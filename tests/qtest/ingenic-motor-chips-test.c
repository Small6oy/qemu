/*
 * QTest testcase for the motor driver chips: the MS419xx and the TMI8152
 * and TMI8153 on a T31's SSI, the MS32006 on a bit-banged I2C bus
 *
 * The test is the vendor sample drivers: it shifts their register writes
 * through the SSI (MS419xx: active-high chip select, bits LSB first, the
 * VD pin pulsed through the GPIO registers; TMI815x: write is address
 * bit 7) or bit-bangs them on I2C, and reads the motors' positions back.
 * The register meanings are the drivers' (no datasheets were at hand).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qobject/qdict.h"

#define GPIO            0x10010000
#define PORT(pin)       (GPIO + ((pin) / 32) * 0x1000)
#define BIT_OF(pin)     (1u << ((pin) % 32))
#define MSK             0x20
#define PAT1            0x30
#define PAT0            0x40
#define MS              1000000ULL

#define SSI0            0x10043000
#define SSI_DR          0x00
#define SSI_CR0         0x04
#define SSI_CR1         0x08
#define SSI_SR          0x0c
#define SR_RFE          BIT(4)

typedef struct Bus {
    QTestState *qts;
    int sda, scl;
} Bus;

static void pin_out(Bus *b, int pin, int level)
{
    uint32_t base = PORT(pin);

    qtest_writel(b->qts, base + MSK + 4, BIT_OF(pin));
    qtest_writel(b->qts, base + PAT1 + 8, BIT_OF(pin));
    qtest_writel(b->qts, base + PAT0 + (level ? 4 : 8), BIT_OF(pin));
}

/* An open-drain release: the pad an input, high unless a device holds it */
static void sda_release(Bus *b)
{
    uint32_t base = PORT(b->sda);

    qtest_writel(b->qts, base + MSK + 4, BIT_OF(b->sda));
    qtest_writel(b->qts, base + PAT1 + 4, BIT_OF(b->sda));
}

static void sda_set(Bus *b, int level)
{
    if (level) {
        sda_release(b);
    } else {
        pin_out(b, b->sda, 0);
    }
}

static int sda_get(Bus *b)
{
    return !!(qtest_readl(b->qts, PORT(b->sda)) & BIT_OF(b->sda));
}

static void scl_set(Bus *b, int level)
{
    pin_out(b, b->scl, level);
}

static void i2c_start(Bus *b)
{
    sda_set(b, 1);
    scl_set(b, 1);
    sda_set(b, 0);
    scl_set(b, 0);
}

static void i2c_stop(Bus *b)
{
    sda_set(b, 0);
    scl_set(b, 1);
    sda_set(b, 1);
}

/* Returns true when the slave ACKs: it pulls SDA low */
static bool i2c_put(Bus *b, uint8_t v)
{
    bool ack;

    for (int i = 7; i >= 0; i--) {
        sda_set(b, v >> i & 1);
        scl_set(b, 1);
        scl_set(b, 0);
    }
    sda_release(b);
    scl_set(b, 1);
    ack = !sda_get(b);
    scl_set(b, 0);
    return ack;
}

static uint8_t i2c_get(Bus *b, bool ack)
{
    uint8_t v = 0;

    sda_release(b);
    for (int i = 0; i < 8; i++) {
        scl_set(b, 1);
        v = v << 1 | sda_get(b);
        scl_set(b, 0);
    }
    sda_set(b, !ack);
    scl_set(b, 1);
    scl_set(b, 0);
    sda_release(b);
    return v;
}

static bool reg_write(Bus *b, uint8_t addr, uint8_t reg, const uint8_t *v,
                      size_t n)
{
    bool ok;

    i2c_start(b);
    ok = i2c_put(b, addr << 1) && i2c_put(b, reg);
    for (size_t i = 0; ok && i < n; i++) {
        ok = i2c_put(b, v[i]);
    }
    i2c_stop(b);
    return ok;
}

static void reg_write1(Bus *b, uint8_t addr, uint8_t reg, uint8_t v)
{
    g_assert_true(reg_write(b, addr, reg, &v, 1));
}

static void reg_read(Bus *b, uint8_t addr, uint8_t reg, uint8_t *v, size_t n)
{
    i2c_start(b);
    g_assert_true(i2c_put(b, addr << 1));
    g_assert_true(i2c_put(b, reg));
    /* repeated START */
    sda_release(b);
    scl_set(b, 1);
    sda_set(b, 0);
    scl_set(b, 0);
    g_assert_true(i2c_put(b, addr << 1 | 1));
    for (size_t i = 0; i < n; i++) {
        v[i] = i2c_get(b, i + 1 < n);
    }
    i2c_stop(b);
}

static uint8_t reg_read1(Bus *b, uint8_t addr, uint8_t reg)
{
    uint8_t v;

    reg_read(b, addr, reg, &v, 1);
    return v;
}

static void gpio_out(QTestState *qts, int pin, int level)
{
    uint32_t base = PORT(pin);

    /* the level first: a pad made an output would drive 0 for a moment */
    qtest_writel(qts, base + PAT0 + (level ? 4 : 8), BIT_OF(pin));
    qtest_writel(qts, base + PAT1 + 8, BIT_OF(pin));
    qtest_writel(qts, base + MSK + 4, BIT_OF(pin));
}

static int64_t prop(QTestState *qts, const char *id, const char *name)
{
    char *path = g_strdup_printf("/machine/peripheral/%s", id);
    QDict *r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                         "{'path': %s, 'property': %s}}", path, name);
    int64_t v = qdict_get_int(r, "return");

    qobject_unref(r);
    g_free(path);
    return v;
}

/*
 * One message: 8-bit frames, the controller's chip select active high when
 * @cs_high, bits LSB first when @lsb (TENDIAN and RENDIAN 1)
 */
static void ssi_msg(QTestState *qts, bool cs_high, bool lsb,
                    const uint8_t *tx, uint8_t *rx, int n)
{
    qtest_writel(qts, SSI0 + SSI_CR1, (cs_high ? BIT(30) : 0) | 6 << 3);
    qtest_writel(qts, SSI0 + SSI_CR0, BIT(15) | BIT(2) | BIT(1) |
                 (lsb ? 1 << 18 | 1 << 16 : 0));
    qtest_writel(qts, SSI0 + SSI_CR0, BIT(15) | (lsb ? 1 << 18 | 1 << 16 : 0));
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, SSI0 + SSI_DR, tx[i]);
    }
    qtest_clock_step(qts, 2 * MS);
    for (int i = 0; i < n; i++) {
        uint32_t sr = qtest_readl(qts, SSI0 + SSI_SR);

        g_assert_false(sr & SR_RFE);
        rx[i] = qtest_readl(qts, SSI0 + SSI_DR);
    }
}

static void ms_write(QTestState *qts, uint8_t reg, uint16_t v)
{
    uint8_t tx[3] = { reg, v & 0xff, v >> 8 }, rx[3];

    ssi_msg(qts, true, true, tx, rx, 3);
}

static uint16_t ms_read(QTestState *qts, uint8_t reg)
{
    uint8_t tx[3] = { reg | 0x40, 0, 0 }, rx[3];

    ssi_msg(qts, true, true, tx, rx, 3);
    return rx[1] | rx[2] << 8;
}

#define VD      63
#define RST     17

/* A VD_FZ pulse, 3 ms a level: a move of PSUM 8 takes 2.4 ms with INTCT 256 */
static void vd_pulse(QTestState *qts)
{
    gpio_out(qts, VD, 1);
    qtest_clock_step(qts, 3 * MS);
    gpio_out(qts, VD, 0);
    qtest_clock_step(qts, 3 * MS);
}

/*
 * The MS41929 datasheet's registers: initial values, the bits kept, a
 * move of PSUM * 8 256ths of a cycle at each rising VD_FZ edge after DT1
 * and DT2, the falling edge with MODESEL_FZ, the stops, the move a coming
 * edge cancels, and ENDIS, BRAKE and DT1 holding the rotor
 */
static void test_ms419xx(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 "
                                 "-global ingenic-t31.soc-variant=t31x "
                                 "-device ms419xx,id=ms,bus=ssi0,cs=0,"
                                 "vd-pin=63,reset-pin=17,range=640");
    int64_t cd;

    gpio_out(qts, RST, 1);
    gpio_out(qts, RST, 0);
    g_assert_cmphex(ms_read(qts, 0x20), ==, 0x3c0a);
    g_assert_cmphex(ms_read(qts, 0x22), ==, 0x0003);
    g_assert_cmphex(ms_read(qts, 0x27), ==, 0x0003);
    ms_write(qts, 0x2c, 0xffff);
    g_assert_cmphex(ms_read(qts, 0x2c), ==, 0x0007);
    ms_write(qts, 0x30, 0x1234);
    g_assert_cmphex(ms_read(qts, 0x30), ==, 0);

    /* what the vendor driver writes */
    ms_write(qts, 0x20, 0x1e01);
    g_assert_cmphex(ms_read(qts, 0x20), ==, 0x1e01);
    ms_write(qts, 0x22, 0x0001);
    ms_write(qts, 0x27, 0x0001);
    ms_write(qts, 0x25, 0x0100);
    ms_write(qts, 0x2a, 0x0100);
    ms_write(qts, 0x24, 0x3408);
    ms_write(qts, 0x29, 0x3508);
    g_assert_cmphex(ms_read(qts, 0x24), ==, 0x3408);
    g_assert_cmphex(ms_read(qts, 0x29), ==, 0x3508);

    /* a released pad reads high: the first rising edge comes after a low */
    gpio_out(qts, VD, 0);
    qtest_clock_step(qts, 3 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 0);

    /* the rising edge starts it, the waits and the steps take their time */
    gpio_out(qts, VD, 1);
    qtest_clock_step(qts, 1 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 0);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 64);
    /* CD is reverse from 0: a stop */
    g_assert_cmpint(prop(qts, "ms", "position-cd"), ==, 0);
    g_assert_cmpint(prop(qts, "ms", "stalls-cd"), ==, 1);
    gpio_out(qts, VD, 0);
    qtest_clock_step(qts, 3 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 64);

    for (int i = 0; i < 11; i++) {
        vd_pulse(qts);
    }
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 640);
    g_assert_cmpint(prop(qts, "ms", "stalls-ab"), ==, 2);
    ms_write(qts, 0x24, 0x3000);
    ms_write(qts, 0x29, 0x3408);
    vd_pulse(qts);
    vd_pulse(qts);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 640);
    g_assert_cmpint(prop(qts, "ms", "position-cd"), ==, 128);

    /* a move too long for the VD interval is cancelled at the next edge */
    ms_write(qts, 0x2a, 0x0400);
    vd_pulse(qts);
    vd_pulse(qts);
    cd = prop(qts, "ms", "position-cd");
    g_assert_cmpint(cd, >, 128 + 10);
    g_assert_cmpint(cd, <, 128 + 64);
    /* and the move that edge started runs its 64 units out */
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-cd"), ==, cd + 64);
    cd += 64;
    ms_write(qts, 0x2a, 0x0100);

    /* held: outputs off, brake, no step cycle, no DT1 */
    ms_write(qts, 0x29, 0x3008);
    vd_pulse(qts);
    ms_write(qts, 0x29, 0x3608);
    vd_pulse(qts);
    ms_write(qts, 0x29, 0x3408);
    ms_write(qts, 0x2a, 0);
    vd_pulse(qts);
    ms_write(qts, 0x2a, 0x0100);
    ms_write(qts, 0x20, 0x1e00);
    vd_pulse(qts);
    ms_write(qts, 0x20, 0x1e01);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 640);
    g_assert_cmpint(prop(qts, "ms", "position-cd"), ==, cd);

    /* MODESEL_FZ: the falling edge acts and the rising one does not */
    ms_write(qts, 0x0b, 0x0400);
    ms_write(qts, 0x24, 0x3508);
    gpio_out(qts, VD, 1);
    qtest_clock_step(qts, 3 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 640);
    gpio_out(qts, VD, 0);
    qtest_clock_step(qts, 3 * MS);
    g_assert_cmpint(prop(qts, "ms", "position-ab"), ==, 576);

    /* the reset pin gives the initial values back */
    gpio_out(qts, RST, 1);
    g_assert_cmphex(ms_read(qts, 0x29), ==, 0);
    g_assert_cmphex(ms_read(qts, 0x20), ==, 0x3c0a);
    qtest_quit(qts);
}

/* TMI815x: 8-bit registers, the write is address bit 7 */
static void tmi_write(QTestState *qts, uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = { reg | 0x80, v }, rx[2];

    ssi_msg(qts, false, false, tx, rx, 2);
}

static uint8_t tmi_read(QTestState *qts, uint8_t reg)
{
    uint8_t tx[2] = { reg, 0 }, rx[2];

    ssi_msg(qts, false, false, tx, rx, 2);
    return rx[1];
}

static char *prop_str(QTestState *qts, const char *id, const char *name)
{
    char *path = g_strdup_printf("/machine/peripheral/%s", id);
    QDict *r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                         "{'path': %s, 'property': %s}}", path, name);
    char *v = g_strdup(qdict_get_str(r, "return"));

    qobject_unref(r);
    g_free(path);
    return v;
}

static void check_ircut(QTestState *qts, const char *state)
{
    char *v = prop_str(qts, "tmi", "ircut");

    g_assert_cmpstr(v, ==, state);
    g_free(v);
}

static QTestState *tmi_init(const char *type)
{
    return qtest_initf("-machine ingenic-t31 "
                       "-global ingenic-t31.soc-variant=t31x "
                       "-device %s,id=tmi,bus=ssi0,cs=0,step-us=100", type);
}

static void test_tmi8152(void)
{
    QTestState *qts = tmi_init("tmi8152");

    tmi_write(qts, 0x02, 0x03);
    tmi_write(qts, 0x02, 0x8b);
    tmi_write(qts, 0x08, 0);
    tmi_write(qts, 0x09, 0);
    tmi_write(qts, 0x04, 0x07);
    tmi_write(qts, 0x05, 7 << 4 | 0x5);
    tmi_write(qts, 0x0b, 20);
    tmi_write(qts, 0x0c, 0);
    g_assert_cmphex(tmi_read(qts, 0x0b), ==, 20);
    tmi_write(qts, 0x04, 0x87);
    qtest_clock_step(qts, 5 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 20);
    g_assert_cmphex(tmi_read(qts, 0x08), ==, 20);
    g_assert_cmpint(prop(qts, "tmi", "position-2"), ==, 0);

    /* channel 2 counter-clockwise: 13 bits, the target 0x2000 less the steps */
    tmi_write(qts, 0x14, 0x07);
    tmi_write(qts, 0x15, 7 << 4 | 0xa);
    tmi_write(qts, 0x1b, 0xfb);
    tmi_write(qts, 0x1c, 0x1f);
    tmi_write(qts, 0x14, 0x87);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-2"), ==, -5);
    g_assert_cmphex(tmi_read(qts, 0x18), ==, 0xfb);
    g_assert_cmphex(tmi_read(qts, 0x19), ==, 0x1f);

    /* the chip ID, and the low-power bit stopping a channel */
    g_assert_cmphex(tmi_read(qts, 0x00), ==, 0x81);
    g_assert_cmphex(tmi_read(qts, 0x01), ==, 0x50);
    tmi_write(qts, 0x02, 0xc1);
    tmi_write(qts, 0x0b, 40);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 20);
    tmi_write(qts, 0x02, 0x8b);
    qtest_clock_step(qts, 5 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 40);

    /* the IR-cut bridge: IREN, IRINA and IRINB as the TMI8150B's table */
    tmi_write(qts, 0x11, 0x86);
    check_ircut(qts, "forwards");
    tmi_write(qts, 0x11, 0x8a);
    check_ircut(qts, "backwards");
    tmi_write(qts, 0x11, 0x8e);
    check_ircut(qts, "brake");
    tmi_write(qts, 0x11, 0x82);
    check_ircut(qts, "coast");
    tmi_write(qts, 0x11, 0x06);
    check_ircut(qts, "off");

    /* a disabled channel stands, and the chip reset clears the counters */
    tmi_write(qts, 0x02, 0x03);
    g_assert_cmphex(tmi_read(qts, 0x08), ==, 0);
    qtest_quit(qts);
}

/*
 * The sequence of a reverse engineered driver for the TMI8152
 * (sstepansky/tmi8152-spi-dev, cross-check only): the ID, init and enable
 * through 0x02, a counter cleared before each move, the direction 0x75 or
 * 0x7a, the target 13 bits (0x2000 less the steps counter-clockwise), mode
 * 2 started with 0x82, the position read back from 0x08 and 0x09
 */
static void test_tmi8152_driver(void)
{
    QTestState *qts = tmi_init("tmi8152");

    g_assert_cmphex(tmi_read(qts, 0x00) << 8 | tmi_read(qts, 0x01), ==,
                    0x8150);
    tmi_write(qts, 0x02, 0x03);
    tmi_write(qts, 0x02, 0x8b);
    tmi_write(qts, 0x08, 0);
    tmi_write(qts, 0x09, 0);
    tmi_write(qts, 0x04, 0x02);
    tmi_write(qts, 0x05, 0x75);
    tmi_write(qts, 0x06, 0x40);
    tmi_write(qts, 0x04, 0x02);
    tmi_write(qts, 0x0b, 100);
    tmi_write(qts, 0x0c, 0);
    tmi_write(qts, 0x04, 0x82);
    qtest_clock_step(qts, 15 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 100);
    g_assert_cmphex(tmi_read(qts, 0x08), ==, 100);
    g_assert_cmphex(tmi_read(qts, 0x09), ==, 0);

    /* back 40 (to the right): cleared, 0x7a, 0x2000 - 40 */
    tmi_write(qts, 0x08, 0);
    tmi_write(qts, 0x09, 0);
    tmi_write(qts, 0x04, 0x02);
    tmi_write(qts, 0x05, 0x7a);
    tmi_write(qts, 0x0b, (0x2000 - 40) & 0xff);
    tmi_write(qts, 0x0c, (0x2000 - 40) >> 8);
    tmi_write(qts, 0x04, 0x82);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 60);
    g_assert_cmphex(tmi_read(qts, 0x08), ==, (0x2000 - 40) & 0xff);
    g_assert_cmphex(tmi_read(qts, 0x09), ==, (0x2000 - 40) >> 8);

    /* continuous rotation, mode 3, runs on until the start bit is cleared */
    tmi_write(qts, 0x05, 0x75);
    tmi_write(qts, 0x04, 0x83);
    qtest_clock_step(qts, 5 * MS);
    tmi_write(qts, 0x04, 0x03);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), >, 60 + 40);
    qtest_quit(qts);
}

/*
 * TMI8160 V1.2, Register List p.9 and Control Interface p.7: the defaults,
 * the ID, reserved addresses and continuous bytes
 */
static void test_tmi8160_regs(void)
{
    QTestState *qts = tmi_init("tmi8160");
    uint8_t tx[3], rx[3];

    g_assert_cmphex(tmi_read(qts, 0x00), ==, 0x81);
    g_assert_cmphex(tmi_read(qts, 0x01), ==, 0x60);
    g_assert_cmphex(tmi_read(qts, 0x02), ==, 0x80);
    g_assert_cmphex(tmi_read(qts, 0x04), ==, 0x90);
    g_assert_cmphex(tmi_read(qts, 0x10), ==, 0x00);
    for (int b = 0x20; b <= 0x40; b += 0x20) {
        g_assert_cmphex(tmi_read(qts, b), ==, 0x07);
        g_assert_cmphex(tmi_read(qts, b + 1), ==, 0x30);
        g_assert_cmphex(tmi_read(qts, b + 2), ==, 0x00);
        g_assert_cmphex(tmi_read(qts, b + 3), ==, 0x07);
        g_assert_cmphex(tmi_read(qts, b + 4), ==, 0x00);
    }

    /* the ID and the states are read only, reserved addresses hold nothing */
    tmi_write(qts, 0x00, 0x55);
    tmi_write(qts, 0x04, 0x55);
    tmi_write(qts, 0x21, 0x55);
    tmi_write(qts, 0x03, 0x55);
    tmi_write(qts, 0x25, 0x55);
    tmi_write(qts, 0x60, 0x55);
    g_assert_cmphex(tmi_read(qts, 0x00), ==, 0x81);
    g_assert_cmphex(tmi_read(qts, 0x04), ==, 0x90);
    g_assert_cmphex(tmi_read(qts, 0x21), ==, 0x30);
    g_assert_cmphex(tmi_read(qts, 0x03), ==, 0);
    g_assert_cmphex(tmi_read(qts, 0x25), ==, 0);
    g_assert_cmphex(tmi_read(qts, 0x60), ==, 0);

    /* a continuous write of the 16-bit turn-count setting, and a read of it */
    tx[0] = 0x2e | 0x80;
    tx[1] = 0x34;
    tx[2] = 0x12;
    ssi_msg(qts, false, false, tx, rx, 3);
    g_assert_cmphex(tmi_read(qts, 0x2e), ==, 0x34);
    g_assert_cmphex(tmi_read(qts, 0x2f), ==, 0x12);
    tx[0] = 0x2e;
    tx[1] = tx[2] = 0;
    ssi_msg(qts, false, false, tx, rx, 3);
    g_assert_cmphex(rx[1] | rx[2] << 8, ==, 0x1234);
    qtest_quit(qts);
}

static void test_tmi8160(const void *type)
{
    QTestState *qts = tmi_init(type);

    tmi_write(qts, 0x02, 0x8a);
    tmi_write(qts, 0x20, 0x1d);
    tmi_write(qts, 0x2a, 0);
    tmi_write(qts, 0x2b, 0);
    tmi_write(qts, 0x22, 7 << 4 | 0x1);
    tmi_write(qts, 0x2e, 30);
    tmi_write(qts, 0x2f, 0);
    tmi_write(qts, 0x20, 0x9d);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 30);
    g_assert_cmphex(tmi_read(qts, 0x2a), ==, 30);

    /* counter-clockwise: the driver writes 65536 minus the count */
    tmi_write(qts, 0x20, 0x1d);
    tmi_write(qts, 0x2a, 0);
    tmi_write(qts, 0x2b, 0);
    tmi_write(qts, 0x22, 7 << 4 | 0x2);
    tmi_write(qts, 0x2e, (65536 - 12) & 0xff);
    tmi_write(qts, 0x2f, (65536 - 12) >> 8);
    tmi_write(qts, 0x20, 0x9d);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-1"), ==, 18);
    g_assert_cmphex(tmi_read(qts, 0x2a), ==, 0xf4);
    g_assert_cmphex(tmi_read(qts, 0x2b), ==, 0xff);
    tmi_write(qts, 0x10, 0x86);
    check_ircut(qts, "forwards");

    /* channel 2 is at +0x20 */
    tmi_write(qts, 0x42, 7 << 4 | 0x1);
    tmi_write(qts, 0x4e, 4);
    tmi_write(qts, 0x40, 0x9d);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "tmi", "position-2"), ==, 4);
    qtest_quit(qts);
}

#define ADDR 0x10

static QTestState *i2c_init(void)
{
    return qtest_init("-machine ingenic-t31 "
                      "-global ingenic-t31.soc-variant=t31x "
                      "-device ingenic-gpio-i2c,id=i2cm,sda=57,scl=58 "
                      "-device ms32006,id=mot,bus=i2cm,address=0x10,"
                      "range=100");
}

static void test_ms32006(void)
{
    QTestState *qts = i2c_init();
    Bus b = { qts, 57, 58 };

    reg_write1(&b, ADDR, 0x00, 0x01);
    /* 1000 steps a second: 24 MHz / 1000 / 128 = 187 */
    reg_write1(&b, ADDR, 0x01, 187);
    reg_write1(&b, ADDR, 0x02, 0x80);
    reg_write1(&b, ADDR, 0x03, 10);
    reg_write1(&b, ADDR, 0x04, 0x80);
    g_assert_cmphex(reg_read1(&b, ADDR, 0x03), ==, 10);
    reg_write1(&b, ADDR, 0x09, 0x80);
    qtest_clock_step(qts, 4500 * 1000ULL);
    g_assert_cmpint(prop(qts, "mot", "position-a"), ==, 4);
    g_assert_cmpint(reg_read1(&b, ADDR, 0x0b), ==, 6);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "mot", "position-a"), ==, 10);
    g_assert_cmpint(reg_read1(&b, ADDR, 0x0b), ==, 0);
    g_assert_cmpint(prop(qts, "mot", "position-b"), ==, 0);

    /* B counter-clockwise from 10 on a range of 100: steps of 3 and a stop */
    reg_write1(&b, ADDR, 0x05, 187);
    reg_write1(&b, ADDR, 0x06, 0x80);
    reg_write1(&b, ADDR, 0x07, 3);
    reg_write1(&b, ADDR, 0x08, 0x80 | 0x40);
    reg_write1(&b, ADDR, 0x09, 0x40);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "mot", "position-b"), ==, 0);
    g_assert_cmpint(prop(qts, "mot", "stalls-b"), ==, 3);

    /* a pause holds A mid-run; clearing the enable stops it */
    reg_write1(&b, ADDR, 0x03, 20);
    reg_write1(&b, ADDR, 0x09, 0x80);
    qtest_clock_step(qts, 2500 * 1000ULL);
    reg_write1(&b, ADDR, 0x00, 0x05);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "mot", "position-a"), ==, 12);
    reg_write1(&b, ADDR, 0x04, 0);
    reg_write1(&b, ADDR, 0x00, 0x01);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmpint(prop(qts, "mot", "position-a"), ==, 12);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("motor-chips/ms419xx", test_ms419xx);
    qtest_add_func("motor-chips/tmi8152", test_tmi8152);
    qtest_add_func("motor-chips/tmi8152-driver", test_tmi8152_driver);
    qtest_add_func("motor-chips/tmi8160-regs", test_tmi8160_regs);
    qtest_add_data_func("motor-chips/tmi8160", "tmi8160", test_tmi8160);
    qtest_add_data_func("motor-chips/tmi8153", "tmi8153", test_tmi8160);
    qtest_add_func("motor-chips/ms32006", test_ms32006);
    return g_test_run();
}
