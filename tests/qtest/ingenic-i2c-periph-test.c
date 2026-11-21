/*
 * QTest testcase for the I2C parts a T31X's camera board carries beside
 * its sensor: a 24C16 EEPROM and a PCF8563 real-time clock
 *
 * The test is the guest's I2C driver: it drives I2C1's DesignWare
 * registers (TAR, then data commands with STOP in bit 9) as the vendor
 * kernel does. The EEPROM's eight addresses (0x50 to 0x57), each a 256
 * byte block, and its page write follow the 24C16 data sheet. The RTC
 * follows the PCF8563 data sheet (Rev. 11.1): the register reset values
 * of Table 27, the flags and INT of section 8.3.2, the century flag, the
 * alarm as the minute increments, the countdown timer and STOP. INT is
 * read on the SoC pad it is wired to.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/timer.h"

#define I2C1            0x10051000
#define TAR             0x04
#define DC              0x10
#define ENB             0x6c
#define RXFLR           0x78
#define TXABRT          0x80
#define CTXABRT         0x54
#define DC_READ         0x100
#define DC_STOP         0x200

#define GPIO            0x10010000
#define INT_PIN         40              /* PB8 */

#define MACHINE "-machine ingenic-t31 -global ingenic-t31.soc-variant=t31x "

static void i2c_write(QTestState *qts, int addr, const uint8_t *d, int n,
                      bool stop)
{
    qtest_writel(qts, I2C1 + TAR, addr);
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, I2C1 + DC,
                     d[i] | (stop && i == n - 1 ? DC_STOP : 0));
    }
}

/* Reads after a write with no STOP turn the bus round, as the core does */
static void i2c_read(QTestState *qts, int addr, uint8_t *d, int n)
{
    qtest_writel(qts, I2C1 + TAR, addr);
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, I2C1 + DC, DC_READ | (i == n - 1 ? DC_STOP : 0));
    }
    g_assert_cmpuint(qtest_readl(qts, I2C1 + RXFLR), ==, n);
    for (int i = 0; i < n; i++) {
        d[i] = qtest_readl(qts, I2C1 + DC);
    }
}

static void put(QTestState *qts, int addr, uint8_t reg, const uint8_t *d,
                int n)
{
    uint8_t buf[300];

    buf[0] = reg;
    memcpy(buf + 1, d, n);
    i2c_write(qts, addr, buf, n + 1, true);
}

static void get(QTestState *qts, int addr, uint8_t reg, uint8_t *d, int n)
{
    i2c_write(qts, addr, &reg, 1, false);
    i2c_read(qts, addr, d, n);
}

static QTestState *init(const char *extra)
{
    QTestState *qts = qtest_initf(MACHINE "%s", extra);

    qtest_writel(qts, I2C1 + ENB, 1);
    return qts;
}

static bool nacked(QTestState *qts)
{
    bool r = qtest_readl(qts, I2C1 + TXABRT) & 1;

    qtest_readl(qts, I2C1 + CTXABRT);
    return r;
}

#define EEPROM "-device at24c-eeprom,bus=i2c1,address=0x50,rom-size=2048"

/* One block per address: the same offset at 0x50 and 0x53 is two bytes */
static void test_24c16_blocks(void)
{
    QTestState *qts = init(EEPROM);
    uint8_t a[3] = { 'A', 'B', 'C' }, b[3] = { 'X', 'Y', 'Z' }, r[3];

    put(qts, 0x50, 0x10, a, 3);
    put(qts, 0x53, 0x10, b, 3);
    put(qts, 0x57, 0xff, b, 1);
    get(qts, 0x50, 0x10, r, 3);
    g_assert_cmpmem(r, 3, a, 3);
    get(qts, 0x53, 0x10, r, 3);
    g_assert_cmpmem(r, 3, b, 3);
    /* 0x57 offset 0xff is the last byte (0x7ff); the next read wraps to 0 */
    get(qts, 0x57, 0xff, r, 2);
    g_assert_cmpuint(r[0], ==, 'X');
    g_assert_cmpuint(r[1], ==, 0);
    /* a read runs on across the block edge into the next block */
    put(qts, 0x51, 0x00, a, 1);
    get(qts, 0x50, 0xff, r, 2);
    g_assert_cmpuint(r[1], ==, 'A');
    /* only eight addresses */
    qtest_writel(qts, I2C1 + TAR, 0x58);
    qtest_writel(qts, I2C1 + DC, 0);
    g_assert_true(nacked(qts));
    qtest_quit(qts);
}

/* A write stays in its page: it wraps to the page's start */
static void test_24c16_page(void)
{
    QTestState *qts = init(EEPROM ",page-size=16");
    uint8_t d[20], r[20];

    for (int i = 0; i < 20; i++) {
        d[i] = 0x40 + i;
    }
    put(qts, 0x52, 0x2c, d, 20);
    get(qts, 0x52, 0x20, r, 16);
    /* byte 4 wrapped to 0x20; bytes 16 to 19 wrapped again over 0x2c to 0x2f */
    g_assert_cmpuint(r[0], ==, 0x44);
    g_assert_cmpuint(r[11], ==, 0x4f);
    g_assert_cmpuint(r[12], ==, 0x50);
    g_assert_cmpuint(r[15], ==, 0x53);
    qtest_quit(qts);
}

/* A 24C02 keeps one address and rejects a block address */
static void test_24c02(void)
{
    QTestState *qts = init("-device at24c-eeprom,bus=i2c1,address=0x50,"
                           "rom-size=256");
    uint8_t d = 0x5a, r;

    put(qts, 0x50, 0x80, &d, 1);
    get(qts, 0x50, 0x80, &r, 1);
    g_assert_cmpuint(r, ==, 0x5a);
    qtest_writel(qts, I2C1 + TAR, 0x51);
    qtest_writel(qts, I2C1 + DC, 0);
    g_assert_true(nacked(qts));
    qtest_quit(qts);
}

#define RTC "-device pcf8563,bus=i2c1,int-pin=40"
#define BASE "-rtc clock=vm,base=2026-11-21T13:29:30 "

static uint8_t rtc_reg(QTestState *qts, uint8_t reg)
{
    uint8_t v;

    get(qts, 0x51, reg, &v, 1);
    return v;
}

static void rtc_put(QTestState *qts, uint8_t reg, uint8_t v)
{
    put(qts, 0x51, reg, &v, 1);
}

static bool int_active(QTestState *qts)
{
    uint32_t pin = qtest_readl(qts, GPIO + (INT_PIN / 32) * 0x1000);

    return !(pin & (1u << (INT_PIN % 32)));
}

/* Table 27, and the time that starts as the clock's */
static void test_rtc_reset(void)
{
    QTestState *qts = init(BASE RTC);
    uint8_t r[16];

    get(qts, 0x51, 0, r, 16);
    g_assert_cmphex(r[0], ==, 0x08);
    g_assert_cmphex(r[1], ==, 0x00);
    g_assert_cmphex(r[2], ==, 0x30);
    g_assert_cmphex(r[3], ==, 0x29);
    g_assert_cmphex(r[4], ==, 0x13);
    g_assert_cmphex(r[5], ==, 0x21);
    g_assert_cmphex(r[6], ==, 0x06);
    g_assert_cmphex(r[7], ==, 0x91);    /* November, C for the 2000s */
    g_assert_cmphex(r[8], ==, 0x26);
    for (int i = 9; i < 13; i++) {
        g_assert_cmphex(r[i], ==, 0x80);
    }
    g_assert_cmphex(r[13], ==, 0x80);
    g_assert_cmphex(r[14], ==, 0x03);
    g_assert_false(int_active(qts));
    qtest_clock_step(qts, 5 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 2), ==, 0x35);
    qtest_quit(qts);
}

/* VL is set at power-up, until the interface writes the seconds */
static void test_rtc_vl(void)
{
    QTestState *qts = init(BASE RTC ",vl=on");
    uint8_t t[7] = { 0x00, 0x15, 0x09, 0x01, 0x03, 0x81, 0x00 };
    uint8_t r[7];

    g_assert_cmphex(rtc_reg(qts, 2) & 0x80, ==, 0x80);
    put(qts, 0x51, 2, t, 7);
    get(qts, 0x51, 2, r, 7);
    g_assert_cmphex(r[0], ==, 0x00);
    g_assert_cmphex(r[1], ==, 0x15);
    g_assert_cmphex(r[2], ==, 0x09);
    g_assert_cmphex(r[3], ==, 0x01);
    g_assert_cmphex(r[4], ==, 0x03);
    g_assert_cmphex(r[5], ==, 0x81);
    g_assert_cmphex(r[6], ==, 0x00);
    /* the century flag toggles as the years overflow 99 to 00 */
    t[0] = 0x59; t[1] = 0x59; t[2] = 0x23; t[3] = 0x31; t[4] = 0x04;
    t[5] = 0x92; t[6] = 0x99;
    put(qts, 0x51, 2, t, 7);
    qtest_clock_step(qts, NANOSECONDS_PER_SECOND);
    get(qts, 0x51, 2, r, 7);
    g_assert_cmphex(r[0], ==, 0x00);
    g_assert_cmphex(r[3], ==, 0x01);
    g_assert_cmphex(r[5], ==, 0x01);    /* 2100: C is clear again */
    g_assert_cmphex(r[6], ==, 0x00);
    qtest_quit(qts);
}

/* STOP holds the time; releasing it lets it run on from there */
static void test_rtc_stop(void)
{
    QTestState *qts = init(BASE RTC);

    rtc_put(qts, 0, 0x28);
    qtest_clock_step(qts, 10 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 2), ==, 0x30);
    rtc_put(qts, 0, 0x08);
    qtest_clock_step(qts, 2 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 2), ==, 0x32);
    qtest_quit(qts);
}

/*
 * The alarm sets AF as the minute increments to a match: not for a time
 * set to match, and once per match. AIE gates INT; a written 1 leaves AF
 * and a written 0 clears it.
 */
static void test_rtc_alarm(void)
{
    QTestState *qts = init(BASE RTC);
    uint8_t al[2] = { 0x30, 0x13 };

    /* now is 13:29:30: an alarm for 13:29 does not fire */
    rtc_put(qts, 9, 0x29);
    rtc_put(qts, 10, 0x13);
    rtc_put(qts, 1, 0x02);
    qtest_clock_step(qts, 40 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 1) & 0x08, ==, 0);
    g_assert_false(int_active(qts));

    /* 13:30:10 now: 13:31 does */
    put(qts, 0x51, 9, al, 2);
    rtc_put(qts, 9, 0x31);
    qtest_clock_step(qts, 49 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 1) & 0x08, ==, 0);
    qtest_clock_step(qts, 2 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 1) & 0x0a, ==, 0x0a);
    g_assert_true(int_active(qts));
    rtc_put(qts, 1, 0x0a);              /* a written 1 changes nothing */
    g_assert_true(int_active(qts));
    rtc_put(qts, 1, 0x02);
    g_assert_cmphex(rtc_reg(qts, 1) & 0x08, ==, 0);
    g_assert_false(int_active(qts));

    /* AIE off: AF sets, INT stays released */
    rtc_put(qts, 1, 0x00);
    rtc_put(qts, 9, 0x32);
    qtest_clock_step(qts, 60 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 1) & 0x08, ==, 0x08);
    g_assert_false(int_active(qts));
    qtest_quit(qts);
}

/* The countdown timer: TF at the end, INT as a level or as a pulse */
static void test_rtc_timer(void)
{
    QTestState *qts = init(BASE RTC);

    rtc_put(qts, 0x0f, 3);
    rtc_put(qts, 1, 0x01);              /* TIE, level */
    rtc_put(qts, 0x0e, 0x82);           /* TE, 1 Hz */
    qtest_clock_step(qts, 1500 * 1000000LL);
    g_assert_cmpuint(rtc_reg(qts, 0x0f), ==, 2);
    g_assert_cmphex(rtc_reg(qts, 1) & 4, ==, 0);
    qtest_clock_step(qts, 1600 * 1000000LL);
    g_assert_cmphex(rtc_reg(qts, 1) & 4, ==, 4);
    g_assert_true(int_active(qts));
    rtc_put(qts, 1, 0x01);              /* clear TF */
    g_assert_false(int_active(qts));

    /* pulse mode: INT is low for 1/64 s, TF stays set */
    rtc_put(qts, 1, 0x11);
    qtest_clock_step(qts, 2905 * 1000000LL);
    g_assert_true(int_active(qts));
    g_assert_cmphex(rtc_reg(qts, 1) & 4, ==, 4);
    qtest_clock_step(qts, 20 * 1000000LL);
    g_assert_false(int_active(qts));

    /* n = 0 stops the timer */
    rtc_put(qts, 0x0e, 0x00);
    rtc_put(qts, 0x0f, 0);
    rtc_put(qts, 1, 0x01);
    rtc_put(qts, 0x0e, 0x82);
    qtest_clock_step(qts, 5 * NANOSECONDS_PER_SECOND);
    g_assert_cmphex(rtc_reg(qts, 1) & 4, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/i2c/24c16-blocks", test_24c16_blocks);
    qtest_add_func("ingenic/i2c/24c16-page", test_24c16_page);
    qtest_add_func("ingenic/i2c/24c02", test_24c02);
    qtest_add_func("ingenic/i2c/pcf8563-reset", test_rtc_reset);
    qtest_add_func("ingenic/i2c/pcf8563-vl", test_rtc_vl);
    qtest_add_func("ingenic/i2c/pcf8563-stop", test_rtc_stop);
    qtest_add_func("ingenic/i2c/pcf8563-alarm", test_rtc_alarm);
    qtest_add_func("ingenic/i2c/pcf8563-timer", test_rtc_timer);
    return g_test_run();
}
