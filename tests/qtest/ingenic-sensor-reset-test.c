/*
 * QTest testcase for the image sensors' soft reset: a write that sets a
 * model's reset bit makes it NACK its address for a while, and it answers
 * again after that, in virtual time. An OV9712 (0x12 = 0x80) has a window by
 * default; reset-busy-us sets one on any model and 0 turns it off.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2C0            0x10050000
#define TAR             0x04
#define DC              0x10
#define RAWINT          0x34
#define CTXABRT         0x54
#define ENB             0x6c
#define RXFLR           0x78
#define TXABRT          0x80

#define CMD_READ        0x100
#define CMD_STOP        0x200
#define INT_TX_ABRT     (1u << 6)
#define ABRT_7B_NOACK   1u

static QTestState *start(const char *model, const char *extra)
{
    return qtest_initf("-machine ingenic-t31 "
                       "-global ingenic-t31.soc-variant=t31lc "
                       "-device image-sensor,"
                       "mclk-gate=off,bus=i2c0,model=%s%s", model, extra);
}

static void select_target(QTestState *qts, unsigned addr)
{
    qtest_writel(qts, I2C0 + ENB, 0);
    qtest_writel(qts, I2C0 + TAR, addr);
    qtest_writel(qts, I2C0 + ENB, 1);
}

static void write_reg(QTestState *qts, unsigned bytes, unsigned reg,
                      unsigned val)
{
    if (bytes == 2) {
        qtest_writel(qts, I2C0 + DC, reg >> 8);
    }
    qtest_writel(qts, I2C0 + DC, reg & 0xff);
    qtest_writel(qts, I2C0 + DC, CMD_STOP | val);
}

/* Read one register: the value, or -1 if the address was NACKed */
static int read_reg(QTestState *qts, unsigned bytes, unsigned reg)
{
    int v = -1;

    qtest_readl(qts, I2C0 + CTXABRT);
    if (bytes == 2) {
        qtest_writel(qts, I2C0 + DC, reg >> 8);
    }
    qtest_writel(qts, I2C0 + DC, reg & 0xff);
    qtest_writel(qts, I2C0 + DC, CMD_READ | CMD_STOP);
    if (qtest_readl(qts, I2C0 + RAWINT) & INT_TX_ABRT) {
        g_assert_cmphex(qtest_readl(qts, I2C0 + TXABRT) & ABRT_7B_NOACK, ==,
                        ABRT_7B_NOACK);
    } else {
        g_assert_cmpuint(qtest_readl(qts, I2C0 + RXFLR), ==, 1);
        v = qtest_readl(qts, I2C0 + DC) & 0xff;
    }
    qtest_readl(qts, I2C0 + CTXABRT);
    return v;
}

/* The OV9712's 0x12 = 0x80: NACK at once, answering after its window */
static void test_ov9712(void)
{
    QTestState *qts = start("ov9712", "");

    select_target(qts, 0x30);
    g_assert_cmphex(read_reg(qts, 1, 0x0a), ==, 0x97);
    write_reg(qts, 1, 0x12, 0x80);
    g_assert_cmpint(read_reg(qts, 1, 0x0a), ==, -1);
    /* a T10L NACKed a read at once and answered one ~166 us after */
    qtest_clock_step(qts, 100 * 1000);
    g_assert_cmpint(read_reg(qts, 1, 0x0a), ==, -1);
    qtest_clock_step(qts, 100 * 1000);
    g_assert_cmphex(read_reg(qts, 1, 0x0a), ==, 0x97);
    qtest_quit(qts);
}

/* A write that does not set the bit leaves it answering */
static void test_ov9712_other(void)
{
    QTestState *qts = start("ov9712", "");

    select_target(qts, 0x30);
    write_reg(qts, 1, 0x12, 0x40);
    g_assert_cmphex(read_reg(qts, 1, 0x0a), ==, 0x97);
    qtest_quit(qts);
}

static void test_off(void)
{
    QTestState *qts = start("ov9712", ",reset-busy-us=0");

    select_target(qts, 0x30);
    write_reg(qts, 1, 0x12, 0x80);
    g_assert_cmphex(read_reg(qts, 1, 0x0a), ==, 0x97);
    qtest_quit(qts);
}

/* A model with no window of its own takes one from the property */
static void test_sc2336(void)
{
    QTestState *qts = start("sc2336", ",reset-busy-us=1000");

    select_target(qts, 0x30);
    write_reg(qts, 2, 0x0103, 0x01);
    g_assert_cmpint(read_reg(qts, 2, 0x3107), ==, -1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(read_reg(qts, 2, 0x3107), ==, 0xcb);
    qtest_quit(qts);
}

static void test_sc2336_default(void)
{
    QTestState *qts = start("sc2336", "");

    select_target(qts, 0x30);
    write_reg(qts, 2, 0x0103, 0x01);
    g_assert_cmphex(read_reg(qts, 2, 0x3107), ==, 0xcb);
    qtest_quit(qts);
}

static void test_gc4653(void)
{
    QTestState *qts = start("gc4653", ",reset-busy-us=1000");

    select_target(qts, 0x29);
    write_reg(qts, 2, 0x03fe, 0x00);
    g_assert_cmpint(read_reg(qts, 2, 0x03f0), !=, -1);
    write_reg(qts, 2, 0x03fe, 0xf0);
    g_assert_cmpint(read_reg(qts, 2, 0x03f0), ==, -1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmpint(read_reg(qts, 2, 0x03f0), !=, -1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-sensor-reset/ov9712", test_ov9712);
    qtest_add_func("ingenic-sensor-reset/ov9712-other", test_ov9712_other);
    qtest_add_func("ingenic-sensor-reset/off", test_off);
    qtest_add_func("ingenic-sensor-reset/sc2336", test_sc2336);
    qtest_add_func("ingenic-sensor-reset/sc2336-default", test_sc2336_default);
    qtest_add_func("ingenic-sensor-reset/gc4653", test_gc4653);
    return g_test_run();
}
