/*
 * QTest testcase for the sensor's MCLK
 *
 * On a T31X MCLK reaches the sensor from PA15 on pin function 1 only
 * (function 0 reads constant low), and a CIM divider left with CE set does
 * not run until CE is written back to 0. The image sensor answers on I2C
 * while MCLK reaches it and its reset pin is not held, and not otherwise.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "libqtest.h"

#define CPM             0x10000000
#define CIMCDR          (CPM + 0x7c)
#define CPVPCR          (CPM + 0xe0)
#define GPIO            0x10010000
#define PA              GPIO
#define I2C0            0x10050000
#define TAR             0x04
#define DC              0x10
#define RAWINT          0x34
#define CTXABRT         0x54
#define ENB             0x6c
#define RXFLR           0x78

#define CMD_READ        0x100
#define CMD_STOP        0x200
#define INT_TX_ABRT     (1u << 6)

#define CE              0x20000000
#define STOP            0x08000000
#define VPLL_DIV        0x80000031      /* the VPLL's, by 49 + 1 */
#define VPLL_ON         0x0640510d

static QTestState *start(const char *extra)
{
    return qtest_initf("-machine ingenic-t31 "
                       "-global ingenic-t31.soc-variant=t31x "
                       "-device image-sensor,bus=i2c0,model=os02g10%s", extra);
}

/* PA15 or another pin: INT, MSK and PAT1 clear, PAT0 as the function */
static void pin_func(QTestState *qts, unsigned pin, unsigned func)
{
    qtest_writel(qts, PA + 0x18, BIT(pin));
    qtest_writel(qts, PA + 0x28, BIT(pin));
    qtest_writel(qts, PA + (func & 2 ? 0x34 : 0x38), BIT(pin));
    qtest_writel(qts, PA + (func & 1 ? 0x44 : 0x48), BIT(pin));
}

/* An output driving @level */
static void pin_out(QTestState *qts, unsigned pin, unsigned level)
{
    qtest_writel(qts, PA + 0x18, BIT(pin));
    qtest_writel(qts, PA + 0x24, BIT(pin));
    qtest_writel(qts, PA + 0x38, BIT(pin));
    qtest_writel(qts, PA + (level ? 0x44 : 0x48), BIT(pin));
}

/* A CIM divider's write as the vendor kernel does it: with CE, then without */
static void cim_set(QTestState *qts, uint32_t div)
{
    qtest_writel(qts, CIMCDR, div | CE);
    qtest_writel(qts, CIMCDR, div);
}

static bool acks(QTestState *qts)
{
    qtest_writel(qts, I2C0 + ENB, 0);
    qtest_writel(qts, I2C0 + TAR, 0x3c);
    qtest_writel(qts, I2C0 + ENB, 1);
    qtest_readl(qts, I2C0 + CTXABRT);
    qtest_writel(qts, I2C0 + DC, 0xfd);
    qtest_writel(qts, I2C0 + DC, CMD_STOP);
    qtest_writel(qts, I2C0 + DC, 0x02);
    qtest_writel(qts, I2C0 + DC, CMD_READ | CMD_STOP);
    return !(qtest_readl(qts, I2C0 + RAWINT) & INT_TX_ABRT);
}

static void test_right(void)
{
    QTestState *qts = start("");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    pin_func(qts, 15, 1);
    cim_set(qts, VPLL_DIV);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

static void test_function0(void)
{
    QTestState *qts = start("");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    cim_set(qts, VPLL_DIV);
    pin_func(qts, 15, 0);
    g_assert_false(acks(qts));
    pin_func(qts, 15, 2);
    g_assert_false(acks(qts));
    pin_func(qts, 15, 1);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

static void test_ce_left_set(void)
{
    QTestState *qts = start("");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    pin_func(qts, 15, 1);
    qtest_writel(qts, CIMCDR, VPLL_DIV | CE);
    g_assert_cmphex(qtest_readl(qts, CIMCDR) & CE, ==, CE);
    g_assert_false(acks(qts));
    qtest_writel(qts, CIMCDR, VPLL_DIV);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

static void test_stop(void)
{
    QTestState *qts = start("");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    pin_func(qts, 15, 1);
    cim_set(qts, VPLL_DIV | STOP);
    g_assert_false(acks(qts));
    cim_set(qts, VPLL_DIV);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

/* STOP written alone, CE clear, as a clock driver gates the divider */
static void test_stop_alone(void)
{
    QTestState *qts = start("");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    pin_func(qts, 15, 1);
    cim_set(qts, VPLL_DIV);
    g_assert_true(acks(qts));
    qtest_writel(qts, CIMCDR, VPLL_DIV | STOP);
    g_assert_false(acks(qts));
    qtest_writel(qts, CIMCDR, VPLL_DIV);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

static void test_pll_off(void)
{
    QTestState *qts = start("");

    pin_func(qts, 15, 1);
    cim_set(qts, VPLL_DIV);
    g_assert_false(acks(qts));
    qtest_writel(qts, CPVPCR, VPLL_ON);
    qtest_writel(qts, CIMCDR, VPLL_DIV);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

static void test_reset(void)
{
    QTestState *qts = start(",reset=18o");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    pin_func(qts, 15, 1);
    cim_set(qts, VPLL_DIV);
    pin_out(qts, 18, 0);
    g_assert_false(acks(qts));
    pin_out(qts, 18, 1);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

static void test_reset_high(void)
{
    QTestState *qts = start(",reset=18O");

    qtest_writel(qts, CPVPCR, VPLL_ON);
    pin_func(qts, 15, 1);
    cim_set(qts, VPLL_DIV);
    pin_out(qts, 18, 1);
    g_assert_false(acks(qts));
    pin_out(qts, 18, 0);
    g_assert_true(acks(qts));
    qtest_quit(qts);
}

/* Parts whose MCLK pad is not known answer as they always did */
static void test_unknown_pad(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 "
                                 "-device image-sensor,bus=i2c0,"
                                 "model=os02g10");

    g_assert_true(acks(qts));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-sensor-mclk/right", test_right);
    qtest_add_func("ingenic-sensor-mclk/function0", test_function0);
    qtest_add_func("ingenic-sensor-mclk/ce-left-set", test_ce_left_set);
    qtest_add_func("ingenic-sensor-mclk/stop", test_stop);
    qtest_add_func("ingenic-sensor-mclk/stop-alone", test_stop_alone);
    qtest_add_func("ingenic-sensor-mclk/pll-off", test_pll_off);
    qtest_add_func("ingenic-sensor-mclk/reset", test_reset);
    qtest_add_func("ingenic-sensor-mclk/reset-high", test_reset_high);
    qtest_add_func("ingenic-sensor-mclk/unknown-pad", test_unknown_pad);
    return g_test_run();
}
