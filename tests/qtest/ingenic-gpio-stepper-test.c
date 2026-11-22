/*
 * QTest testcase for the GPIO stepper motor
 *
 * The test is the vendors' stepper drivers: it sets four pads of a T31's
 * PB and PC ports one after another through the GPIO registers, in the
 * half-step order the SDK's drivers use (PB22, PB21, PB20, PB19 for the
 * horizontal motor, as the SDK's sample_motor has them), and reads the
 * rotor's position, the stops and the limit switches back.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define GPIO            0x10010000
#define PORT(pin)       (GPIO + ((pin) / 32) * 0x1000)
#define BIT_OF(pin)     (1u << ((pin) % 32))
#define MSK             0x20
#define PAT1            0x30
#define PAT0            0x40
#define PXPIN           0x00

#define PB(n)           (32 + (n))
#define PC(n)           (64 + (n))
#define MS              1000000ULL

static const int phase[4] = { PB(22), PB(21), PB(20), PB(19) };
static const uint8_t half[8] = { 0x8, 0xc, 0x4, 0x6, 0x2, 0x3, 0x1, 0x9 };

static void pin_out(QTestState *qts, int pin, int level)
{
    uint32_t base = PORT(pin);

    /* the level first: a pad made an output would drive 0 for a moment */
    qtest_writel(qts, base + PAT0 + (level ? 4 : 8), BIT_OF(pin));
    qtest_writel(qts, base + PAT1 + 8, BIT_OF(pin));
    qtest_writel(qts, base + MSK + 4, BIT_OF(pin));
}


static int pin_in(QTestState *qts, int pin)
{
    uint32_t base = PORT(pin);

    qtest_writel(qts, base + MSK + 4, BIT_OF(pin));
    qtest_writel(qts, base + PAT1 + 4, BIT_OF(pin));
    return !!(qtest_readl(qts, base + PXPIN) & BIT_OF(pin));
}

/* The driver's motor_move_step(): pins one at a time, then time passes */
static void pattern(QTestState *qts, const int *pins, int n, unsigned bits)
{
    for (int i = 0; i < n; i++) {
        pin_out(qts, pins[i], bits >> (n - 1 - i) & 1);
    }
    qtest_clock_step(qts, 1 * MS);
}

static void step_idx(QTestState *qts, int idx)
{
    pattern(qts, phase, 4, half[idx & 7]);
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

static QTestState *init(const char *extra)
{
    return qtest_initf("-machine ingenic-t31 "
                       "-global ingenic-t31.soc-variant=t31x "
                       "-device gpio-stepper,id=h,phase0=54,phase1=53,"
                       "phase2=52,phase3=51,range=40,start=10,"
                       "min-pin=77,max-pin=78,limit-margin=2 %s", extra);
}

static void test_half_steps(void)
{
    QTestState *qts = init("");

    g_assert_cmpint(prop(qts, "h", "position"), ==, 10);
    /* the first pattern places the rotor, it is no step */
    step_idx(qts, 3);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 10);
    for (int i = 4; i < 4 + 8; i++) {
        step_idx(qts, i);
    }
    g_assert_cmpint(prop(qts, "h", "position"), ==, 18);
    for (int i = 10; i > 10 - 5; i--) {
        step_idx(qts, i);
    }
    g_assert_cmpint(prop(qts, "h", "position"), ==, 13);

    /* coils off holds the rotor; the same pattern again is no move */
    pattern(qts, phase, 4, 0);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 13);
    step_idx(qts, 6);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 13);
    g_assert_cmpint(prop(qts, "h", "glitches"), ==, 0);

    /* full steps: two indexes at a time */
    step_idx(qts, 4);
    step_idx(qts, 2);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 9);
    qtest_quit(qts);
}

static void test_stops_and_switches(void)
{
    QTestState *qts = init("");

    /* start 10, margin 2, range 40: neither switch is closed */
    g_assert_cmpint(pin_in(qts, PC(13)), ==, 1);
    g_assert_cmpint(pin_in(qts, PC(14)), ==, 1);

    step_idx(qts, 0);
    for (int i = 1; i <= 40; i++) {
        step_idx(qts, i);
    }
    /* 30 steps took it to 40, the other ten ran into the stop */
    g_assert_cmpint(prop(qts, "h", "position"), ==, 40);
    g_assert_cmpint(prop(qts, "h", "stalls"), ==, 10);
    g_assert_cmpint(pin_in(qts, PC(14)), ==, 0);
    g_assert_cmpint(pin_in(qts, PC(13)), ==, 1);

    /* the stalled phase is the rotor's again once it turns back */
    step_idx(qts, 39);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 39);
    step_idx(qts, 38);
    step_idx(qts, 37);
    g_assert_cmpint(pin_in(qts, PC(14)), ==, 1);

    for (int i = 36; i >= 0; i--) {
        step_idx(qts, i);
    }
    g_assert_cmpint(prop(qts, "h", "position"), ==, 0);
    g_assert_cmpint(prop(qts, "h", "stalls"), ==, 10 + 0);
    g_assert_cmpint(pin_in(qts, PC(13)), ==, 0);

    /* the test places the rotor: a switch follows */
    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': '/machine/peripheral/h', "
                             "'property': 'position', 'value': 20}}");
    g_assert_cmpint(pin_in(qts, PC(13)), ==, 1);
    qtest_quit(qts);
}

static void test_glitch(void)
{
    QTestState *qts = init("");

    step_idx(qts, 0);
    /* three half steps at once, and a pattern off the table */
    step_idx(qts, 3);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 10);
    g_assert_cmpint(prop(qts, "h", "glitches"), ==, 1);
    pattern(qts, phase, 4, 0xa);
    g_assert_cmpint(prop(qts, "h", "glitches"), ==, 2);
    g_assert_cmpint(prop(qts, "h", "position"), ==, 10);
    qtest_quit(qts);
}

static void test_quadrature(void)
{
    static const int pins[2] = { PC(0), PC(1) };
    static const uint8_t gray[4] = { 0x2, 0x3, 0x1, 0x0 };
    QTestState *qts = qtest_init("-machine ingenic-t31 "
                                 "-global ingenic-t31.soc-variant=t31x "
                                 "-device gpio-stepper,id=q,phase0=64,"
                                 "phase1=65,range=100,start=50");

    pattern(qts, pins, 2, gray[0]);
    for (int i = 1; i <= 6; i++) {
        pattern(qts, pins, 2, gray[i & 3]);
    }
    g_assert_cmpint(prop(qts, "q", "position"), ==, 56);
    pattern(qts, pins, 2, gray[1]);
    g_assert_cmpint(prop(qts, "q", "position"), ==, 55);
    qtest_quit(qts);
}

static void test_bad_pins(void)
{
    if (g_test_subprocess()) {
        qtest_init("-machine ingenic-t31 "
                   "-device gpio-stepper,phase0=3,phase1=3");
        return;
    }
    g_test_trap_subprocess(NULL, 0, 0);
    g_test_trap_assert_failed();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("gpio-stepper/half-steps", test_half_steps);
    qtest_add_func("gpio-stepper/stops-and-switches", test_stops_and_switches);
    qtest_add_func("gpio-stepper/glitch", test_glitch);
    qtest_add_func("gpio-stepper/quadrature", test_quadrature);
    qtest_add_func("gpio-stepper/bad-pins", test_bad_pins);
    return g_test_run();
}
