/*
 * QTest testcase for an ISP accessed with its clock above what the part takes
 *
 * A T20X hung the bus at the first ISP register access with CLKGR0 bit 23
 * open and ISPCDR passing the bare MPLL (1 GHz). The limits are the
 * vendors' (manuals, drivers) and thingino's, none measured but the T41's:
 * an access with the divider above one logs once; with the clock-hang
 * option it also hangs the bus. A divider in range, a shut gate and a
 * divider no one has set log nothing.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define ISP         0x13300000
#define CPM_MPLL    0x10000014
#define CPM_CLKGR0  0x10000020
#define CPM_ISPCDR  0x10000080
#define CPM_ISPSCDR 0x10000090
#define GATE        (1u << 23)
#define MPLL        (1u << 30)
#define CE          (1u << 29)
#define SCLK_A      (1u << 30)

typedef struct Part {
    const char *machine;
    const char *isp;            /* the ISP's QOM path */
    uint32_t scaler_cdr;        /* the MSCA's divider, 0 for none */
    unsigned gate;
    bool pllreg;                /* T21, T30: the PLL register's other format */
} Part;

static const Part parts[] = {
    { "ingenic-t10", "/machine/soc/t20-isp", 0, 23 },
    { "ingenic-t20", "/machine/soc/t20-isp", 0, 23 },
    { "ingenic-t30", "/machine/soc/t20-isp", 0, 23, true },
    { "ingenic-t21", "/machine/soc/isp", 0, 23, true },
    { "ingenic-t23", "/machine/soc/isp", 0, 23 },
    { "ingenic-t31", "/machine/soc/isp", 0, 23 },
    { "ingenic-t32", "/machine/soc/isp", CPM_ISPSCDR, 19 },
    { "ingenic-t33", "/machine/soc/isp", CPM_ISPSCDR, 19 },
    { "ingenic-t40", "/machine/soc/isp", 0, 23 },
};

static char *logname;

static QTestState *start(const Part *p, bool hang, bool open)
{
    QTestState *qts;

    g_unlink(logname);
    qts = qtest_initf("-machine %s -d guest_errors -D %s %s", p->machine,
                      logname, hang ? "-global ingenic-isp.clock-hang=on "
                      "-global ingenic-t20-isp.clock-hang=on "
                      "-global ingenic-t10-isp.clock-hang=on "
                      "-global ingenic-t30-isp.clock-hang=on" : "");
    /* a bare machine has the PLLs off: MPLL at 1 GHz as U-Boot leaves it */
    if (!(qtest_readl(qts, CPM_MPLL) & 1)) {
        qtest_writel(qts, CPM_MPLL, p->pllreg ?
                     (124u << 20) | (2 << 14) | (1 << 11) | 1 :
                     (125u << 20) | (3 << 14) | (1 << 11) | (1 << 8) | 1);
    }
    if (open) {
        qtest_writel(qts, CPM_CLKGR0,
                     qtest_readl(qts, CPM_CLKGR0) & ~(1u << p->gate));
    }
    return qts;
}

static unsigned over_logs(void)
{
    gchar *txt = NULL;
    unsigned n = 0;
    const char *at;

    g_assert_true(g_file_get_contents(logname, &txt, NULL, NULL));
    for (at = txt; (at = strstr(at, "kHz the part takes")); at++) {
        n++;
    }
    g_free(txt);
    return n;
}

static bool hung(QTestState *qts, const Part *p)
{
    QDict *r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                         "{'path': %s, 'property': 'bus-hung'}}", p->isp);
    bool v = qdict_get_bool(r, "return");

    qobject_unref(r);
    return v;
}

/* The ISP's clock as a driver sets it: MPLL (1) through CE, divided */
static void set_div(QTestState *qts, uint32_t cdr, unsigned div)
{
    qtest_writel(qts, cdr, MPLL | CE | div);
}

static void test_part(const void *data)
{
    const Part *p = data;
    QTestState *qts;

    /* the bare MPLL: undivided, as the T20X's glue left it */
    qts = start(p, false, true);
    set_div(qts, CPM_ISPCDR, 0);
    qtest_readl(qts, ISP);
    qtest_writel(qts, ISP + 4, 0);
    qtest_readl(qts, ISP + 8);
    g_assert_cmpuint(over_logs(), ==, 1);
    g_assert_false(hung(qts, p));
    qtest_quit(qts);

    /* in range: MPLL over 16 is at most 100 MHz of the part's 1.5 GHz */
    qts = start(p, false, true);
    set_div(qts, CPM_ISPCDR, 15);
    qtest_readl(qts, ISP);
    qtest_writel(qts, ISP + 4, 0);
    g_assert_cmpuint(over_logs(), ==, 0);
    qtest_quit(qts);

    /* shut: the gate stops the clock the access would wait for */
    qts = start(p, false, false);
    qtest_writel(qts, CPM_CLKGR0,
                 qtest_readl(qts, CPM_CLKGR0) | (1u << p->gate));
    set_div(qts, CPM_ISPCDR, 0);
    qtest_readl(qts, ISP);
    g_assert_cmpuint(over_logs(), ==, 0);
    qtest_quit(qts);

    /* with the option the first access hangs the bus */
    qts = start(p, true, true);
    set_div(qts, CPM_ISPCDR, 0);
    qtest_readl(qts, ISP);
    g_assert_cmpuint(over_logs(), ==, 1);
    g_assert_true(hung(qts, p));
    qtest_quit(qts);

    if (p->scaler_cdr) {
        /* the MSCA's own divider, from 0xf0000 */
        qts = start(p, false, true);
        set_div(qts, CPM_ISPCDR, 15);
        set_div(qts, p->scaler_cdr, 0);
        qtest_readl(qts, ISP + 0x10000);
        g_assert_cmpuint(over_logs(), ==, 0);
        qtest_readl(qts, ISP + 0xf0000);
        g_assert_cmpuint(over_logs(), ==, 1);
        qtest_quit(qts);
    }
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    logname = g_strdup_printf("%s/ingenic-isp-clock-%d.log", g_get_tmp_dir(),
                              getpid());
    for (unsigned i = 0; i < ARRAY_SIZE(parts); i++) {
        gchar *path = g_strdup_printf("ingenic/isp-clock/%s", parts[i].machine);

        qtest_add_data_func(path, &parts[i], test_part);
        g_free(path);
    }
    ret = g_test_run();
    g_unlink(logname);
    return ret;
}
