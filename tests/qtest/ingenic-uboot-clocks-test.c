/*
 * QTest testcase for the CPM of a direct Linux boot (-kernel uImage)
 *
 * The dividers and clock sources as thingino's U-Boot leaves them, read
 * back at its prompt on each part (a T31X's: CPU 1392 MHz, L2 696, AHB0 and
 * AHB2 200, PCLK 100, DDR 600, MAC and SFC 50, MSC0 24). A flash boot
 * leaves the CPM to the firmware: its CPCCR stays as the bootrom has it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define CPM             0x10000000
#define CPCCR           0x00
#define CPAPCR          0x10
#define CPMPCR          0x14
#define DDRCDR          0x2c
#define MACCDR          0x54
#define MSC0CDR         0x68
#define SSICDR          0x74
#define CPVPCR          0xe0

typedef struct {
    const char *machine;
    const char *variant;
    uint32_t cpccr, ddrcdr, maccdr, msc0cdr, ssicdr;
    /* in MHz, 0: the PLLs are not the MNOD form the decode below reads */
    unsigned cpu, l2, ahb0, ahb2, pclk, ddr;
} Case;

static const Case cases[] = {
    { "ingenic-t10", "t10l", 0x9a7b5510, 0xa0000002, 0xa0000017, 0x2000000e,
      0xe0000017 },
    { "ingenic-t10", "t10n", 0x9a7b5510, 0xa0000002, 0xa0000017, 0x20000011,
      0xe0000017 },
    { "ingenic-t20", "t20l", 0x9a794410, 0xa0000001, 0x60000013, 0x2000000e,
      0xa000000e },
    { "ingenic-t20", "t20n", 0x9a794410, 0xa0000001, 0x60000013, 0x20000011,
      0xa0000011 },
    { "ingenic-t21", "t21n", 0x9a773310, 0xa0000001, 0x60000023, 0x20000007,
      0x50000010 },
    { "ingenic-t23", "t23n", 0x9a7b5510, 0xa0000001, 0x6000002f, 0x20000018,
      0x50000017, 1188, 594, 200, 200, 100, 600 },
    { "ingenic-t30", "t30n", 0x9a773310, 0xa0000001, 0x60000013, 0x20000012,
      0x50000011 },
    { "ingenic-t30", "t30a", 0x9a7b5510, 0xa0000001, 0x60000017, 0x20000018,
      0x50000017 },
    { "ingenic-t31", "t31l", 0x9a773310, 0xa0000001, 0x60000013, 0x20000014,
      0x50000013, 1008, 504, 250, 250, 125, 500 },
    { "ingenic-t31", "t31n", 0x9a773310, 0xa0000001, 0x60000013, 0x2000001c,
      0x50000013, 1404, 702, 250, 250, 125, 500 },
    { "ingenic-t31", "t31x", 0x9a7b5510, 0xa0000001, 0x60000017, 0x2000001c,
      0x50000017, 1392, 696, 200, 200, 100, 600 },
    { "ingenic-t31", "t31zx", 0x9a7b5510, 0xa0000001, 0x60000017, 0x2000001c,
      0x50000017, 1392, 696, 200, 200, 100, 600 },
};

static char *make_uimage(void)
{
    uint8_t img[64 + 4096] = { 0 };
    char *path;
    int fd = g_file_open_tmp("ingenic-uboot-clocks-XXXXXX", &path, NULL);

    stl_be_p(img, 0x27051956);
    stl_be_p(img + 12, 4096);
    stl_be_p(img + 16, 0x80010000);
    stl_be_p(img + 20, 0x80010000);
    img[28] = 5;
    img[29] = 5;
    img[30] = 2;
    g_assert(write(fd, img, sizeof(img)) == sizeof(img));
    close(fd);
    return path;
}

/* EXTCLK 24 MHz * M / N / OD1 / OD0, in MHz */
static unsigned pll_mhz(uint32_t reg)
{
    unsigned m = reg >> 20 & 0xfff, n = reg >> 14 & 0x3f;
    unsigned od1 = reg >> 11 & 7, od0 = reg >> 8 & 7;

    g_assert(reg & 1);
    return 24 * m / (n * od1 * od0);
}

static void test_direct(const void *data)
{
    const Case *c = data;
    g_autofree char *kernel = make_uimage();
    QTestState *qts = qtest_initf("-machine %s "
                                  "-global ingenic-t31.soc-variant=%s "
                                  "-kernel %s", c->machine, c->variant,
                                  kernel);
    uint32_t cc = qtest_readl(qts, CPM + CPCCR);

    g_assert_cmphex(cc, ==, c->cpccr);
    g_assert_cmphex(qtest_readl(qts, CPM + DDRCDR), ==, c->ddrcdr);
    g_assert_cmphex(qtest_readl(qts, CPM + MACCDR), ==, c->maccdr);
    g_assert_cmphex(qtest_readl(qts, CPM + MSC0CDR), ==, c->msc0cdr);
    g_assert_cmphex(qtest_readl(qts, CPM + SSICDR), ==, c->ssicdr);
    if (c->cpu) {
        unsigned apll = pll_mhz(qtest_readl(qts, CPM + CPAPCR));
        unsigned mpll = pll_mhz(qtest_readl(qts, CPM + CPMPCR));
        unsigned ddr = mpll / ((qtest_readl(qts, CPM + DDRCDR) & 0xf) + 1);
        unsigned pdiv = (cc >> 16 & 0xf) + 1;

        g_assert_cmpuint(apll / ((cc & 0xf) + 1), ==, c->cpu);
        g_assert_cmpuint(apll / ((cc >> 4 & 0xf) + 1), ==, c->l2);
        /* SEL_H0PLL and SEL_H2PLL 2: the MPLL */
        g_assert_cmpuint(cc >> 26 & 3, ==, 2);
        g_assert_cmpuint(cc >> 24 & 3, ==, 2);
        g_assert_cmpuint(mpll / ((cc >> 8 & 0xf) + 1), ==, c->ahb0);
        g_assert_cmpuint(mpll / ((cc >> 12 & 0xf) + 1), ==, c->ahb2);
        g_assert_cmpuint(mpll / pdiv, ==, c->pclk);
        g_assert_cmpuint(ddr, ==, c->ddr);
    }
    qtest_quit(qts);
    unlink(kernel);
}

/* A flash boot leaves the CPCCR to the firmware, as the bootrom has it */
static void test_flash(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    g_assert_cmphex(qtest_readl(qts, CPM + CPCCR), ==, 0x55800000);
    qtest_quit(qts);
}

/* The MAC clock is 50 MHz and the SFC's 50 MHz off the same MPLL */
static void test_t31x_rates(void)
{
    g_autofree char *kernel = make_uimage();
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-t31.soc-variant=t31x "
                                  "-kernel %s", kernel);
    unsigned mpll = pll_mhz(qtest_readl(qts, CPM + CPMPCR));
    unsigned vpll = pll_mhz(qtest_readl(qts, CPM + CPVPCR));
    unsigned apll = pll_mhz(qtest_readl(qts, CPM + CPAPCR));
    uint32_t mac = qtest_readl(qts, CPM + MACCDR);
    uint32_t ssi = qtest_readl(qts, CPM + SSICDR);
    uint32_t msc = qtest_readl(qts, CPM + MSC0CDR);

    g_assert_cmpuint(apll, ==, 1392);
    g_assert_cmpuint(mpll, ==, 1200);
    g_assert_cmpuint(vpll, ==, 1200);
    g_assert_cmpuint(mpll / ((mac & 0xff) + 1), ==, 50);
    g_assert_cmpuint(mpll / ((ssi & 0xff) + 1), ==, 50);
    /* the MSC divides its source (SCLK_A) by twice its field plus one */
    g_assert_cmpuint(apll / (2 * ((msc & 0xff) + 1)), ==, 24);
    qtest_quit(qts);
    unlink(kernel);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *name = g_strdup_printf("ingenic-uboot-clocks/%s",
                                                cases[i].variant);

        qtest_add_data_func(name, &cases[i], test_direct);
    }
    qtest_add_func("ingenic-uboot-clocks/flash-boot", test_flash);
    qtest_add_func("ingenic-uboot-clocks/t31x-rates", test_t31x_rates);
    return g_test_run();
}
