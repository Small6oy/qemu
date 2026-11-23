/*
 * Ingenic SoCs: an access that never completes
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_BUS_HANG_H
#define HW_MISC_INGENIC_BUS_HANG_H

#include "exec/hwaddr.h"

/*
 * A register access on silicon that hangs the bus: no core executes another
 * instruction and nothing but the watchdog's reset, if it runs, brings the
 * SoC back. Here every CPU stops, the timers and devices run on, and a
 * system reset resumes the CPUs.
 */
void ingenic_bus_hang(const char *who, hwaddr addr, bool write);

/*
 * A register access of @block with its clock at @hz, which is above the
 * @max_hz the part is known to run it at (a vendor manual's or driver's
 * rate, not a measured limit; 0 for a clock no one has set). Logs the first
 * such access, once for each @logged; with @hang it also hangs the bus as a
 * T20X's did at 1 GHz. Returns whether it hung.
 */
bool ingenic_clock_over(const char *who, const char *block, uint64_t hz,
                        uint64_t max_hz, bool hang, bool *logged,
                        hwaddr addr, bool write);

#endif /* HW_MISC_INGENIC_BUS_HANG_H */
