/*
 * Ingenic SoCs: an access that never completes
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/cpu.h"
#include "system/cpus.h"
#include "hw/misc/ingenic-bus-hang.h"

void ingenic_bus_hang(const char *who, hwaddr addr, bool write)
{
    CPUState *cpu;

    qemu_log_mask(LOG_GUEST_ERROR, "%s: %s of 0x%" HWADDR_PRIx
                  " hangs the bus; only a reset recovers\n",
                  who, write ? "write" : "read", addr);
    CPU_FOREACH(cpu) {
        cpu->stop = true;
        qemu_cpu_kick(cpu);
    }
    cpu_stop_current();
}

bool ingenic_clock_over(const char *who, const char *block, uint64_t hz,
                        uint64_t max_hz, bool hang, bool *logged,
                        hwaddr addr, bool write)
{
    if (!hz || hz <= max_hz) {
        return false;
    }
    if (!*logged) {
        *logged = true;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: %s of 0x%" HWADDR_PRIx " with the %s clock at "
                      "%" PRIu64 " kHz, above the %" PRIu64 " kHz the part "
                      "takes%s\n", who, write ? "write" : "read", addr,
                      block, hz / 1000, max_hz / 1000,
                      hang ? "; hangs the bus" : "");
    }
    if (hang) {
        ingenic_bus_hang(who, addr, write);
    }
    return hang;
}
