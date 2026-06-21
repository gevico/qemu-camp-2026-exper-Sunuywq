/*
 * QEMU G233 on-chip peripherals
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Header for the WDT / GPIO / PWM / SPI controllers used by the g233
 * RISC-V SoC. Each device is a single MMIO region with a single PLIC IRQ
 * output (PWM has none). They are wired up by hw/riscv/g233.c.
 */

#ifndef HW_MISC_G233_H
#define HW_MISC_G233_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

DeviceState *g233_wdt_create(hwaddr base, qemu_irq irq);
DeviceState *g233_gpio_create(hwaddr base, qemu_irq irq);
DeviceState *g233_pwm_create(hwaddr base);
DeviceState *g233_spi_create(hwaddr base, qemu_irq irq);

#endif /* HW_MISC_G233_H */
