/*
 * QEMU model: G233 Watchdog Timer
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MMIO register layout (base 0x10010000):
 *   +0x00 CTRL  [rw] bit0 EN, bit1 INTEN
 *   +0x04 LOAD  [rw] reload value
 *   +0x08 VAL   [r ] current counter (counts down at 1 MHz)
 *   +0x0C SR    [w1c] bit0 TIMEOUT
 *   +0x10 KEY   [w ] 0x5A5A5A5A=feed (reload VAL from LOAD)
 *                    0x1ACCE551=lock (refuse to clear EN)
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/g233.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_G233_WDT "g233-wdt"
typedef struct G233WdtState G233WdtState;
DECLARE_INSTANCE_CHECKER(G233WdtState, G233_WDT, TYPE_G233_WDT)

#define G233_WDT_CTRL_EN     (1u << 0)
#define G233_WDT_CTRL_INTEN  (1u << 1)
#define G233_WDT_SR_TIMEOUT  (1u << 0)
#define G233_WDT_KEY_FEED    0x5A5A5A5A
#define G233_WDT_KEY_LOCK    0x1ACCE551

/* 1 MHz tick: counter decrements once every 1000 ns. */
#define G233_WDT_TICK_NS     1000

struct G233WdtState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t load;
    uint32_t val;
    uint32_t sr;
    bool locked;

    QEMUTimer tick_timer;
};

static void g233_wdt_update_irq(G233WdtState *s)
{
    bool raise = (s->sr & G233_WDT_SR_TIMEOUT) &&
                 (s->ctrl & G233_WDT_CTRL_INTEN);
    qemu_set_irq(s->irq, raise);
}

static void g233_wdt_reset_timer(G233WdtState *s)
{
    timer_del(&s->tick_timer);
    if (s->ctrl & G233_WDT_CTRL_EN) {
        /* Run the timer at the next ns-aligned tick boundary so that
         * qtest_clock_step(ns) deterministically advances the counter. */
        timer_mod_ns(&s->tick_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + G233_WDT_TICK_NS);
    }
}

static void g233_wdt_tick(void *opaque)
{
    G233WdtState *s = opaque;

    if (!(s->ctrl & G233_WDT_CTRL_EN)) {
        return;
    }
    if (s->val > 0) {
        s->val--;
    }
    if (s->val == 0) {
        s->sr |= G233_WDT_SR_TIMEOUT;
        g233_wdt_update_irq(s);
        /* Auto-stop: clear EN so a feed re-arms cleanly. */
        s->ctrl &= ~G233_WDT_CTRL_EN;
        return;
    }
    timer_mod_ns(&s->tick_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + G233_WDT_TICK_NS);
}

static uint64_t g233_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    G233WdtState *s = opaque;
    switch (offset) {
    case 0x00: return s->ctrl;
    case 0x04: return s->load;
    case 0x08: return s->val;
    case 0x0C: return s->sr;
    case 0x10: return 0; /* write-only */
    default:   return 0;
    }
}

static void g233_wdt_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    G233WdtState *s = opaque;
    switch (offset) {
    case 0x00: {
        uint32_t new_ctrl = (uint32_t)value;
        if (s->locked && (s->ctrl & G233_WDT_CTRL_EN) &&
            !(new_ctrl & G233_WDT_CTRL_EN)) {
            /* Locked: ignore attempts to clear EN. */
            new_ctrl |= G233_WDT_CTRL_EN;
        }
        s->ctrl = new_ctrl;
        if (s->ctrl & G233_WDT_CTRL_EN) {
            s->val = s->load ? s->load : 0xFFFFFFFF;
        }
        g233_wdt_update_irq(s);
        g233_wdt_reset_timer(s);
        break;
    }
    case 0x04:
        s->load = (uint32_t)value;
        if (s->ctrl & G233_WDT_CTRL_EN) {
            s->val = s->load ? s->load : 0xFFFFFFFF;
        }
        break;
    case 0x08:
        /* VAL is read-only — writes ignored. */
        break;
    case 0x0C:
        s->sr &= ~((uint32_t)value & G233_WDT_SR_TIMEOUT);
        g233_wdt_update_irq(s);
        break;
    case 0x10:
        if ((uint32_t)value == G233_WDT_KEY_FEED) {
            s->val = s->load ? s->load : 0xFFFFFFFF;
            s->ctrl |= G233_WDT_CTRL_EN;
            g233_wdt_reset_timer(s);
        } else if ((uint32_t)value == G233_WDT_KEY_LOCK) {
            s->locked = true;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps g233_wdt_ops = {
    .read = g233_wdt_read,
    .write = g233_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_wdt_reset(DeviceState *dev)
{
    G233WdtState *s = G233_WDT(dev);

    timer_del(&s->tick_timer);
    s->ctrl = 0;
    s->load = 0xFFFFFFFF;
    s->val  = 0xFFFFFFFF;
    s->sr   = 0;
    s->locked = false;
    qemu_set_irq(s->irq, 0);
}

static void g233_wdt_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233WdtState *s = G233_WDT(dev);

    sysbus_init_irq(sbd, &s->irq);
    memory_region_init_io(&s->mmio, OBJECT(s), &g233_wdt_ops, s,
                          TYPE_G233_WDT, 0x14);
    sysbus_init_mmio(sbd, &s->mmio);
    timer_init_ns(&s->tick_timer, QEMU_CLOCK_VIRTUAL, g233_wdt_tick, s);
}

static const VMStateDescription g233_wdt_vmstate = {
    .name = TYPE_G233_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, G233WdtState),
        VMSTATE_UINT32(load, G233WdtState),
        VMSTATE_UINT32(val,  G233WdtState),
        VMSTATE_UINT32(sr,   G233WdtState),
        VMSTATE_BOOL(locked,  G233WdtState),
        VMSTATE_TIMER(tick_timer, G233WdtState),
        VMSTATE_END_OF_LIST(),
    },
};

static void g233_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = g233_wdt_realize;
    device_class_set_legacy_reset(dc, g233_wdt_reset);
    dc->vmsd = &g233_wdt_vmstate;
}

static const TypeInfo g233_wdt_info = {
    .name           = TYPE_G233_WDT,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233WdtState),
    .class_init     = g233_wdt_class_init,
};

static void g233_wdt_register_types(void)
{
    type_register_static(&g233_wdt_info);
}
type_init(g233_wdt_register_types)

DeviceState *g233_wdt_create(hwaddr base, qemu_irq irq)
{
    DeviceState *dev = qdev_new(TYPE_G233_WDT);
    SysBusDevice *s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, base);
    sysbus_connect_irq(s, 0, irq);
    return dev;
}
