/*
 * QEMU model: G233 GPIO controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MMIO register layout (base 0x10012000, 32 pins):
 *   +0x00 DIR  [rw] 1=output, 0=input
 *   +0x04 OUT  [rw] output data
 *   +0x08 IN   [r ] input = OUT where DIR=output, else 0
 *   +0x0C IE   [rw] interrupt enable mask
 *   +0x10 IS   [w1c] interrupt status (set on edge event or level match)
 *   +0x14 TRIG [rw] 0=edge, 1=level
 *   +0x18 POL  [rw] edge: 0=falling, 1=rising; level: 0=low, 1=high
 *
 * Edge mode: set IS bit on a 0→1 transition when POL=1, or on 1→0
 *            when POL=0. The bit stays set until software writes 1
 *            (w1c) to clear it.
 * Level mode: IS bit = (OUT xor POL) is held while the condition holds.
 *             When the condition is no longer met, the bit clears
 *             automatically. Software can still w1c to acknowledge.
 *
 * IN reflects OUT for output pins. For input pins IN is always 0
 * (no external pin inputs in this model).
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/misc/g233.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_G233_GPIO "g233-gpio"
typedef struct G233GpioState G233GpioState;
DECLARE_INSTANCE_CHECKER(G233GpioState, G233_GPIO, TYPE_G233_GPIO)

struct G233GpioState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;

    uint32_t dir;   /* 1=output */
    uint32_t out;   /* output data */
    uint32_t ie;    /* interrupt enable */
    uint32_t is;    /* interrupt status (w1c) */
    uint32_t trig;  /* 0=edge, 1=level */
    uint32_t pol;   /* polarity */
    uint32_t prev_out; /* for edge detection */
};

static void g233_gpio_update(G233GpioState *s)
{
    /* Compute the in-progress level-driven IS mask, but do not yet
     * commit it — level mode is dynamic, so the IS bits reflect the
     * current OUT/POL state every read. We do need to keep previously
     * set edge IS bits in `is` until w1c, so merging happens in read(). */
    if (s->trig) {
        /* Level mode: IS = (out == pol) for output pins (DIR=1) */
        uint32_t active = s->dir & (s->pol ? s->out : ~s->out);
        s->is = (s->is & ~s->dir) | active;
    }
    /* Edge bits stay in s->is until cleared by software. */

    qemu_set_irq(s->irq, (s->is & s->ie) ? 1 : 0);
}

static uint32_t g233_gpio_compute_in(G233GpioState *s)
{
    /* Input is 0 for input pins; OUT where DIR=1. */
    return s->out & s->dir;
}

static uint64_t g233_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    G233GpioState *s = opaque;
    switch (offset) {
    case 0x00: return s->dir;
    case 0x04: return s->out;
    case 0x08: return g233_gpio_compute_in(s);
    case 0x0C: return s->ie;
    case 0x10: return s->is;
    case 0x14: return s->trig;
    case 0x18: return s->pol;
    default:   return 0;
    }
}

static void g233_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                            unsigned size)
{
    G233GpioState *s = opaque;
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case 0x00:
        s->dir = v;
        g233_gpio_update(s);
        break;
    case 0x04: {
        uint32_t old_out = s->prev_out;
        s->out = v;
        if (!s->trig) {
            /* Edge mode: detect rising (POL=1) or falling (POL=0) on
             * output pins (DIR=1). */
            uint32_t mask = s->dir;
            uint32_t rising = (s->pol ? (~old_out & v) : 0) & mask;
            uint32_t falling = (s->pol ? 0 : (old_out & ~v)) & mask;
            s->is |= (rising | falling) & s->ie;
        }
        s->prev_out = v;
        g233_gpio_update(s);
        break;
    }
    case 0x0C:
        s->ie = v;
        g233_gpio_update(s);
        break;
    case 0x10:
        /* w1c */
        s->is &= ~v;
        g233_gpio_update(s);
        break;
    case 0x14:
        s->trig = v;
        g233_gpio_update(s);
        break;
    case 0x18:
        s->pol = v;
        g233_gpio_update(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps g233_gpio_ops = {
    .read = g233_gpio_read,
    .write = g233_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_gpio_reset(DeviceState *dev)
{
    G233GpioState *s = G233_GPIO(dev);
    s->dir = s->out = s->ie = s->is = s->trig = s->pol = 0;
    s->prev_out = 0;
    qemu_set_irq(s->irq, 0);
}

static void g233_gpio_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233GpioState *s = G233_GPIO(dev);

    sysbus_init_irq(sbd, &s->irq);
    memory_region_init_io(&s->mmio, OBJECT(s), &g233_gpio_ops, s,
                          TYPE_G233_GPIO, 0x1C);
    sysbus_init_mmio(sbd, &s->mmio);
}

static const VMStateDescription g233_gpio_vmstate = {
    .name = TYPE_G233_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dir, G233GpioState),
        VMSTATE_UINT32(out, G233GpioState),
        VMSTATE_UINT32(ie,  G233GpioState),
        VMSTATE_UINT32(is,  G233GpioState),
        VMSTATE_UINT32(trig,G233GpioState),
        VMSTATE_UINT32(pol, G233GpioState),
        VMSTATE_UINT32(prev_out, G233GpioState),
        VMSTATE_END_OF_LIST(),
    },
};

static void g233_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = g233_gpio_realize;
    device_class_set_legacy_reset(dc, g233_gpio_reset);
    dc->vmsd = &g233_gpio_vmstate;
}

static const TypeInfo g233_gpio_info = {
    .name           = TYPE_G233_GPIO,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233GpioState),
    .class_init     = g233_gpio_class_init,
};

static void g233_gpio_register_types(void)
{
    type_register_static(&g233_gpio_info);
}
type_init(g233_gpio_register_types)

DeviceState *g233_gpio_create(hwaddr base, qemu_irq irq)
{
    DeviceState *dev = qdev_new(TYPE_G233_GPIO);
    SysBusDevice *s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, base);
    sysbus_connect_irq(s, 0, irq);
    return dev;
}
