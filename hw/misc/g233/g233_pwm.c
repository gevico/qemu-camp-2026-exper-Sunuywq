/*
 * QEMU model: G233 PWM controller (4 channels)
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MMIO register layout (base 0x10015000):
 *   +0x000 GLB  [r/w1c] bits[3:0] = CHn_EN mirror
 *                           bits[7:4] = CHn_DONE (write 1 to clear)
 *   Channel n (n=0..3) at 0x10 + n*0x10:
 *     +0x00 CTRL   [rw] bit0 EN, bit1 POL
 *     +0x04 PERIOD [rw] period value
 *     +0x08 DUTY   [rw] duty cycle
 *     +0x0C CNT    [r ] running counter
 *
 * Behavior:
 *  - When CHn_CTRL.EN=1, the counter increments once per microsecond.
 *  - When CNT reaches PERIOD, CHn_DONE in GLB is set and CHn_EN is
 *    cleared (one-shot mode — software re-enables to start a new period).
 *  - GLB[3:0] = CHn_EN mirror, readable as soon as the channel is enabled.
 *  - GLB[7:4] = CHn_DONE, sticky until w1c.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/misc/g233.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_G233_PWM "g233-pwm"
typedef struct G233PwmState G233PwmState;
DECLARE_INSTANCE_CHECKER(G233PwmState, G233_PWM, TYPE_G233_PWM)

#define G233_PWM_CHANNELS      4
#define G233_PWM_CHAN_STRIDE   0x10
#define G233_PWM_CH_BASE(n)    (0x10 + (n) * G233_PWM_CHAN_STRIDE)
#define G233_PWM_TICK_NS       1000  /* 1 MHz */
#define G233_PWM_DONE_BIT      0x80  /* internal flag stored in ctrl[7] */

typedef struct G233PwmChan {
    uint32_t ctrl;
    uint32_t period;
    uint32_t duty;
    uint32_t cnt;
    QEMUTimer timer;
} G233PwmChan;

typedef struct G233PwmChanBp {
    G233PwmState *s;
    int n;
} G233PwmChanBp;

struct G233PwmState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;

    G233PwmChan ch[G233_PWM_CHANNELS];
    G233PwmChanBp bp[G233_PWM_CHANNELS];
};

static void g233_pwm_chan_tick(void *opaque);

static void g233_pwm_chan_arm(G233PwmState *s, int n)
{
    G233PwmChan *c = &s->ch[n];
    timer_del(&c->timer);
    if (c->ctrl & 0x01) {
        timer_mod_ns(&c->timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + G233_PWM_TICK_NS);
    }
}

static void g233_pwm_chan_tick(void *opaque)
{
    struct G233PwmChanBp *bp = opaque;
    G233PwmState *s = bp->s;
    G233PwmChan *c = &s->ch[bp->n];

    if (!(c->ctrl & 0x01)) {
        return;
    }
    c->cnt++;
    if (c->period != 0 && c->cnt >= c->period) {
        c->cnt = c->period;
        c->ctrl |= G233_PWM_DONE_BIT;
        c->ctrl &= ~0x01;     /* auto-stop */
        return;
    }
    timer_mod_ns(&c->timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + G233_PWM_TICK_NS);
}

static uint32_t g233_pwm_read_glb(G233PwmState *s)
{
    uint32_t v = 0;
    for (int i = 0; i < G233_PWM_CHANNELS; i++) {
        if (s->ch[i].ctrl & 0x01) {
            v |= (1u << i);
        }
        if (s->ch[i].ctrl & G233_PWM_DONE_BIT) {
            v |= (1u << (4 + i));
        }
    }
    return v;
}

static uint64_t g233_pwm_read(void *opaque, hwaddr offset, unsigned size)
{
    G233PwmState *s = opaque;
    if (offset == 0x0) {
        return g233_pwm_read_glb(s);
    }
    if (offset >= 0x10) {
        unsigned n = (offset - 0x10) / G233_PWM_CHAN_STRIDE;
        unsigned r = (offset - 0x10) % G233_PWM_CHAN_STRIDE;
        if (n < G233_PWM_CHANNELS) {
            switch (r) {
            case 0x00: return s->ch[n].ctrl & ~G233_PWM_DONE_BIT;
            case 0x04: return s->ch[n].period;
            case 0x08: return s->ch[n].duty;
            case 0x0C: return s->ch[n].cnt;
            }
        }
    }
    return 0;
}

static void g233_pwm_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    G233PwmState *s = opaque;
    if (offset == 0x0) {
        /* GLB w1c on DONE bits [7:4]. EN bits [3:0] are read-only. */
        uint32_t v = (uint32_t)value;
        for (int i = 0; i < G233_PWM_CHANNELS; i++) {
            if (v & (1u << (4 + i))) {
                s->ch[i].ctrl &= ~G233_PWM_DONE_BIT;
            }
        }
        return;
    }
    if (offset >= 0x10) {
        unsigned n = (offset - 0x10) / G233_PWM_CHAN_STRIDE;
        unsigned r = (offset - 0x10) % G233_PWM_CHAN_STRIDE;
        if (n < G233_PWM_CHANNELS) {
            G233PwmChan *c = &s->ch[n];
            switch (r) {
            case 0x00: {
                uint32_t old_en = c->ctrl & 0x01;
                uint32_t new_ctrl = (uint32_t)value & ~G233_PWM_DONE_BIT;
                c->ctrl = (c->ctrl & G233_PWM_DONE_BIT) | new_ctrl;
                if (!old_en && (new_ctrl & 0x01)) {
                    c->cnt = 0;
                }
                g233_pwm_chan_arm(s, n);
                break;
            }
            case 0x04:
                c->period = (uint32_t)value;
                break;
            case 0x08:
                c->duty = (uint32_t)value;
                break;
            case 0x0C:
                /* CNT is read-only. */
                break;
            }
        }
    }
}

static const MemoryRegionOps g233_pwm_ops = {
    .read = g233_pwm_read,
    .write = g233_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_pwm_reset(DeviceState *dev)
{
    G233PwmState *s = G233_PWM(dev);
    for (int i = 0; i < G233_PWM_CHANNELS; i++) {
        timer_del(&s->ch[i].timer);
        s->ch[i].ctrl = 0;
        s->ch[i].period = 0;
        s->ch[i].duty = 0;
        s->ch[i].cnt = 0;
    }
}

static void g233_pwm_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233PwmState *s = G233_PWM(dev);
    int i;

    memory_region_init_io(&s->mmio, OBJECT(s), &g233_pwm_ops, s,
                          TYPE_G233_PWM,
                          0x10 + G233_PWM_CHANNELS * G233_PWM_CHAN_STRIDE);
    sysbus_init_mmio(sbd, &s->mmio);
    for (i = 0; i < G233_PWM_CHANNELS; i++) {
        s->bp[i].s = s;
        s->bp[i].n = i;
        timer_init_ns(&s->ch[i].timer, QEMU_CLOCK_VIRTUAL,
                      g233_pwm_chan_tick, &s->bp[i]);
    }
}

static const VMStateDescription g233_pwm_vmstate = {
    .name = TYPE_G233_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_END_OF_LIST(),
    },
};

static void g233_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = g233_pwm_realize;
    device_class_set_legacy_reset(dc, g233_pwm_reset);
    dc->vmsd = &g233_pwm_vmstate;
}

static const TypeInfo g233_pwm_info = {
    .name           = TYPE_G233_PWM,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233PwmState),
    .class_init     = g233_pwm_class_init,
};

static void g233_pwm_register_types(void)
{
    type_register_static(&g233_pwm_info);
}
type_init(g233_pwm_register_types)

DeviceState *g233_pwm_create(hwaddr base)
{
    DeviceState *dev = qdev_new(TYPE_G233_PWM);
    SysBusDevice *s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, base);
    return dev;
}
