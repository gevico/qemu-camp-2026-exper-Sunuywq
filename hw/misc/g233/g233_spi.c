/*
 * QEMU model: G233 SPI master controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * MMIO register layout (base 0x10018000, master mode, 2 CS):
 *   +0x00 CR1  [rw] bit0 SPE, bit2 MSTR, bit5 ERRIE,
 *                  bit6 RXNEIE, bit7 TXEIE
 *   +0x04 CR2  [rw] bits[1:0] active CS index (CS is active low to the
 *                  selected slave; the other CS line is high)
 *   +0x08 SR   [r/w1c] bit0 RXNE, bit1 TXE, bit4 OVERRUN (w1c)
 *   +0x0C DR   [rw] write=tx byte, read=rx byte (read clears RXNE)
 *
 * Two W25X flash chips are attached via the SSI bus:
 *   CS0 → W25X16 (jedec 0xEF3015)
 *   CS1 → W25X32 (jedec 0xEF3016)
 *
 * Transfer: write to DR -> byte goes to the currently-active CS -> slave
 * returns one byte. The byte is captured in the RX register; RXNE is
 * set and an IRQ is raised (RXNEIE or TXEIE). The next DR write will
 * trigger an OVERRUN if RXNE was not cleared by a previous DR read.
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/g233.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_G233_SPI "g233-spi"
typedef struct G233SpiState G233SpiState;
DECLARE_INSTANCE_CHECKER(G233SpiState, G233_SPI, TYPE_G233_SPI)

#define G233_SPI_NUM_CS 2

#define G233_SPI_CR1_SPE     (1u << 0)
#define G233_SPI_CR1_MSTR    (1u << 2)
#define G233_SPI_CR1_ERRIE   (1u << 5)
#define G233_SPI_CR1_RXNEIE  (1u << 6)
#define G233_SPI_CR1_TXEIE   (1u << 7)

#define G233_SPI_SR_RXNE     (1u << 0)
#define G233_SPI_SR_TXE      (1u << 1)
#define G233_SPI_SR_OVERRUN  (1u << 4)

struct G233SpiState {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;

    SSIBus *spi;
    qemu_irq cs_lines[G233_SPI_NUM_CS];

    uint32_t cr1;
    uint32_t cr2;
    uint32_t sr;
    uint8_t  rx_byte;
    bool     rx_full;
};

static void g233_spi_update_irq(G233SpiState *s)
{
    bool raise = false;
    if ((s->sr & G233_SPI_SR_RXNE) && (s->cr1 & G233_SPI_CR1_RXNEIE)) {
        raise = true;
    }
    if ((s->sr & G233_SPI_SR_TXE) && (s->cr1 & G233_SPI_CR1_TXEIE)) {
        raise = true;
    }
    if ((s->sr & G233_SPI_SR_OVERRUN) && (s->cr1 & G233_SPI_CR1_ERRIE)) {
        raise = true;
    }
    qemu_set_irq(s->irq, raise ? 1 : 0);
}

static void g233_spi_apply_cs(G233SpiState *s)
{
    /* CR2[1:0] is the active-low CS index. All others are high. */
    int active = s->cr2 & 0x3;
    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        qemu_set_irq(s->cs_lines[i], (i == active) ? 0 : 1);
    }
}

static uint64_t g233_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    G233SpiState *s = opaque;
    switch (offset) {
    case 0x00: return s->cr1;
    case 0x04: return s->cr2;
    case 0x08: return s->sr;
    case 0x0C:
        if (s->rx_full) {
            s->rx_full = false;
            s->sr &= ~G233_SPI_SR_RXNE;
            g233_spi_update_irq(s);
            return s->rx_byte;
        }
        return 0;
    default:   return 0;
    }
}

static void g233_spi_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    G233SpiState *s = opaque;
    switch (offset) {
    case 0x00: {
        uint32_t old = s->cr1;
        s->cr1 = (uint32_t)value;
        /* Re-evaluate IRQ in case enable bits changed. */
        if (old != s->cr1) {
            g233_spi_update_irq(s);
        }
        break;
    }
    case 0x04:
        s->cr2 = (uint32_t)value;
        g233_spi_apply_cs(s);
        break;
    case 0x08:
        /* w1c */
        s->sr &= ~((uint32_t)value & G233_SPI_SR_OVERRUN);
        g233_spi_update_irq(s);
        break;
    case 0x0C: {
        if (!(s->cr1 & G233_SPI_CR1_SPE) ||
            !(s->cr1 & G233_SPI_CR1_MSTR)) {
            /* Not enabled: just buffer the byte. */
            break;
        }
        uint8_t tx = (uint8_t)value;
        uint8_t rx = ssi_transfer(s->spi, tx);
        if (s->rx_full) {
            s->sr |= G233_SPI_SR_OVERRUN;
        } else {
            s->rx_byte = rx;
            s->rx_full = true;
            s->sr |= G233_SPI_SR_RXNE;
        }
        /* TXE = 1 (always ready to take another byte) */
        s->sr |= G233_SPI_SR_TXE;
        g233_spi_update_irq(s);
        break;
    }
    default:
        break;
    }
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_spi_reset(DeviceState *dev)
{
    G233SpiState *s = G233_SPI(dev);
    s->cr1 = 0;
    s->cr2 = 0;
    s->sr  = G233_SPI_SR_TXE;   /* TXE defaults to 1 */
    s->rx_byte = 0;
    s->rx_full = false;
    qemu_set_irq(s->irq, 0);
    /* Deassert all CS lines. */
    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        qemu_set_irq(s->cs_lines[i], 1);
    }
}

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233SpiState *s = G233_SPI(dev);
    DeviceState *flash0, *flash1;

    s->spi = ssi_create_bus(dev, "spi");
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), s->cs_lines, SSI_GPIO_CS,
                             G233_SPI_NUM_CS);

    memory_region_init_io(&s->mmio, OBJECT(s), &g233_spi_ops, s,
                          TYPE_G233_SPI, 0x10);
    sysbus_init_mmio(sbd, &s->mmio);

    /* Attach two flash chips to the shared SSI bus with distinct CS indices. */
    flash0 = qdev_new("w25x16");
    qdev_prop_set_uint8(flash0, "cs", 0);
    qdev_realize_and_unref(flash0, BUS(s->spi), &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(s), SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash0, SSI_GPIO_CS, 0));
    flash1 = qdev_new("w25x32");
    qdev_prop_set_uint8(flash1, "cs", 1);
    qdev_realize_and_unref(flash1, BUS(s->spi), &error_fatal);
    qdev_connect_gpio_out_named(DEVICE(s), SSI_GPIO_CS, 1,
                                qdev_get_gpio_in_named(flash1, SSI_GPIO_CS, 0));
}

static const VMStateDescription g233_spi_vmstate = {
    .name = TYPE_G233_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr1, G233SpiState),
        VMSTATE_UINT32(cr2, G233SpiState),
        VMSTATE_UINT32(sr,  G233SpiState),
        VMSTATE_UINT8(rx_byte, G233SpiState),
        VMSTATE_BOOL(rx_full, G233SpiState),
        VMSTATE_END_OF_LIST(),
    },
};

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = g233_spi_realize;
    device_class_set_legacy_reset(dc, g233_spi_reset);
    dc->vmsd = &g233_spi_vmstate;
}

static const TypeInfo g233_spi_info = {
    .name           = TYPE_G233_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233SpiState),
    .class_init     = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}
type_init(g233_spi_register_types)

DeviceState *g233_spi_create(hwaddr base, qemu_irq irq)
{
    DeviceState *dev = qdev_new(TYPE_G233_SPI);
    SysBusDevice *s = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(s, &error_fatal);
    sysbus_mmio_map(s, 0, base);
    sysbus_connect_irq(s, 0, irq);
    return dev;
}
