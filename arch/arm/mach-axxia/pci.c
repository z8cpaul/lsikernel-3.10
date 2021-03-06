/*
 * arch/arm/mach-axxia/pci.c
 *
 * PCIe support for AXM55xx.
 *
 * Copyright (C) 2013 LSI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/msi.h>
#include <linux/kernel_stat.h>
#include <asm/sizes.h>
#include <asm/mach/pci.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm-generic/errno-base.h>
#include <mach/pci.h>

#define PCIE_CONFIG              (0x1000)
#define PCIE_STATUS              (0x1004)
#define PCIE_CORE_DEBUG          (0x1008)
#define PCIE_LOOPBACK_FAIL       (0x100C)
#define PCIE_MPAGE_U(n)          (0x1010 + (n * 8)) /* n = 0..7 */
#define PCIE_MPAGE_L(n)          (0x1014 + (n * 8)) /* n = 0..7 */
#define PCIE_TPAGE_BAR0(n)       (0x1050 + (n * 4)) /* n = 0..7 */
#define     PCIE_TPAGE_32        (0<<31) /* AXI 32-bit access */
#define     PCIE_TPAGE_128       (1<<31) /* AXI 128-bit access */
#define PCIE_TPAGE_BAR1(n)       (0x1070 + (n * 4)) /* n = 0..7 */
#define PCIE_TPAGE_BAR2(n)       (0x1090 + (n * 4)) /* n = 0..7 */
#define PCIE_MSG_IN_FIFO         (0x10B0)
#define PCIE_MSG_IN_FIFO_STATUS  (0x10B4)
#define PCIE_MSG_OUT             (0x10B8)
#define PCIE_TRN_ORDER_STATUS    (0x10BC)
#define PCIE_INT0_STATUS         (0x10C0)
#define PCIE_INT0_ENABLE         (0x10C4)
#define PCIE_INT0_FORCE          (0x10C8)
#define    INT0_MSI              0x80000000U
#define    INT0_INT_ASSERTED     0x08000000U
#define    INT0_INT_DEASSERTED   0x04000000U
#define    INT0_ERROR            0x73FFFFABU
#define PCIE_PHY_STATUS0         (0x10CC)
#define PCIE_PHY_STATUS1         (0x10D0)
#define PCIE_PHY_CONTROL0        (0x10D4)
#define PCIE_PHY_CONTROL1        (0x10D8)
#define PCIE_PHY_CONTROL2        (0x10DC)
#define PCIE_RESERVED_E0         (0x10E0)
#define PCIE_RESERVED_E4         (0x10E4)
#define PCIE_RESERVED_E8         (0x10E8)
#define PCIE_AXI_MASTER_WR       (0x10EC)
#define PCIE_LINK_STATUS         (0x117C)
#define PCIE_EP_BAR2_CFG         (0x1184)
#define PCIE_AXI_MSI_ADDR        (0x1190)
#define PCIE_INT1_STATUS         (0x11C4)
#define PCIE_INT1_ENABLE         (0x11C8)
#define PCIE_INT1_FORCE          (0x11CC)
#define PCIE_RC_BAR0_SIZE        (0x11F4)
#define PCIE_MSI0_STATUS         (0x1230)
#define PCIE_MSI0_ENABLE         (0x1234)
#define PCIE_MSI0_FORCE          (0x1238)
#define PCIE_MSI1_STATUS(_grp)   (0x123C+(_grp)*12)
#define PCIE_MSI1_ENABLE(_grp)   (0x1240+(_grp)*12)
#define PCIE_MSI1_FORCE(_grp)    (0x1244+(_grp)*12)

/* Every MPAGE register maps 128MB in the AXI memory range */
#define MPAGE_SIZE               (128U<<20)

/* We have 7 MPAGE registers available for outbound window (one reserved for
 * mapping PCI configuration space).
 */
#define MAX_OUTBOUND_SIZE	 (7 * MPAGE_SIZE)

/* Number of IRQs allocated to MSI */
#define NUM_MSI_IRQ (NR_IRQS - AXXIA_MSI_FIRST)

/* Bitmap for allocated MSIs */
static DECLARE_BITMAP(msi_irq_in_use, NUM_MSI_IRQ);

static const struct resource pcie_outbound_default[] = {
	[0] = {
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_MEM
	}
};

struct axxia_pciex_port {
	char                name[16];
	unsigned int	    index;
	u8                  root_bus_nr;
	bool                link_up;
	int                 irq[17]; /* 1 legacy, 16 MSI */
	void __iomem	    *regs;
	void __iomem	    *cfg_data;
	u32                 last_mpage;
	int                 endpoint;
	struct device_node  *node;
	struct resource	    utl_regs;
	struct resource	    cfg_space;
	/* Outbound PCI base address */
	u64                 pci_addr;
	/* Outbound range in (physical) CPU addresses */
	struct resource	    outbound;
	/* Inbound PCI base address */
	u64                 pci_bar;
	/* Inbound range in (physical) CPU addresses */
	struct resource	    inbound;
	/* Virtual and physical (CPU space) address for MSI table */
	void                *msi_virt;
	dma_addr_t          msi_phys;
	/* PCI memory space address for MSI table */
	u32                 msi_pci_addr;
};

#define PCIE_MAX_PORTS 2
static struct axxia_pciex_port *axxia_pciex_ports;


static void
fixup_axxia_pci_bridge(struct pci_dev *dev)
{
	/* if we aren't a PCIe don't bother */
	if (!pci_find_capability(dev, PCI_CAP_ID_EXP))
		return;

	/* Set the class appropriately for a bridge device */
	dev_info(&dev->dev,
		 "Fixup PCI Class to PCI_CLASS_BRIDGE_HOST for %04x:%04x\n",
		 dev->vendor, dev->device);
	dev->class = PCI_CLASS_BRIDGE_HOST << 8;
	/* Make the bridge transparent */
	dev->transparent = 1;
}

DECLARE_PCI_FIXUP_HEADER(0x1000, 0x5101, fixup_axxia_pci_bridge);
DECLARE_PCI_FIXUP_HEADER(0x1000, 0x5108, fixup_axxia_pci_bridge);
DECLARE_PCI_FIXUP_HEADER(0x1000, 0x5120, fixup_axxia_pci_bridge);

/* Convert to Bus# to PCIe port# */
static struct axxia_pciex_port *bus_to_port(struct pci_bus *bus)
{
	return axxia_pciex_ports + pci_domain_nr(bus);
}

/*
 * Validate the Bus#/Device#/Function#
 */
static int
axxia_pciex_validate_bdf(struct pci_bus *bus, unsigned int devfn)
{
	struct axxia_pciex_port *port;

	port = bus_to_port(bus);

	/* Endpoint can not generate upstream(remote) config cycles */
	if (port->endpoint)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (((!((PCI_FUNC(devfn) == 0) && (PCI_SLOT(devfn) == 0)))
		&& (bus->number == port->root_bus_nr))
		|| (!(PCI_SLOT(devfn) == 0)
		&& (bus->number == port->root_bus_nr+1))) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	return 0;
}

/*
 * Return the configuration access base address
 */
static void __iomem *
axxia_pciex_get_config_base(struct axxia_pciex_port *port,
			    struct pci_bus *bus,
			    unsigned int devfn)
{
	int relbus, dev, fn;
	unsigned mpage;

	if (bus->number == port->root_bus_nr)
		return port->regs;

	relbus = bus->number - (port->root_bus_nr + 1);
	dev    = PCI_SLOT(devfn);
	fn     = PCI_FUNC(devfn);

	if (dev > 31)
		return NULL;

	/* Build the mpage register (MPAGE[4]=1 for cfg access) */
	mpage = (fn << 19) | (bus->number << 11) | (dev << 6) | (1<<4);

	/* Primary bus */
	if (relbus && (bus->number != port->root_bus_nr))
		mpage |= 1<<5;

	if (mpage != port->last_mpage) {
		writel(0,     port->regs + PCIE_MPAGE_U(7));
		writel(mpage, port->regs + PCIE_MPAGE_L(7));
		port->last_mpage = mpage;
	}

	return port->cfg_data;
}

/*
 * Read PCI config space
 */
static int
arm_pciex_axxia_read_config(struct pci_bus *bus,
			    unsigned int devfn,
			    int offset,
			    int len,
			    u32 *val)
{
	struct axxia_pciex_port *port = bus_to_port(bus);
	void __iomem *addr;
	u32 bus_addr;
	u32 val32;
	int bo = offset & 0x3;
	int rc = PCIBIOS_SUCCESSFUL;
	u32 bus_addr1;

	if (axxia_pciex_validate_bdf(bus, devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = axxia_pciex_get_config_base(port, bus, devfn);

	if (!addr) {
		*val = 0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/*
	 * addressing is different for local config access vs.
	 * access through the mapped config space.
	 */
	if (bus->number == 0) {
		int wo = offset & 0xfffffffc;
		bus_addr = (u32)addr + wo;
		bus_addr1 = bus_addr;
	} else {
		/*
		 * mapped config space only supports 32-bit access
		 *
		 *  AXI address [3:0] is not used at all.
		 *  AXI address[9:4] becomes register number.
		 *  AXI address[13:10] becomes Ext. register number
		 *  AXI address[17:14] becomes 1st DWBE for configuration
		 *  read only.
		 *  AXI address[29:27] is used to select one of 8 Mpage
		 *  registers.
		 */
		bus_addr = (u32) addr + (offset << 2);
		bus_addr1 = bus_addr;

		switch (len) {
		case 1:
			bus_addr |=  ((1 << bo)) << 14;
			break;
		case 2:
			bus_addr |=  ((3 << bo)) << 14;
			break;
		default:
			bus_addr |=  (0xf) << 14;
			break;
		}
	}
	/*
	 * do the read
	 */
	val32 = readl((u32 __iomem *)bus_addr);

	switch (len) {
	case 1:
		*val = (val32 >> (bo * 8)) & 0xff;
		break;
	case 2:
		*val = (val32 >> (bo * 8)) & 0xffff;
		break;
	default:
		*val = val32;
		break;
	}

#ifdef PRINT_CONFIG_ACCESSES
	pr_info("acp_read_config for PCIE%d: %3d  fn=0x%04x o=0x%04x l=%d "
		"a=0x%08x v=0x%08x, dev=%d\n",
			port->index, bus->number, devfn, offset, len,
			bus_addr, *val, PCI_SLOT(devfn));
#endif
	return rc;
}

/*
 * Write PCI config space.
 */
static int
arm_pciex_axxia_write_config(struct pci_bus *bus,
			     unsigned int devfn,
			     int offset,
			     int len,
			     u32 val)
{
	struct axxia_pciex_port *port = bus_to_port(bus);
	void __iomem *addr;
	u32 bus_addr;
	u32 val32;

	if (axxia_pciex_validate_bdf(bus, devfn) != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = axxia_pciex_get_config_base(port, bus, devfn);

	if (!addr)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * addressing is different for local config access vs.
	 * access through the mapped config space. We need to
	 * translate the offset for mapped config access
	 */
	if (bus->number == 0) {
		/* the local ACP RC only supports 32-bit dword config access,
		 * so if this is a byte or 16-bit word access we need to
		 * perform a read-modify write
		 */
		if (len == 4) {
			bus_addr = (u32) addr + offset;
		} else {
			int bs = ((offset & 0x3) * 8);

			bus_addr = (u32) addr + (offset & 0xfffffffc);
			val32 = readl((u32 __iomem *)bus_addr);

			if (len == 2) {
				val32 = (val32 & ~(0xffff << bs))
					| ((val & 0xffff) << bs);
			} else {
				val32 = (val32 & ~(0xff << bs))
					| ((val & 0xff) << bs);
			}

			val = val32;
			len = 4;
		}
	} else {
		bus_addr = (u32) addr + (offset << 2) + (offset & 0x3);
	}

#ifdef PRINT_CONFIG_ACCESSES
	pr_info("acp_write_config: bus=%3d devfn=0x%04x offset=0x%04x len=%d"
		"addr=0x%08x val=0x%08x\n",
		bus->number, devfn, offset, len, bus_addr, val);
#endif

	switch (len) {
	case 1:
		writeb(val, (u8 __iomem *)(bus_addr));
		break;
	case 2:
		writew(val, (u16 __iomem *)(bus_addr));
		break;
	default:
		writel(val, (u32 __iomem *)(bus_addr));
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops axxia_pciex_pci_ops = {
	.read  = arm_pciex_axxia_read_config,
	.write = arm_pciex_axxia_write_config,
};

/*
 * pcie_legacy_isr
 *
 * The interrupt line for this handler is shared between the PCIE controller
 * itself (for status and error interrupts) and devices using legacy PCI
 * interupt signalling. Statis and error interrupts are serviced here and this
 * handler will return IRQ_HANDLED. If the reasont is the assertion of a device
 * legacy interrupt, this handler returns IRQ_NONE the next action on this line
 * will be called (the PCI EP interrupt service routine).
 */
static irqreturn_t
pcie_legacy_isr(int irq, void *arg)
{
	struct axxia_pciex_port *port  = arg;
	void __iomem            *mbase = port->regs;
	u32                      intr_status;
	irqreturn_t              retval = IRQ_HANDLED;

	/* read the PEI interrupt status register */
	intr_status = readl(mbase + PCIE_INT0_STATUS);

	/* check if this is a PCIe message not from an external device */
	if (intr_status & INT0_ERROR) {
			u32 int_enb;
			u32 offset;

			pr_info("PCIE%d: Error interrupt %#x\n",
				port->index, intr_status);

			pr_info("PCIE%d: link status = %#x\n",
				port->index, readl(mbase + PCIE_LINK_STATUS));

			if (intr_status & 0x00020000) {
				pr_info("PCIE%d: t2a_fn_indp_err_stat = %#x\n",
					port->index, readl(mbase+0x1170));
				int_enb = readl(mbase + PCIE_INT0_ENABLE);
				int_enb &= 0xfffdffff;
				writel(int_enb, mbase + PCIE_INT0_ENABLE);
			}

			if (intr_status & 0x00040000) {
				pr_info("PCIE%d: t2a_fn_indp_other_err_stat = %#x\n",
					port->index, readl(mbase+0x1174));
				int_enb = readl(mbase + PCIE_INT0_ENABLE);
				int_enb &= 0xfffbffff;
				writel(int_enb, mbase + PCIE_INT0_ENABLE);
			}

			if (intr_status & 0x00000800) {
				pr_info("PCIE%d: config=%#x status=%#x\n",
					port->index,
					readl(mbase + PCIE_CONFIG),
					readl(mbase + PCIE_STATUS));
				int_enb = readl(mbase + PCIE_INT0_ENABLE);
				int_enb &= 0xfffff7ff;
				writel(int_enb, mbase + PCIE_INT0_ENABLE);
			}

			/*
			 * Dump all the potentially interesting PEI registers
			 */
			for (offset = 0x114c; offset <= 0x1180; offset += 4) {
				pr_err("  [0x%04x] = 0x%08x\n",
				       offset, readl(mbase + offset));
			}
	} else if (intr_status & INT0_INT_ASSERTED) {
		/* Empty the message FIFO */
		while ((readl(port->regs + PCIE_MSG_IN_FIFO_STATUS) & 1) == 0)
			(void) readl(port->regs + PCIE_MSG_IN_FIFO);
		/* Next handler in chain will service this interrupt */
		retval = IRQ_NONE;
	}

	/*
	 *  We clear all the interrupts in the PEI status, even though
	 *  interrupts from external devices have not yet been handled.
	 *  That should be okay, since the PCI IRQ in the GIC won't be
	 *  re-enabled until all external handlers have been called.
	 */
	writel(intr_status, mbase + PCIE_INT0_STATUS);

	return retval;
}

/*
 * pcie_msi_irq_handler
 *
 * This is the handler for PCIE MSI service. It will decode the signalled MSI
 * using the following hierarchy of status bits. This handler is installed as a
 * chained handler for each of the 16 interrupt lines on the top-level
 * interrupt controller. When a pending MSI is found, this handler forwards the
 * interrupt service to the corresponding MSI IRQs (numbered from
 * AXXIA_MSI_FIRST..NR_IRQS).
 *
 *                                             PCIE_MSI1_STATUS(group)
 *
 *                 PCIE_MSI0_STATUS                  +----------+
 *                                                   | MSI      |
 *   +----------+    +----------+                    | 0..15    |
 *   | ARM GIC  |    | GROUP    |        /-----------+          |
 *   |          +----+ 0..15    +-------/            |          |
 *   |          |    |          |                    |          |
 *   |          +----+          +-------\            +----------+
 *   |          |    |          |        \
 *   |          +----+          |         \          +----------+
 *   |          |    |          |          \         | MSI      |
 *   |          +----+          |           \        | 16..31   |
 *   |          |    |          |            \-------+          |
 *   |          +----+          |                    |          |
 *   |          |    |          |                    |          |
 *   |          |    |          |                    +----------+
 *   |          | .  |          |
 *   |          | .  |          |                    ...
 *   |          | .  |          |
 *   |          |    |          |                    +----------+
 *   |          |    |          |                    | MSI      |
 *   |          +----+          +--------------------+ 240..255 |
 *   |          |    |          |                    |          |
 *   +----------+    +----------+                    |          |
 *                                                   |          |
 *                                                   +----------+
 */
static void
pcie_msi_irq_handler(int irq, struct axxia_pciex_port *port)
{
	void __iomem *mbase = port->regs;
	u32 group = irq - port->irq[1];
	u32 status;

	/* Check if interrupt is pending */
	status = readl(mbase + PCIE_MSI0_STATUS);
	if (!(status & (1<<group)))
		return;

	/* Check next level interrupt status */
	status = readl(mbase + PCIE_MSI1_STATUS(group)) & 0xffff;
	while (status) {
		u32 line = ffs(status) - 1;
		status &= ~(1<<line);
		/* Clear interrupt on sub-level */
		writel((1<<line), mbase + PCIE_MSI1_STATUS(group));
		generic_handle_irq(AXXIA_MSI_FIRST + (group * 16) + line);
	}

	/* Clear interrupt on top-level*/
	writel(1<<group, mbase + PCIE_MSI0_STATUS);

}

static void
pcie_pei0_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	kstat_incr_irqs_this_cpu(irq, desc);
	/* Handle the PCIe interrupt */
	pcie_msi_irq_handler(irq, &axxia_pciex_ports[0]);
	/* Signal end-of-interrupt */
	irq_desc_get_chip(desc)->irq_eoi(&desc->irq_data);
}

/* PCIe setup function */
static int axxia_pcie_setup(int portno, struct pci_sys_data *sys)
{
	struct axxia_pciex_port *port = &axxia_pciex_ports[sys->domain];
	u32 pci_config, pci_status, link_state;
	int i, num_pages, err;
	u32 outbound_size;
	u32 inbound_size;
	u64 dest;

	port->root_bus_nr = sys->busnr;

	/* Map PCIe bridge control registers */
	port->regs = ioremap(port->utl_regs.start,
			     resource_size(&port->utl_regs));
	if (!port->regs) {
		pr_err("PCIE%d: Failed to map control registers\n", portno);
		goto fail;
	}

	/* Map range for access to PCI configuration space */
	port->cfg_data = ioremap(port->cfg_space.start,
				 resource_size(&port->cfg_space));
	if (!port->cfg_data) {
		pr_err("PCIE%d: Failed to map config space\n", portno);
		goto fail;
	}

	pci_add_resource_offset(&sys->resources, &port->outbound,
				port->outbound.start - port->pci_addr);

	/* Status/error interrupt */
	port->irq[0] = irq_of_parse_and_map(port->node, 0);
	err = request_irq(port->irq[0], pcie_legacy_isr, IRQF_SHARED,
			  "pcie", port);
	if (err) {
		pr_err("PCIE%d: Failed to request IRQ#%d (%d)\n",
		       portno, port->irq[0], err);
		goto fail;
	}

	/* MSI interrupts for PEI0 */
	if (sys->domain == 0) {
		for (i = 1; i <= 16; i++) {
			port->irq[i] = irq_of_parse_and_map(port->node, i);
			irq_set_chained_handler(port->irq[i],
						pcie_pei0_msi_handler);
		}
	}

	/* Setup as root complex */
	pci_config = readl(port->regs + PCIE_CONFIG);
	pci_status = readl(port->regs + PCIE_STATUS);
	link_state = (pci_status >> 8) & 0x3f;
	pr_info("PCIE%d: status=0x%08x, link state=%#x\n",
		port->index, pci_status, link_state);

	/* make sure the ACP device is configured as PCI Root Complex */
	if ((pci_status & 0x18) != 0x18) {
		pr_err("PCIE%d: Device is not Root Complex\n", port->index);
		goto fail;
	}

	/* Make sure the link is up */
	if (link_state != 0xb) {
		/* Reset */
		pr_warn("PCIE%d: Link in bad state - resetting\n", port->index);
		pci_config |= 1;
		writel(pci_config, port->regs + PCIE_CONFIG);
		msleep(1000);
		pci_status = readl(port->regs + PCIE_STATUS);
		link_state = (pci_status & 0x3f00) >> 8;
		pr_warn("PCIE%d: (after reset) link state=%#x\n",
			port->index, link_state);
		if (link_state != 0xb) {
			pr_warn("PCIE%d: Link in bad state - giving up!\n",
				port->index);
			goto fail;
		}
	}

	/*
	 * Setup outbound PCI Memory Window
	 */

	outbound_size = resource_size(&port->outbound);
	num_pages = (outbound_size + MPAGE_SIZE - 1) / MPAGE_SIZE;
	dest = port->pci_addr;
	for (i = 0; i < num_pages; i++) {
		u32 mpage_u = dest >> 32;
		u32 mpage_l = (u32)dest & ~(MPAGE_SIZE-1);
		writel(mpage_u, port->regs + PCIE_MPAGE_U(i));
		writel(mpage_l, port->regs + PCIE_MPAGE_L(i));
		pr_debug("PCIE%d: MPAGE(%d) = %08x %08x\n",
			 port->index, i, mpage_u, mpage_l);
		dest += MPAGE_SIZE;
	}

	/*
	 * Setup inbound PCI window
	 */

	/* Configure the inbound window size */
	inbound_size = (u32) resource_size(&port->inbound);
	writel(~(inbound_size-1), port->regs + PCIE_RC_BAR0_SIZE);

	/* Verify BAR0 size */
	{
		u32 bar0_size;
		writel(~0, port->regs + PCI_BASE_ADDRESS_0);
		bar0_size = readl(port->regs + PCI_BASE_ADDRESS_0);
		if ((bar0_size & ~0xf) != ~(inbound_size-1))
			pr_err("PCIE%d: Config BAR0 failed\n", port->index);
	}

	/* Set the BASE0 address to start of PCIe base */
	writel(port->pci_bar, port->regs + PCI_BASE_ADDRESS_0);

        /* Set the BASE1 address to 0x0 */
        writel(0x0, port->regs + PCI_BASE_ADDRESS_1);


	/* Setup TPAGE registers for inbound mapping
	 *
	 * We set the MSB of each TPAGE to select 128-bit AXI access. For the
	 * address field we simply program an incrementing value to map
	 * consecutive pages
	 */
	for (i = 0; i < 8; i++)
		writel(PCIE_TPAGE_128 | i, port->regs + PCIE_TPAGE_BAR0(i));


	/* Enable all legacy/status/error interrupts */
	writel(INT0_MSI | INT0_INT_ASSERTED | INT0_ERROR,
	       port->regs + PCIE_INT0_ENABLE);

	/* Enable all MSI interrupt groups */
	writel(0xFFFF, port->regs + PCIE_MSI0_ENABLE);
	/* Enable all lines in all subgroups */
	for (i = 0; i < 16; i++)
		writel(0xFFFF, port->regs + PCIE_MSI1_ENABLE(i));

	return 1;
fail:
	if (port->cfg_data)
		iounmap(port->cfg_data);
	if (port->regs)
		iounmap(port->regs);
	return 0;
}

/*
 * Allocate MSI page. A MSI is generated when EP writes to this PCI address.
 * The region must be 1Kb to manage 256 MSIs.
 */
static void *
pcie_alloc_msi_table(struct pci_dev *pdev, struct axxia_pciex_port *port)
{
	u32 msi_lower;
	void *msi_virt;

	if (dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64)) != 0 &&
	    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		dev_err(&pdev->dev, "No memory for MSI table\n");
		return NULL;
	}

	msi_virt = dma_alloc_coherent(&pdev->dev, 1024,
				      &port->msi_phys,
				      GFP_KERNEL);
	if (msi_virt) {
		msi_lower = (u32)port->msi_phys;
		/* Please note we have 1:1 mapping for inbound */
		port->msi_pci_addr = port->inbound.start + msi_lower;
		writel(msi_lower>>10, port->regs + PCIE_AXI_MSI_ADDR);
	}

	return msi_virt;
}


/*
 * Scan PCIe bus
 */
static struct pci_bus *
axxia_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	if (WARN_ON(nr >= PCIE_MAX_PORTS))
		return NULL;

	return pci_scan_root_bus(NULL, sys->busnr, &axxia_pciex_pci_ops,
				 sys, &sys->resources);
}



static int
axxia_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pci_sys_data *sys = dev->sysdata;
	struct axxia_pciex_port *port = &axxia_pciex_ports[sys->domain];
	return port->irq[0];
}

/* IRQ chip ops for MSI IRQs */
static struct irq_chip axxia_msi_chip = {
	.name = "PCI-MSI",
	.irq_enable  = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask    = mask_msi_irq,
	.irq_unmask  = unmask_msi_irq,
};

/* Port definition struct */
static struct hw_pci axxia_pcie_hw[] = {
	[0] = {
		.nr_controllers = 1,
		.domain = 0,
		.setup = axxia_pcie_setup,
		.scan = axxia_pcie_scan_bus,
		.map_irq = axxia_pcie_map_irq
	},
	[1] = {
		.nr_controllers = 1,
		.domain = 1,
		.setup = axxia_pcie_setup,
		.scan = axxia_pcie_scan_bus,
		.map_irq = axxia_pcie_map_irq
	}
};

static void
axxia_probe_pciex_bridge(struct device_node *np)
{
	struct axxia_pciex_port *port;
	u32 portno;
	const char *val;
	const u32 *field;
	int rlen;
	int pna = of_n_addr_cells(np); /* address-size of parent */
	u32 pci_space;
	u64 pci_addr;
	u64 cpu_addr;
	u64 size;

	/* Check if device is enabled */
	if (!of_device_is_available(np))
		return;

	/* Get the port number from the device-tree */
	if (of_property_read_u32(np, "port", &portno) != 0) {
		pr_err("%s: No 'port' property\n", np->full_name);
		return;
	}

	if (portno >= PCIE_MAX_PORTS) {
		pr_err("%s: Port %d out of range\n", np->full_name, portno);
		return;
	}

	port = &axxia_pciex_ports[portno];
	port->index = portno;
	snprintf(port->name, sizeof(port->name) - 1, "PCIE%d", portno);
	port->node = of_node_get(np);

	/* Check if device_type property is set to "pci" or "pci-endpoint".
	 * Resulting from this setup this PCIe port will be configured
	 * as root-complex or as endpoint.
	 */
	val = of_get_property(port->node, "device_type", NULL);
	port->endpoint = val && strcmp(val, "pci-endpoint") == 0;

	/* Fetch address range for PCIE config space */
	if (of_address_to_resource(np, 0, &port->cfg_space)) {
		pr_err("PCIE%d: No resource for PCIe config space\n", portno);
		return;
	}

	/* Fetch address range for host bridge internal registers */
	if (of_address_to_resource(np, 1, &port->utl_regs)) {
		pr_err("PCIE%d: No resource for PCIe registers\n", portno);
		return;
	}

	if (request_resource(&iomem_resource, &port->utl_regs))
		return;

	/*
	 * Outbound PCI memory window
	 */

	/* Defaults */
	port->outbound = pcie_outbound_default[portno];
	port->outbound.name = port->name;

	field = of_get_property(np, "ranges", &rlen);
	if (field) {
		pci_space = of_read_number(field + 0, 1);
		switch ((pci_space >> 24) & 3) {
		case 0:
			pr_err("PCIE%d: Invalid 'ranges'\n", portno);
			break;
		case 1: /* PCI IO Space */
			pr_err("PCIE%d: PCI IO not supported\n", portno);
			break;
		case 2: /* PCI MEM 32-bit */
		case 3: /* PCI MEM 64-bit */
			cpu_addr  = of_read_number(field + 3, 2);
			size      = of_read_number(field + 5, 2);
			port->outbound.start = cpu_addr;
			port->outbound.end   = cpu_addr + size - 1;
			port->pci_addr = of_read_number(field + 1, 2);
			break;
		}
	}

	if (resource_size(&port->outbound) > MAX_OUTBOUND_SIZE) {
		pr_err("PCIE%d: Outbound window too large (using max %#x)\n",
		       portno, MAX_OUTBOUND_SIZE);
		port->outbound.end = (port->outbound.start +
				      MAX_OUTBOUND_SIZE - 1);
	}

	if (request_resource(&iomem_resource, &port->outbound)) {
		pr_err("PCIE%d: Memory resource request failed\n", portno);
		return;
	}

	if (request_resource(&iomem_resource, &port->cfg_space)) {
		pr_err("PCIE%d: Config space request failed\n", portno);
		return;
	}

	pr_info("PCIE%d: Outbound %#llx..%#llx (CPU) -> %#llx (PCI)\n",
		portno,
		port->outbound.start, port->outbound.end,
		port->pci_addr);

	/*
	 * Inbound PCI memory window
	 */

	/* Default 4GB */
	port->inbound.name  = "PCIE DMA";
	port->inbound.start = 0x00000000;
	port->inbound.end   = 0xffffffff;
	port->inbound.flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;

	/* Get dma-ranges property */
	field = of_get_property(np, "dma-ranges", &rlen);
	if (!field) {
		pr_info("PCIE%d: No 'dma-ranges' property, using defaults\n",
			portno);
	} else {
		BUILD_BUG_ON(sizeof(resource_size_t) != sizeof(u64));

		/* Limited to one inbound window for now... */
		pci_space = of_read_number(field + 0, 1);
		pci_addr  = of_read_number(field + 1, 2);
		cpu_addr  = of_read_number(field + 3, pna);
		size      = of_read_number(field + pna + 3, 2);

		port->inbound.start = cpu_addr;
		port->inbound.end   = cpu_addr + size - 1;
		port->pci_bar       = pci_addr;
	}

	pr_info("PCIE%d: Inbound %#llx (PCI) -> %#llx..%#llx (CPU)\n",
		portno,
		port->pci_bar,
		port->inbound.start, port->inbound.end);
}

/*
 * Allocate a MSI interrupt number and IRQ (the IRQ is a constant offset from
 * the MSI index). The kernel IRQs for MSI are in a range above hardware IRQs.
 *
 * First call also allocates the 1Kb MSI table and configures the controller.
 */
int
arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	struct axxia_pciex_port *port = bus_to_port(pdev->bus);
	struct msi_msg msg;
	int pos, irq;

	/* Allocate MSI table on demand when first device needs it */
	if (!port->msi_virt)
		port->msi_virt = pcie_alloc_msi_table(pdev, port);

	if (!port->msi_virt)
		return -ENOMEM;

	/* Find available MSI */
again:
	pos = find_first_zero_bit(msi_irq_in_use, NUM_MSI_IRQ);
	/* Offset to get the IRQ */
	irq = AXXIA_MSI_FIRST + pos;
	if (irq >= NR_IRQS)
		return -ENOSYS;
	if (test_and_set_bit(pos, msi_irq_in_use))
		goto again;

	/* Initialize IRQ descriptor */
	dynamic_irq_init(irq);
	irq_set_msi_desc(irq, desc);
	/* Use a simple handle for our "SW" MSI IRQs */
	irq_set_chip_and_handler(irq, &axxia_msi_chip, handle_simple_irq);
	set_irq_flags(irq, IRQF_VALID);

	/* Configure PCI device with its MSI address */
	msg.address_hi = 0x0;
	msg.address_lo = port->msi_pci_addr + 4*pos;
	msg.data       = irq;
	write_msi_msg(irq, &msg);

	return 0;
}

/*
 * Called by the generic MSI layer to free MSI IRQ.
 */
void
arch_teardown_msi_irq(unsigned int irq)
{
	int pos = irq - AXXIA_MSI_FIRST;

	if (0 <= pos && pos < NR_IRQS) {
		clear_bit(pos, msi_irq_in_use);
		dynamic_irq_cleanup(irq);
	}
}

/**
 * Initialize PCIe controller(s) found in the device tree.
 */
void __init
axxia_pcie_init(void)
{
	struct device_node *np;
	int num_ports = 0;

	/* allocate memory */
	axxia_pciex_ports = kzalloc(PCIE_MAX_PORTS *
				    sizeof(struct axxia_pciex_port),
				    GFP_KERNEL);
	if (!axxia_pciex_ports) {
		pr_err("PCIE: No memory\n");
		return;
	}

	for_each_compatible_node(np, NULL, "lsi,plb-pciex") {
		if (!of_device_is_available(np))
			continue;

		axxia_probe_pciex_bridge(np);
		pci_common_init(&axxia_pcie_hw[num_ports]);
		if (++num_ports == PCIE_MAX_PORTS)
			break;
	}
}
