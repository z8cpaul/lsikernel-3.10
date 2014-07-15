/*
 *  linux/arch/arm/mach-axxia/lsi_power_management.c
 *
 *  C *  Created on: Jun 19, 2014
 *      Author: z8cpaul
 *  opyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Created on: Jun 19, 2014
 *      Author: z8cpaul
 */
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/delay.h>
#include <asm/exception.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <asm/cp15.h>

#include "lsi_power_management.h"

#define SYSCON_PHYS_ADDR 0x002010030000ULL
#define DICKENS_PHYS_ADDR 0x2000000000

#define PM_WAIT_TIME (10000)
#define MAX_CLUSTER  (4)

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

static const u32 cluster_to_node[MAX_CLUSTER] = { DKN_CLUSTER0_NODE,
DKN_CLUSTER1_NODE,
DKN_CLUSTER2_NODE,
DKN_CLUSTER3_NODE };

static const u32 cluster_to_poreset[MAX_CLUSTER] = {
PORESET_CLUSTER0,
PORESET_CLUSTER1,
PORESET_CLUSTER2,
PORESET_CLUSTER3 };

static u32 pm_cpu_powered_down;

static u32 got_to;

/*======================= LOCAL FUNCTIONS ==============================*/
static void pm_set_bits_syscon_register(void __iomem *syscon, u32 reg, u32 data);
static void pm_clear_bits_syscon_register(void __iomem *syscon, u32 reg, u32 data);
static bool pm_test_for_bit_with_timeout(void __iomem *syscon, u32 reg, u32 bit);
static bool pm_wait_for_bit_clear_with_timeout(void __iomem *syscon, u32 reg,
		u32 bit);
static void pm_dickens_logical_shutdown(u32 cluster);
static void pm_dickens_logical_powerup(u32 cluster);
static int pm_cpu_physical_isolation_and_power_down(int cpu);
static void pm_L2_isolation_and_power_down(int cluster);
static void __pm_cpu_shutdown(void *data);
static int pm_cpu_physical_connection_and_power_up(int cpu);
static void pm_L2_physical_connection_and_power_up(u32 cluster);
static void pm_L2_logical_powerup(u32 cluster, u32 cpu);

static bool pm_first_cpu_of_cluster(u32 cpu) {
	u32 count = 0;
	switch (cpu) {
	case (0):
	case (1):
	case (2):
	case (3):
		/* This will never happen because cpu 0 will never be turned off */
		break;
	case (4):
	case (5):
	case (6):
	case (7):
		if (pm_cpu_powered_down & (1 << 4))
			count++;
		if (pm_cpu_powered_down & (1 << 5))
			count++;
		if (pm_cpu_powered_down & (1 << 6))
			count++;
		if (pm_cpu_powered_down & (1 << 7))
			count++;
		if (count == 4)
			return true;
		break;
	case (8):
	case (9):
	case (10):
	case (11):
		if (pm_cpu_powered_down & (1 << 8))
			count++;
		if (pm_cpu_powered_down & (1 << 9))
			count++;
		if (pm_cpu_powered_down & (1 << 10))
			count++;
		if (pm_cpu_powered_down & (1 << 11))
			count++;
		if (count == 4)
			return true;
		break;
	case (12):
	case (13):
	case (14):
	case (15):
		if (pm_cpu_powered_down & (1 << 12))
			count++;
		if (pm_cpu_powered_down & (1 << 13))
			count++;
		if (pm_cpu_powered_down & (1 << 14))
			count++;
		if (pm_cpu_powered_down & (1 << 15))
			count++;
		if (count == 4)
			return true;
		break;
	default:
		pr_err("ERROR: the cpu does not exist: %d - %s:%d\n", cpu, __FILE__,
				__LINE__);
		break;
	}
	return false;
}

bool pm_cpu_last_of_cluster(u32 cpu) {
	u32 count = 0;
	switch (cpu) {
	case (0):
	case (1):
	case (2):
	case (3):
		/* This will never happen because cpu 0 will never be turned off */
		break;
	case (4):
	case (5):
	case (6):
	case (7):
		if (pm_cpu_powered_down & (1 << 4))
			count++;
		if (pm_cpu_powered_down & (1 << 5))
			count++;
		if (pm_cpu_powered_down & (1 << 6))
			count++;
		if (pm_cpu_powered_down & (1 << 7))
			count++;
		if (count == 3)
			return true;
		break;
	case (8):
	case (9):
	case (10):
	case (11):
		if (pm_cpu_powered_down & (1 << 8))
			count++;
		if (pm_cpu_powered_down & (1 << 9))
			count++;
		if (pm_cpu_powered_down & (1 << 10))
			count++;
		if (pm_cpu_powered_down & (1 << 11))
			count++;
		if (count == 3)
			return true;
		break;
	case (12):
	case (13):
	case (14):
	case (15):
		if (pm_cpu_powered_down & (1 << 12))
			count++;
		if (pm_cpu_powered_down & (1 << 13))
			count++;
		if (pm_cpu_powered_down & (1 << 14))
			count++;
		if (pm_cpu_powered_down & (1 << 15))
			count++;
		if (count == 3)
			return true;
		break;
	default:
		pr_err("ERROR: the cpu does not exist: %d - %s:%d\n", cpu,  __FILE__,
				__LINE__);
		break;
	}
	return false;
}

static void pm_set_bits_syscon_register(void __iomem *syscon, u32 reg, u32 data) {
	u32 tmp;

	tmp = readl(syscon + reg);
	tmp |= data;
	writel(tmp, syscon + reg);
}

static void pm_clear_bits_syscon_register(void __iomem *syscon, u32 reg, u32 data) {
	u32 tmp;

	tmp = readl(syscon + reg);
	tmp &= ~(data);
	writel(tmp, syscon + reg);
}

static bool pm_test_for_bit_with_timeout(void __iomem *syscon, u32 reg, u32 bit) {

	u32 tmp = 0;
	u32 cnt = 0;

	while (cnt < PM_WAIT_TIME)
	{
		tmp = readl(syscon + reg);
		if (CHECK_BIT(tmp, bit))
			break;
		cnt++;
	}
	if (cnt == PM_WAIT_TIME)
	{
		pr_err("reg=0x%x tmp:=0x%x\n", reg, tmp);
		return false;
	}
	return true;
}

static bool pm_wait_for_bit_clear_with_timeout(void __iomem *syscon, u32 reg,
		u32 bit)
{
	u32 cnt = 0;
	u32 tmp = 0;

	while (cnt < PM_WAIT_TIME)
	{
		tmp = readl(syscon + reg);
		if (!(CHECK_BIT(tmp, bit)))
			break;
		cnt++;
	}
	if (cnt == PM_WAIT_TIME) {
		pr_err("reg=0x%x tmp:=0x%x\n", reg, tmp);
		return false;
	}

	return true;
}
static void pm_dickens_logical_shutdown(u32 cluster) {
	int i;
	u32 status;
	u32 bit;
	int retries;
	void __iomem *dickens;
	got_to = __LINE__;

	dickens = ioremap(DICKENS_PHYS_ADDR, SZ_4M);
	if (dickens == NULL) {
		pr_err("DICKENS: Failed to map the dickens registers\n");
		return;
	}

	bit = (0x01 << cluster_to_node[cluster]);

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		writel(bit,
				dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
						+ DKN_HNF_SNOOP_DOMAIN_CTL_CLR);

		retries = PM_WAIT_TIME;

		do {
			status = readl(
					dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
							+ DKN_HNF_SNOOP_DOMAIN_CTL);
			udelay(1);
		} while ((0 < --retries) && (CHECK_BIT(status, bit)));

		if (0 == retries)
		{
			pr_err("DICKENS: Failed to clear the SNOOP main control\n");
			return;

		}

	}
	/* Clear the domain cluster */
	writel(bit, dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL_CLR);

	/* Check for complete */
	retries = PM_WAIT_TIME;

	do {
		status = readl(
				dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET)
						+ DKN_MN_DVM_DOMAIN_CTL);
		udelay(1);
	} while ((0 < --retries) && (CHECK_BIT(status, bit)));

	if (0 == retries)
	{
		pr_err("DICKENS: failed to set DOMAIN OFFSET\n");
		return;

	}

	iounmap(dickens);

	return;
}

static void pm_dickens_logical_powerup(u32 cluster)
{
	int i;
	u32 status;
	u32 bit;
	int retries;

	void __iomem *dickens = ioremap(DICKENS_PHYS_ADDR, SZ_4M);
	if (dickens == NULL) {
		pr_err("Failed to map dickens registers\n");
		return;
	}

	bit = (0x01 << cluster_to_node[cluster]);

	for (i = 0; i < DKN_HNF_TOTAL_NODES; ++i) {
		writel(bit,
				dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
						+ DKN_HNF_SNOOP_DOMAIN_CTL_SET);

		retries = PM_WAIT_TIME;

		do {
			status = readl(
					dickens + (0x10000 * (DKN_HNF_NODE_ID + i))
							+ DKN_HNF_SNOOP_DOMAIN_CTL);
			udelay(1);
		} while ((0 < --retries) && (!CHECK_BIT(status, bit)));

		if (0 == retries)
			BUG();

	}

	/* Clear the domain cluster */
	writel(bit, dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET) + DKN_MN_DVM_DOMAIN_CTL_SET);

	/* Check for complete */
	retries = PM_WAIT_TIME;

	do {
		status = readl(
				dickens + (0x10000 * DKN_DVM_DOMAIN_OFFSET)
						+ DKN_MN_DVM_DOMAIN_CTL);
		udelay(1);
	} while ((0 < --retries) && (!CHECK_BIT(status, bit)));

	if (0 == retries)
		BUG();

	iounmap(dickens);

	return;
}

static void __pm_cpu_shutdown(void *data)
{

	pm_data *pm_request = (pm_data *)data;
	void __iomem *syscon;
	bool success;
	u32 cluster_mask = (0x01 << pm_request->cluster);
	bool last_cpu;
	int rVal = 0;

	/*
	 * Is this the last cpu of a cluster then turn off the L2 cache
	 * along with the CPU.
	 */
	last_cpu = pm_cpu_last_of_cluster(pm_request->cpu);
	if (last_cpu) {

		/* Remove the cluster from the Dickens coherency domain */
		pm_dickens_logical_shutdown(pm_request->cluster);

		/* Power down the cpu */
		pm_cpu_physical_isolation_and_power_down(pm_request->cpu);

		syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
		if (WARN_ON(!syscon))
			return;

		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_CSYSREQ_TS, cluster_mask);
		success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_CACTIVE_TS, pm_request->cluster);
		if (!success) {
			pr_err(
					"Failed to keep other cluster TS going on cluster %d: %s-%d\n",
					pm_request->cluster, __FILE__, __LINE__);
			iounmap(syscon);
			return;
		}

		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_CSYSREQ_ATB, cluster_mask);
		success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_CACTIVE_ATB, pm_request->cluster);
		if (!success) {
			pr_err(
					"Failed to keep other cluster ATB going on cluster %d: %s-%d\n",
					pm_request->cluster, __FILE__, __LINE__);
			iounmap(syscon);
			return;
		}

		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_CSYSREQ_APB, cluster_mask);
		success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_CACTIVE_APB, pm_request->cluster);
		if (!success) {
			pr_err(
					"Failed to keep other cluster APB going on cluster %d: %s-%d\n",
					pm_request->cluster, __FILE__, __LINE__);
			iounmap(syscon);
			return;
		}

		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_CSYSREQ_CNT, cluster_mask);
		success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_CACTIVE_CNT, pm_request->cluster);
		if (!success) {
			pr_err(
					"Failed to keep other cluster count going on cluster %d: %s-%d\n",
					pm_request->cluster, __FILE__, __LINE__);
			iounmap(syscon);
			return;
		}

		/* Turn off the ACE */
		pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_ACEPWRDNRQ, cluster_mask);

		/* Wait for ACE to complete power off */
		success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_NACEPWRDNACK, pm_request->cluster);
		if (!success) {
			pr_err("Failed to power off ACE on cluster %d: %s-%d\n",
					pm_request->cluster, __FILE__, __LINE__);
			iounmap(syscon);
			return;
		}

		/* Isolate the cluster */
		pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_ISOLATEL2MISC, cluster_mask);

		/* Wait for WFI L2 to go to standby */
		success = pm_test_for_bit_with_timeout(syscon, NCP_SYSCON_PWR_STANDBYWFIL2, pm_request->cluster);
		if (!success) {
			pr_err("Failed to enter L2 WFI on cluster %d: %s-%d\n",
					pm_request->cluster, __FILE__, __LINE__);
			iounmap(syscon);
			return;
		}

		iounmap(syscon);

		/* Power off the L2 */
		pm_L2_isolation_and_power_down(pm_request->cluster);
		if (rVal == 0)
			pr_info("CPU %d is powered down with cluster: %d\n", pm_request->cpu, pm_request->cluster);
		else
			pr_err("CPU %d failed to power down\n", pm_request->cpu);


	} else {

		rVal = pm_cpu_physical_isolation_and_power_down(pm_request->cpu);
		if (rVal == 0)
		{
			pr_info("CPU %d is powered down\n", pm_request->cpu);
			pm_cpu_powered_down |= (1 << pm_request->cpu);
		}
		else
			pr_err("CPU %d failed to power down\n", pm_request->cpu);
	}

	return;
}

int pm_cpu_logical_die(pm_data *pm_request)
{
	void __iomem *syscon;
	bool success;

	smp_call_function_single(pm_request->cpu, pm_cpu_logical_shutdown, (void *)pm_request, 1);

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return (-1);

	/* Wait for the cpu to enter wfi */
	success = pm_test_for_bit_with_timeout(syscon, NCP_SYSCON_PWR_STANDBYWFI, pm_request->cpu);
	if (!success) {
		pr_err("Failed to enter WFI mode on cpu %d: %s-%d\n",
				pm_request->cpu, __FILE__, __LINE__);
		iounmap(syscon);
		return(-1);
	}

	iounmap(syscon);
	return (0);
}

int pm_cpul2_logical_die(pm_data *pm_request)
{
	void __iomem *syscon;
	bool success;

	smp_call_function_single(pm_request->cpu, pm_L2_logical_shutdown, (void *)pm_request, 1);

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return (-1);

	/* Wait for the cpu to enter wfi */
	success = pm_test_for_bit_with_timeout(syscon, NCP_SYSCON_PWR_STANDBYWFI, pm_request->cpu);
	if (!success) {
		pr_err("Failed to enter WFI mode on cpu %d: %s-%d\n",
				pm_request->cpu, __FILE__, __LINE__);
		iounmap(syscon);
		return(-1);
	}

	iounmap(syscon);
	return (0);
}

void pm_cpu_shutdown(u32 cpu) {

	pm_data pm_request;

	u32 pcpu = cpu_logical_map(smp_processor_id());
	u32 rcpu = cpumask_any_and(cpu_present_mask, cpu_online_mask);
	u32 reqCpu = cpu_logical_map(cpu);

	/* Check to see if the cpu is powered up */
	if (pm_cpu_powered_down & (1 << cpu)) {
		pr_err("CPU %d is already powered off - %s:%d\n", cpu, __FILE__, __LINE__);
		return;
	}
	/*
	 * Is this the last cpu to be powered off, then don't
	 * allow the power to be shut off.
	 */
	if (cpu == 0)
	{
		pr_err("Cannot turn off cpu 0 - %s:%d\n", __FILE__, __LINE__);
		return ;
	}

	/*
	 * Is this process on the requested cpu to power down
	 * then send it to another cpu for processing
	 */
	pm_request.cpu = cpu;
	pm_request.cluster = reqCpu / CORES_PER_CLUSTER;

	if (pcpu == cpu) {
		smp_call_function_single(rcpu, __pm_cpu_shutdown, (void *)&pm_request, 0);

	} else {
		__pm_cpu_shutdown(&pm_request);
	}

}

int pm_cpu_powerup(u32 cpu) {

	bool first_cpu;
	int rVal = 0;
	void __iomem *syscon = NULL;
	u32 cpu_mask = (0x01 << cpu);

	u32 pcpu = cpu_logical_map(smp_processor_id());
	u32 cluster = pcpu / CORES_PER_CLUSTER;

	/*
	 * Is this the first cpu of a cluster to come back on?
	 * Then power up the L2 cache.
	 */
	first_cpu = pm_first_cpu_of_cluster(cpu);
	if (first_cpu) {

		pm_L2_logical_powerup(cpu, cluster);

	} else {

		/* Hold the CPU in reset */

		syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
		if (WARN_ON(!syscon))
			return(-EINVAL);

		pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWRUP_CPU_RST, cpu_mask);
		iounmap(syscon);

		rVal = pm_cpu_physical_connection_and_power_up(cpu);
		if (rVal)
		{
			pr_err("Failed to power up physical connection of cpu: %d\n", cpu);
			goto pm_power_up;
		}

		udelay(16);

		/* Clear the CPU from reset and let it go */
		syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
		if (WARN_ON(!syscon))
			return(-EINVAL);

		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWRUP_CPU_RST,	cpu_mask);

		/*
		 * Clear the powered down mask
		 */
		pm_cpu_powered_down &= ~(1 << cpu);

	}

pm_power_up:
	iounmap(syscon);
	return(rVal);
}

unsigned long pm_get_powered_down_cpu(void)
{
	return (pm_cpu_powered_down);
}

void pm_cpu_logical_shutdown(void *data) {
	u32 val;

	asm volatile(
	"       mrc     p15, 0, %0, c1, c0, 0\n"
	"       bic     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 0\n"
	: "=&r" (val)
	: "Ir" (CR_C)
	: "cc");

	/* Clear and invalidate all date from L1 data cache */
	flush_cache_all();

	/* Switch the processor over to AMP mode out of SMP */
	asm volatile(
			"       mrc     p15, 0, %0, c1, c0, 1\n"
			"       bic     %0, %0, %1\n"
			"       mcr     p15, 0, %0, c1, c0, 1\n"
			: "=&r" (val)
			: "Ir" (0x40)
			: "cc");

	isb();
	dsb();

	wfi();

	return;

}

static int pm_cpu_physical_isolation_and_power_down(int cpu) {

	void __iomem *syscon;
	int rVal = 0;

	bool success;
	u32 mask = (0x01 << cpu);

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return -EINVAL;

	/* Initiate power down of the CPU's HS Rams */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPCPURAM, mask);

	/* Wait until the RAM power down is complete */
	success = pm_test_for_bit_with_timeout(syscon, NCP_SYSCON_PWR_NPWRUPCPURAM_ACK, cpu);
	if (!success)
	{
		rVal = -EINVAL;
		pr_err("CPU: Failed to power down CPU RAM\n");
		goto power_down_cleanup;
	}

	/* Activate the CPU's isolation clamps */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_ISOLATECPU, mask);

	/* Initiate power down of the CPU logic */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPCPUSTG2, mask);

	udelay(10);

	/* Continue power down of the CPU logic */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPCPUSTG1, mask);

	success = pm_test_for_bit_with_timeout(syscon, NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK, cpu);
	if (!success)
	{
		rVal = -EINVAL;
		pr_err("CPU: Failed to power down stage 1 cpu\n");
		goto power_down_cleanup;
	}

power_down_cleanup:
	iounmap(syscon);
	return rVal;
}

static int pm_cpu_physical_connection_and_power_up(int cpu)
{

	int rVal = 0;
	void __iomem *syscon;
	bool success;
	u32 mask = (0x01 << cpu);

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return(-EINVAL);

	/* Initiate power up of the CPU */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPCPUSTG1, mask);

	/* Wait until CPU logic power is compete */
	success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK, cpu);
	if (!success)
	{
		rVal = -EINVAL;
		pr_err("CPU: Failed to get ACK from power down stage 1\n");
		goto power_up_cleanup;
	}

	/* Continue stage 2 power up of the CPU*/
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPCPUSTG2, mask);

	udelay(10);

	/* Initiate power up of HS Rams */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPCPURAM, mask);

	/* Wait until the RAM power up is complete */
	pr_err("%s-%d\n", __FILE__, __LINE__);
	success = pm_wait_for_bit_clear_with_timeout(syscon, NCP_SYSCON_PWR_NPWRUPCPURAM_ACK, cpu);
	if (!success)
	{
		rVal = -EINVAL;
		pr_err("CPU: Failed to get ACK of power power up\n");
		goto power_up_cleanup;
	}

	/* Release the CPU's isolation clamps */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_ISOLATECPU, mask);

power_up_cleanup:
	iounmap(syscon);

	return(rVal);

}
/*========================================== L2 FUNCTIONS ========================================*/

void pm_L2_logical_shutdown(void *data) {
	u32 val;


	asm volatile(
	"       mrc     p15, 0, %0, c1, c0, 0\n"
	"       bic     %0, %0, %1\n"
	"       mcr     p15, 0, %0, c1, c0, 0\n"
	: "=&r" (val)
	: "Ir" (CR_C)
	: "cc");


	asm volatile(
			/*
			 * Disable L2 prefetch
			 */
			"       mrc     p15, 1, %0, c15, c0, 3\n"
			"       orr     %0, %0, %1\n"
			"       mcr     p15, 1, %0, c15, c0, 3\n"
			: "=&r" (val)
			: "Ir" (0x400)
			: "cc");

	isb();
	dsb();

	/* Clear and invalidate all L1 and L2 data cache */
	flush_cache_all();


	/* Turn the DBG Double Lock quiet */
	asm volatile(
			/*
			 * Turn Off the DBGOSDLR.DLK bit
			 */
			"       mrc     p14, 0, %0, c1, c3, 4\n"
			"       orr     %0, %0, %1\n"
			"       mcr     p14, 0, %0, c1, c3, 4\n"
			: "=&r" (val)
			: "Ir" (0x1)
			: "cc");

	/* Switch the processor over to AMP mode out of SMP */
	asm volatile(
			"       mrc     p15, 0, %0, c1, c0, 1\n"
			"       bic     %0, %0, %1\n"
			"       mcr     p15, 0, %0, c1, c0, 1\n"
			: "=&r" (val)
			: "Ir" (0x40)
			: "cc");

	isb();
	dsb();

	wfi();
	return;
}

static void pm_L2_isolation_and_power_down(int cluster) {
	void __iomem *syscon;
	u32 mask = (0x1 << cluster);


	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return;

	/* Enable the chip select for the cluster */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_CHIPSELECTEN, mask);

	/* Disable the hsram */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL2HSRAM, mask);

	switch (cluster) {
	case (0):

#ifdef PM_POWER_OFF_ONLY_DATARAM
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);

#endif
		break;
	case (1):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (2):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	case (3):

#ifdef PM_POWER_OFF_ONLY_DATARAM

		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
#else
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_ALL_MASK);
		udelay(20);
		pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0, RAM_ALL_MASK);
		udelay(20);
#endif
		break;
	default:
		pr_err("Illegal cluster: %d > 3\n", cluster);
		break;
	}

	/* Power down stage 2 */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL2LGCSTG2, mask);

	/* Power down stage 1 */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL2LGCSTG1, mask);


	iounmap(syscon);
}

static void pm_L2_physical_connection_and_power_up(u32 cluster) {
	void __iomem *syscon;
	bool success;
	u32 mask = (0x1 << cluster);

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return;

	/* Power up stage 1 */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL2LGCSTG1, mask);

	/* Wait for the stage 1 power up to complete */
	success = pm_wait_for_bit_clear_with_timeout(syscon,
			NCP_SYSCON_PWR_NPWRUPL2LGCSTG1_ACK, cluster);
	if (!success)
		goto power_up_l2_cleanup;

	/* Power on stage 2 */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL2LGCSTG2, mask);

	/* Set the chip select */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_CHIPSELECTEN, mask);

	/* Power up the ram */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_PWRUPL2HSRAM, mask);

	/* Wait for the stage 1 power up to complete */
	success = pm_wait_for_bit_clear_with_timeout(syscon,
			NCP_SYSCON_PWR_NPWRUPL2HSRAM_ACK, cluster);
	if (!success)
		goto power_up_l2_cleanup;

	switch (cluster) {
	case (0):
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
		break;
	case (1):
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
		break;
	case (2):
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
		break;
	case (3):
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK0_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1, RAM_BANK1_LS_MASK);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK1_MS_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK2_MASK);
		udelay(20);
		pm_set_bits_syscon_register(syscon,
				NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2, RAM_BANK3_MASK);
		udelay(20);
		break;
	default:
		pr_err("Illegal cluster: %d > 3\n", cluster);
	}

	/* Clear the chip select */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_CHIPSELECTEN, mask);

	/* Release the isolation clamps */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_ISOLATEL2MISC, mask);

	/* Enable the ACE bridge */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_ACEPWRDNRQ, mask);

	udelay(16);

power_up_l2_cleanup:

	iounmap(syscon);
}

static void pm_L2_logical_powerup(u32 cluster, u32 cpu) {
	void __iomem *syscon;
	u32 mask = (0x1 << cluster);
	u32 cpu_mask = (0x1 << cpu);

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return;

	/* put the cluster into a cpu hold */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_RESET_AXIS,
			cluster_to_poreset[cluster]);

	/* Power put the cpu into reset */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_RESET_CPU, cpu_mask);

	/* Hold the chip debug cluster */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_HOLD_DBG, mask);

	/* Hold the L2 cluster */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_HOLD_L2, mask);

	/* Cluster physical power up */
	pm_L2_physical_connection_and_power_up(cluster);

	udelay(16);

	/* put the cluster into a cpu hold */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_RESET_AXIS,
			cluster_to_poreset[cluster]);

	udelay(64);

	/* Enable the system counter */
	pm_set_bits_syscon_register(syscon, NCP_SYSCON_PWR_CSYSREQ_CNT, mask);

	/* Release the L2 cluster */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_HOLD_L2, mask);

	pm_dickens_logical_powerup(cluster);

	/* start L2 */
	pm_clear_bits_syscon_register(syscon, NCP_SYSCON_PWR_ACINACTM, mask);

	iounmap(syscon);

	return;

}

void pm_debug_read_pwr_registers(void) {
	u32 reg;

	void __iomem *syscon;

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return;

	reg = readl(syscon + 0x1400);
	pr_err("NCP_SYSCON_PWR_CLKEN: 0x%x\n", reg);
	reg = readl(syscon + 0x1408);
	pr_err("NCP_SYSCON_PWR_ACINACTM: 0x%x\n", reg);
	reg = readl(syscon + 0x140c);
	pr_err("NCP_SYSCON_PWR_CHIPSELECTEN: 0x%x\n", reg);
	reg = readl(syscon + 0x1410);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x1414);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1418);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x141c);
	pr_err("NCP_SYSCON_PWR_CSYSREQ_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1420);
	pr_err("NCP_SYSCON_PWR_PWRUPL2LGCSTG1: 0x%x\n", reg);
	reg = readl(syscon + 0x1424);
	pr_err("NCP_SYSCON_PWR_PWRUPL2LGCSTG2: 0x%x\n", reg);
	reg = readl(syscon + 0x1428);
	pr_err("NCP_SYSCON_PWR_PWRUPL2HSRAM: 0x%x\n", reg);
	reg = readl(syscon + 0x142c);
	pr_err("NCP_SYSCON_PWR_ACEPWRDNRQ: 0x%x\n", reg);
	reg = readl(syscon + 0x1430);
	pr_err("NCP_SYSCON_PWR_ISOLATEL2MIS: 0x%x\n", reg);
	reg = readl(syscon + 0x1438);
	pr_err("NCP_SYSCON_PWR_NPWRUPL2LGCSTG1_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x143c);
	pr_err("NCP_SYSCON_PWR_NPWRUPL2HSRAM_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1440);
	pr_err("NCP_SYSCON_PWR_STANDBYWFIL2: 0x%x\n", reg);
	reg = readl(syscon + 0x1444);
	pr_err("NCP_SYSCON_PWR_CSYSACK_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x1448);
	pr_err("NCP_SYSCON_PWR_CACTIVE_TS: 0x%x\n", reg);
	reg = readl(syscon + 0x144c);
	pr_err("NCP_SYSCON_PWR_CSYSACK_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1450);
	pr_err("NCP_SYSCON_PWR_CACTIVE_CNT: 0x%x\n", reg);
	reg = readl(syscon + 0x1454);
	pr_err("NCP_SYSCON_PWR_CSYSACK_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x1458);
	pr_err("NCP_SYSCON_PWR_CACTIVE_ATB: 0x%x\n", reg);
	reg = readl(syscon + 0x145c);
	pr_err("NCP_SYSCON_PWR_CSYSACK_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1460);
	pr_err("NCP_SYSCON_PWR_CACTIVE_APB: 0x%x\n", reg);
	reg = readl(syscon + 0x1464);
	pr_err("NCP_SYSCON_PWR_NACEPWRDNACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1468);
	pr_err("NCP_SYSCON_PWR_CACTIVEM_EAGM: 0x%x\n", reg);
	reg = readl(syscon + 0x146c);
	pr_err("NCP_SYSCON_PWR_CACTIVEM_EAGS: 0x%x\n", reg);
	reg = readl(syscon + 0x1470);
	pr_err("NCP_SYSCON_PWR_CACTIVES_EAGM: 0x%x\n", reg);
	reg = readl(syscon + 0x1474);
	pr_err("NCP_SYSCON_PWR_CACTIVES_EAGS: 0x%x\n", reg);
	reg = readl(syscon + 0x1480);
	pr_err("NCP_SYSCON_PWR_PWRUPCPUSTG1: 0x%x\n", reg);
	reg = readl(syscon + 0x1484);
	pr_err("NCP_SYSCON_PWR_PWRUPCPUSTG2: 0x%x\n", reg);
	reg = readl(syscon + 0x1488);
	pr_err("NCP_SYSCON_PWR_PWRUPCPURAM: 0x%x\n", reg);
	reg = readl(syscon + 0x148c);
	pr_err("NCP_SYSCON_PWR_ISOLATECPU: 0x%x\n", reg);
	reg = readl(syscon + 0x1490);
	pr_err("NCP_SYSCON_PWR_NPWRUPCPUSTG1_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1494);
	pr_err("NCP_SYSCON_PWR_NPWRUPCPURAM_ACK: 0x%x\n", reg);
	reg = readl(syscon + 0x1498);
	pr_err("NCP_SYSCON_PWR_QACTIVE: 0x%x\n", reg);
	reg = readl(syscon + 0x149C);
	pr_err("NCP_SYSCON_PWR_STANDBYWFI: 0x%x\n", reg);
	reg = readl(syscon + 0x14A0);
	pr_err("NCP_SYSCON_PWR_STANDBYWFE: 0x%x\n", reg);
	reg = readl(syscon + 0x14A4);
	pr_err("NCP_SYSCON_PWR_DBGNOPWRDWN: 0x%x\n", reg);
	reg = readl(syscon + 0x14A8);
	pr_err("NCP_SYSCON_PWR_DBGPWRUPREQ: 0x%x\n", reg);

	iounmap(syscon);
}

void pm_dump_L2_registers(void) {
	u32 reg;

	void __iomem *syscon;

	syscon = ioremap(SYSCON_PHYS_ADDR, SZ_64K);
	if (WARN_ON(!syscon))
		return;
	reg = readl(syscon + 0x1580);
	pr_err("NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x1584);
	pr_err("NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x1588);
	pr_err("NCP_SYSCON_PWR_PWRUPL20RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x158c);
	pr_err("NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x1590);
	pr_err("NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x1594);
	pr_err("NCP_SYSCON_PWR_PWRUPL21RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x1598);
	pr_err("NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x159c);
	pr_err("NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x15a0);
	pr_err("NCP_SYSCON_PWR_PWRUPL22RAM_PWRUPL2RAM0: 0x%x\n", reg);
	reg = readl(syscon + 0x15a4);
	pr_err("NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM2: 0x%x\n", reg);
	reg = readl(syscon + 0x15a8);
	pr_err("NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM1: 0x%x\n", reg);
	reg = readl(syscon + 0x15ac);
	pr_err("NCP_SYSCON_PWR_PWRUPL23RAM_PWRUPL2RAM0: 0x%x\n", reg);

	iounmap(syscon);
}


