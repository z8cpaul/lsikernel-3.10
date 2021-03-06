/*
 * ACP3400 I2C adapter
 *
 * Based on DU-TS I2C Adapter Driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/of_i2c.h>
#include <linux/slab.h>

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define DRV_NAME "acp3400-i2c"

struct acp3400_i2c_regs {
	unsigned int txconfig;
	unsigned int rxconfig;
	unsigned int txstatus;
	unsigned int rxstatus;
	unsigned int irqenable;
	unsigned int irqclear;
	unsigned int irqstatus;
	unsigned int clkconfig;
	unsigned int startsetup;
	unsigned int stopsetup;
	unsigned int datasetup;
	unsigned int bypassmode;
	unsigned int slaveaddr;
	unsigned int txdata0;
	unsigned int txdata1;
	unsigned int rxdata0;
	unsigned int rxdata1;
};

struct acp3400_i2c_timer_regs {
	unsigned int loadval; /* 0x20 */
	unsigned int val;     /* 0x24 */
	unsigned int control; /* 0x28 */
	unsigned int irq_clr; /* 0x2c */
	unsigned int irq_stat_raw; /* 0x30 */
	unsigned int irq_stat; /* 0x34 */
	unsigned int bg_loadval; /* 0x38 */
};


/* Master Clock Configuration */
/* Configured clock frequency i2c_freq = 100kHz. */

/* I2C register values */
#define ACPI2C_CLK_100KHZ	(1000 | (1000 << 16))
#define ACPI2C_MSTSHC		(940 | (800 << 16))
#define ACPI2C_MSPSHC		(800 | (0 << 16))
#define ACPI2C_MDSHC		(255 | (127 << 16))

#define ACPI2C_XFER_START		0x00000001
#define ACPI2C_XFER_DONE		0x00000001
#define ACPI2C_READ_MODE		0x00000200
#define ACPI2C_STOP_MODE		0x20000000
#define ACPI2C_MASTER_MODE		0x00000100
#define ACPI2C_MASTER_OP_CLEAR		0x00000400
#define ACPI2C_10BIT_ADDR		0x00000080
#define ACPI2C_CLEAR_IRQ		0x0000000F
#define ACPI2C_DELAY			500 /* us */
#define ACPI2C_RETRIES			100
#define ACPI2C_REG_BSIZE		4 /* bytes */
#define ACPI2C_DATA_REGS		2

struct acp3400_i2c {
	struct device *dev;
	struct i2c_adapter adap;
	struct acp3400_i2c_regs __iomem *i2c_regs;
	struct acp3400_i2c_timer_regs __iomem *timer_regs;
	struct mutex i2c_lock;
};

#ifdef ACP3400_I2C_DEBUG
static void dump_regs(struct acp3400_i2c *i2c)
{
	pr_info("i2c-reg: txconfig    %8.8x\n",
		 in_le32(&i2c->i2c_regs->txconfig));
	pr_info("i2c-reg: rxconfig    %8.8x\n",
		 in_le32(&i2c->i2c_regs->rxconfig));
	pr_info("i2c-reg: txstatus    %8.8x\n",
		 in_le32(&i2c->i2c_regs->txstatus));
	pr_info("i2c-reg: rxstatus    %8.8x\n",
		 in_le32(&i2c->i2c_regs->rxstatus));
	pr_info("i2c-reg: irqenable   %8.8x\n",
		 in_le32(&i2c->i2c_regs->irqenable));
	pr_info("i2c-reg: irqclear    %8.8x\n",
		 in_le32(&i2c->i2c_regs->irqclear));
	pr_info("i2c-reg: irqstatus   %8.8x\n",
		 in_le32(&i2c->i2c_regs->irqstatus));
	pr_info("i2c-reg: clkconfig   %8.8x\n",
		 in_le32(&i2c->i2c_regs->clkconfig));
	pr_info("i2c-reg: startsetup  %8.8x\n",
		 in_le32(&i2c->i2c_regs->startsetup));
	pr_info("i2c-reg: stopsetup   %8.8x\n",
		 in_le32(&i2c->i2c_regs->stopsetup));
	pr_info("i2c-reg: datasetup   %8.8x\n",
		 in_le32(&i2c->i2c_regs->datasetup));
	pr_info("i2c-reg: bypassmode  %8.8x\n",
		 in_le32(&i2c->i2c_regs->bypassmode));
	pr_info("i2c-reg: slaveaddr   %8.8x\n",
		 in_le32(&i2c->i2c_regs->slaveaddr));
	pr_info("i2c-reg: txdata0     %8.8x\n",
		 in_le32(&i2c->i2c_regs->txdata0));
	pr_info("i2c-reg: txdata1     %8.8x\n",
		 in_le32(&i2c->i2c_regs->txdata1));
	pr_info("i2c-reg: rxdata0     %8.8x\n",
		 in_le32(&i2c->i2c_regs->rxdata0));
	pr_info("i2c-reg: rxdata1     %8.8x\n",
		 in_le32(&i2c->i2c_regs->rxdata1));
	pr_info("i2c-timer-reg: loadval %8.8x\n",
		 in_le32(&i2c->timer_regs->loadval));
	pr_info("i2c-timer-reg: val     %8.8x\n",
		 in_le32(&i2c->timer_regs->val));
	pr_info("i2c-timer-reg: control %8.8x\n",
		 in_le32(&i2c->timer_regs->control));
}
#endif
/*
 * Low level write routine
 */
static int acp3400_i2c_write_bytes(struct acp3400_i2c *i2c,
				   struct i2c_msg *msgs)
{
	unsigned char *bufp = msgs->buf;
	unsigned int reg_value, data[2] = {0, 0};
	int cnt, ret = 0;

	if (msgs->len > (ACPI2C_REG_BSIZE * ACPI2C_DATA_REGS))
		msgs->len = ACPI2C_REG_BSIZE * ACPI2C_DATA_REGS;

	/* Set message */
	for (cnt = 0; cnt < msgs->len; cnt++) {
		data[1] <<= 8;
		data[1] |= ((data[0] >> 24) & 0xFF);
		data[0] <<= 8;
		data[0] |= bufp[cnt];
	}
	out_le32(&i2c->i2c_regs->txdata0, data[0]);
	out_le32(&i2c->i2c_regs->txdata1, data[1]);

	/* setup and start a transmission */
	reg_value = ACPI2C_MASTER_MODE | ACPI2C_STOP_MODE;
	reg_value |= (msgs->len << 1) & 0x1e;
	if (msgs->flags & I2C_M_TEN) {
		reg_value |= ACPI2C_10BIT_ADDR;
		/* TODO update slave address accordingly */
	}
	out_le32(&i2c->i2c_regs->txconfig, reg_value);

	reg_value &= ~ACPI2C_STOP_MODE;
	out_le32(&i2c->i2c_regs->txconfig, reg_value);

	reg_value |= ACPI2C_XFER_START;
	out_le32(&i2c->i2c_regs->txconfig, reg_value);

	/* Check if the message has been sent
	 * Wait a totally of 1 s for the transmission */
	reg_value = cnt = 0;
	while (0 == reg_value && cnt++ < ACPI2C_RETRIES) {
		udelay(ACPI2C_DELAY);
		/* Read transmission status */
		reg_value = in_le32(&i2c->i2c_regs->txstatus);
	}
#ifdef ACP3400_I2C_DEBUG
	if (ACPI2C_XFER_DONE != reg_value)
		dump_regs(i2c);
#endif
	/* Clear registers */
	out_le32(&i2c->i2c_regs->irqclear, ACPI2C_CLEAR_IRQ);
	out_le32(&i2c->i2c_regs->txconfig, ACPI2C_MASTER_OP_CLEAR);

	out_le32(&i2c->i2c_regs->txconfig, ACPI2C_STOP_MODE);

	if (ACPI2C_XFER_DONE == reg_value)
		ret = msgs->len;
	else
		ret = -EIO;

	return ret;
}

/*
 * Low level read routine
 */

static int acp3400_i2c_read_bytes(struct acp3400_i2c *i2c,
		struct i2c_msg *msgs)
{
	unsigned char *bufp = msgs->buf;
	unsigned int reg_value, data[2];
	int cnt, ret = 0;

	if (msgs->len > (ACPI2C_REG_BSIZE * ACPI2C_DATA_REGS))
		msgs->len = ACPI2C_REG_BSIZE * ACPI2C_DATA_REGS;

	/* Setup a reception */
	reg_value = (msgs->len << 1) & 0x1e;
	if (msgs->flags & I2C_M_TEN) {
		reg_value |= ACPI2C_10BIT_ADDR;
		/* TODO update slave address accordingly */
	}
	out_le32(&i2c->i2c_regs->rxconfig, reg_value);

	/* set read mode and start clock */
	reg_value |= ACPI2C_XFER_START;
	out_le32(&i2c->i2c_regs->rxconfig, reg_value);

	reg_value = ACPI2C_STOP_MODE | ACPI2C_MASTER_MODE | ACPI2C_READ_MODE;
	out_le32(&i2c->i2c_regs->txconfig, reg_value);

	reg_value &= ~ACPI2C_STOP_MODE;
	out_le32(&i2c->i2c_regs->txconfig, reg_value);

	reg_value |= ACPI2C_XFER_START;
	out_le32(&i2c->i2c_regs->txconfig, reg_value);

	/* Check if the message has been received
	 * Wait a totally of 1 s for the reception */
	reg_value = cnt = 0;
	while (0 == (ACPI2C_XFER_DONE & reg_value) &&
	       cnt++ < ACPI2C_RETRIES) {
		udelay(ACPI2C_DELAY);
		/* Read transmission status */
		reg_value = in_le32(&i2c->i2c_regs->rxstatus);
	}

	/* get message */
	data[0] = in_le32(&i2c->i2c_regs->rxdata0);
	data[1] = in_le32(&i2c->i2c_regs->rxdata1);
	for (cnt = 0; cnt < msgs->len; cnt++) {
		if (cnt < ACPI2C_REG_BSIZE)
			bufp[cnt] = data[0] >> ((8 * cnt) & 0xFF);
		else
			bufp[cnt] = data[1] >>
				((8 * (cnt - ACPI2C_REG_BSIZE)) & 0xFF);
	}
#ifdef ACP3400_I2C_DEBUG
	if (ACPI2C_XFER_DONE != (reg_value & 0x03))
		dump_regs(i2c);
#endif
	/* clear registers */
	out_le32(&i2c->i2c_regs->irqclear, ACPI2C_CLEAR_IRQ);
	out_le32(&i2c->i2c_regs->txconfig, ACPI2C_MASTER_OP_CLEAR);

	out_le32(&i2c->i2c_regs->txconfig, ACPI2C_STOP_MODE);

	if (ACPI2C_XFER_DONE == (reg_value & 0x03))
		ret = msgs->len;
	else
		ret = -EIO;

	return ret;
}

/*
 * I2C timer setup
 */
static void acp3400_i2c_timer_setup(struct acp3400_i2c *i2c)
{
	/* disable timer 1 */
	out_le32(&i2c->timer_regs->control, 0);
	/* Program the Timer1 Load Value register with a value that sets
	 * the timer period to 250 ns (that is, 4 MHz frequency). When you
	 * configure the ACP peripheral clock (clk_per) for 200 MHz, the
	 * Timer1 Load Value is 0x31. */
	out_le32(&i2c->timer_regs->loadval, 0x31);
	out_le32(&i2c->timer_regs->bg_loadval, 0x31);

	/* Configure and enable Timer1 for periodic wrapping mode
	 * with a prescaler of 1 by writing 0xc0 to the Timer1 Control
	 * Register. */
	out_le32(&i2c->timer_regs->control, 0xc0);
}
/*
 * Low level master transfer routine
 */
static int acp3400_i2c_xfer_bytes(struct acp3400_i2c *i2c,
		struct i2c_msg *msgs)
{
	int ret = 0;

	mutex_lock(&i2c->i2c_lock);
	/* Prepare ACP3400 I2C for a transaction */
	out_le32(&i2c->i2c_regs->txconfig,
		 ACPI2C_MASTER_OP_CLEAR | ACPI2C_MASTER_MODE);

	out_le32(&i2c->i2c_regs->txconfig,
		 ACPI2C_MASTER_MODE | ACPI2C_STOP_MODE);

	/* I2C clock frequency and duty cycle */
	out_le32(&i2c->i2c_regs->clkconfig, ACPI2C_CLK_100KHZ);
	/* The setup and hold durations for the START condition. */
	out_le32(&i2c->i2c_regs->startsetup, ACPI2C_MSTSHC);
	/* The setup and hold durations for the STOP condition. */
	out_le32(&i2c->i2c_regs->stopsetup, ACPI2C_MSPSHC);
	/* The setup and hold durations for the data bits. */
	out_le32(&i2c->i2c_regs->datasetup, ACPI2C_MDSHC);
	/* Set Slave Address */
	out_le32(&i2c->i2c_regs->slaveaddr, msgs->addr);
	/* Disable the actions for which the host requires to be interrupted */
	out_le32(&i2c->i2c_regs->irqenable, 0);

	/* Send/Receive Data */
	if (msgs->flags & I2C_M_RD)
		ret = acp3400_i2c_read_bytes(i2c, msgs);
	else
		ret = acp3400_i2c_write_bytes(i2c, msgs);

	mutex_unlock(&i2c->i2c_lock);
	return ret;
}
static void acp3400_i2c_dummy_xfer(struct acp3400_i2c *i2c)
{
	/* Prepare ACP3400 I2C for a transaction */
	out_le32(&i2c->i2c_regs->txconfig,
		 ACPI2C_MASTER_OP_CLEAR | ACPI2C_MASTER_MODE);

	out_le32(&i2c->i2c_regs->txconfig,
		 ACPI2C_MASTER_MODE | ACPI2C_STOP_MODE);

	/* I2C clock frequency and duty cycle */
	out_le32(&i2c->i2c_regs->clkconfig, ACPI2C_CLK_100KHZ);
	/* The setup and hold durations for the START condition. */
	out_le32(&i2c->i2c_regs->startsetup, ACPI2C_MSTSHC);
	/* The setup and hold durations for the STOP condition. */
	out_le32(&i2c->i2c_regs->stopsetup, ACPI2C_MSPSHC);
	/* The setup and hold durations for the data bits. */
	out_le32(&i2c->i2c_regs->datasetup, ACPI2C_MDSHC);
	/* Set Dummy Slave Address */
	out_le32(&i2c->i2c_regs->slaveaddr, 0x7f);
	/* Disable the actions for which the host requires to be interrupted */
	out_le32(&i2c->i2c_regs->irqenable, 0);
	/* Number of bytes 0, clear stop mode */
	out_le32(&i2c->i2c_regs->txconfig, ACPI2C_MASTER_MODE);

	/* Set Transmit Ready - triggers the transmit transaction */
	out_le32(&i2c->i2c_regs->txconfig,
		 (ACPI2C_XFER_START | ACPI2C_MASTER_MODE));

}

/*
 * Generic master transfer entrypoint.
 * Returns the number of processed messages or error (<0)
 */
static int acp3400_i2c_xfer(struct i2c_adapter *adap,
		struct i2c_msg *msgs, int num)
{
	struct acp3400_i2c *i2c = i2c_get_adapdata(adap);
	int msg_cnt, ret = 0;

	if (!num)
		return 0;

#ifdef ACP3400_I2C_DEBUG
	if (num == 1 && msgs[0].addr == 0x7f && msgs[0].len == 0) {
		mutex_lock(&i2c->i2c_lock);
		acp3400_i2c_dummy_xfer(i2c);
		mutex_unlock(&i2c->i2c_lock);
		return 0;
	}
#endif
	/*
	 * Check the sanity of the passed messages.
	 * Uhh, generic i2c layer is more suitable place for such code...
	 */
	if ((msgs[0].addr > 0x3ff) ||
	    (!(msgs[0].flags & I2C_M_TEN) && (msgs[0].addr > 0x7f)))
		return -EINVAL;

	for (msg_cnt = 0; msg_cnt < num; ++msg_cnt) {
		if (msgs[msg_cnt].len <= 0)
			return -EINVAL;
		if ((msgs[0].addr != msgs[msg_cnt].addr) ||
		    ((msgs[0].flags & I2C_M_TEN) !=
			(msgs[msg_cnt].flags & I2C_M_TEN)))
			return -EINVAL;
	}

	/* Do real transfer */
	for (msg_cnt = 0; msg_cnt < num; msg_cnt++)
		ret = acp3400_i2c_xfer_bytes(i2c, &msgs[msg_cnt]);
	return ret < 0 ? ret : num;
}

static u32 acp3400_i2c_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR;
}

static const struct i2c_algorithm acp3400_i2c_algo = {
	.master_xfer = acp3400_i2c_xfer,
	.functionality = acp3400_i2c_functionality,
};

static struct i2c_adapter acp3400_i2c_ops = {
	.owner = THIS_MODULE,
	.name = "ACP3400 adapter",
	.class = I2C_CLASS_HWMON | I2C_CLASS_SPD,
	.algo = &acp3400_i2c_algo,
	.timeout = HZ,
};

static int acp34xx_i2c_probe(struct platform_device *dev)
{
	struct device_node *np = dev->dev.of_node;

	struct acp3400_i2c *i2c;
	int result = -ENODEV;
	const u32 *field;

	if (!np)
		return -ENODEV;

	field = of_get_property(np, "enabled", NULL);
	if (!field || (field && (0 == *field)))
		return -EINVAL;

	i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		goto err;

	i2c->i2c_regs = of_iomap(np, 0);
	i2c->timer_regs = of_iomap(np, 1);
	if (!i2c->i2c_regs || !i2c->timer_regs) {
		pr_err("%s: failed to map I/O\n", np->full_name);
		goto err;
	}

	i2c->adap = acp3400_i2c_ops;
	i2c_set_adapdata(&i2c->adap, i2c);
	i2c->adap.dev.of_node = of_node_get(np);
	mutex_init(&i2c->i2c_lock);

	/* I2C timer setup */
	acp3400_i2c_timer_setup(i2c);
	acp3400_i2c_dummy_xfer(i2c);
	result = i2c_add_adapter(&i2c->adap);
	if (result < 0) {
		pr_err("%s: failed to add adapter\n",
				np->full_name);
		goto err;
	}

	pr_info("%s: adapter has been added\n", np->full_name);

	of_i2c_register_devices(&i2c->adap);

	dev_set_drvdata(&dev->dev, i2c);
	return 0;
err:
	if (i2c) {
		if (i2c->i2c_regs)
			iounmap(i2c->i2c_regs);
		if (i2c->timer_regs)
			iounmap(i2c->timer_regs);
		kfree(i2c);
	}

	return result;
}


static int acp34xx_i2c_remove(struct platform_device *dev)
{
	struct acp3400_i2c *i2c = dev_get_drvdata(&dev->dev);

	i2c_del_adapter(&i2c->adap);
	kfree(i2c);

	return 0;
}

static struct of_device_id acp_i2c_match[] = {
	{
		.compatible = "acp-i2c",
	},
	{
		.compatible = "acp,acp3400-i2c",
	},
	{ /* end of list */ },
};

static struct platform_driver acp_i2c_driver = {
	.driver = {
		.name = "acp-i2c",
		.owner = THIS_MODULE,
		.of_match_table = acp_i2c_match,
	},
	.probe		= acp34xx_i2c_probe,
	.remove		= acp34xx_i2c_remove,
};

module_platform_driver(acp_i2c_driver);

MODULE_AUTHOR("Andrey Panteleev <andrey.xx.panteleev@ericsson.com>");
MODULE_DESCRIPTION("I2C adapter for ACP3400");
MODULE_LICENSE("GPL");
