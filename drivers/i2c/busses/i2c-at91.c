/*
 *  i2c Support for Atmel's AT91 Two-Wire Interface (TWI)
 *
 *  Copyright (C) 2011 Weinmann Medical GmbH
 *  Author: Nikolaus Voss <n.voss@weinmann.de>
 *
 *  Evolved from original work by:
 *  Copyright (C) 2004 Rick Bronson
 *  Converted to 2.6 by Andrew Victor <andrew@sanpeople.com>
 *
 *  Borrowed heavily from original work by:
 *  Copyright (C) 2000 Philip Edelbrock <phil@stimpy.netroedge.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define TWI_CLK_HZ		100000			/* max 400 Kbits/s */
#define AT91_I2C_TIMEOUT	msecs_to_jiffies(100)	/* transfer timeout */

/* AT91 TWI register definitions */
#define	AT91_TWI_CR		0x0000	/* Control Register */
#define	AT91_TWI_START		0x0001	/* Send a Start Condition */
#define	AT91_TWI_STOP		0x0002	/* Send a Stop Condition */
#define	AT91_TWI_MSEN		0x0004	/* Master Transfer Enable */
#define	AT91_TWI_SVDIS		0x0020	/* Slave Transfer Disable */
#define	AT91_TWI_SWRST		0x0080	/* Software Reset */

#define	AT91_TWI_MMR		0x0004	/* Master Mode Register */
#define	AT91_TWI_IADRSZ_1	0x0100	/* Internal Device Address Size */
#define	AT91_TWI_MREAD		0x1000	/* Master Read Direction */

#define	AT91_TWI_IADR		0x000c	/* Internal Address Register */

#define	AT91_TWI_CWGR		0x0010	/* Clock Waveform Generator Reg */

#define	AT91_TWI_SR		0x0020	/* Status Register */
#define	AT91_TWI_TXCOMP		0x0001	/* Transmission Complete */
#define	AT91_TWI_RXRDY		0x0002	/* Receive Holding Register Ready */
#define	AT91_TWI_TXRDY		0x0004	/* Transmit Holding Register Ready */

#define	AT91_TWI_OVRE		0x0040	/* Overrun Error */
#define	AT91_TWI_UNRE		0x0080	/* Underrun Error */
#define	AT91_TWI_NACK		0x0100	/* Not Acknowledged */

#define	AT91_TWI_IER		0x0024	/* Interrupt Enable Register */
#define	AT91_TWI_IDR		0x0028	/* Interrupt Disable Register */
#define	AT91_TWI_IMR		0x002c	/* Interrupt Mask Register */
#define	AT91_TWI_RHR		0x0030	/* Receive Holding Register */
#define	AT91_TWI_THR		0x0034	/* Transmit Holding Register */

enum twi_ip_id {
	DEFAULT		= 0, /* default ip, no known limitations */
	RM9200		= 1,
	SAM9261		= 2,
};

struct at91_twi_dev {
	struct device		*dev;
	void __iomem		*base;
	struct completion	cmd_complete;
	struct clk		*clk;
	u8			*buf;
	size_t			buf_len;
	struct i2c_msg		*msg;
	int			irq;
	unsigned		transfer_status;
	struct i2c_adapter	adapter;
	enum twi_ip_id		ip_id;
	unsigned		twi_cwgr_reg;
};

static unsigned at91_twi_read(struct at91_twi_dev *dev, unsigned reg)
{
	return readl_relaxed(dev->base + reg);
}

static void at91_twi_write(struct at91_twi_dev *dev, unsigned reg, unsigned val)
{
	// writel_relaxed(val, dev->base + reg);
   // JMB changed
	writel(val, dev->base + reg);
}

static void at91_disable_twi_interrupts(struct at91_twi_dev *dev)
{
	at91_twi_write(dev, AT91_TWI_IDR,
		       AT91_TWI_TXCOMP | AT91_TWI_RXRDY | AT91_TWI_TXRDY);
}

static void at91_init_twi_bus(struct at91_twi_dev *dev)
{
	at91_disable_twi_interrupts(dev);
	at91_twi_write(dev, AT91_TWI_CR, AT91_TWI_SWRST);
	at91_twi_write(dev, AT91_TWI_CR, AT91_TWI_MSEN);
	at91_twi_write(dev, AT91_TWI_CR, AT91_TWI_SVDIS);
	at91_twi_write(dev, AT91_TWI_CWGR, dev->twi_cwgr_reg);
}

/*
 * Calculate symmetric clock as stated in datasheet:
 * twi_clk = F_MAIN / (2 * (cdiv * (1 << ckdiv) + offset))
 */
static void __devinit at91_calc_twi_clock(struct at91_twi_dev *dev, int twi_clk)
{
	int ckdiv, cdiv, div;
	int offset = 4;
	int max_ckdiv = 7;

	if (dev->ip_id == RM9200) {
		offset = 3;
		max_ckdiv = 5;
	} else if (dev->ip_id == SAM9261) {
		offset = 4;
		max_ckdiv = 5;
	}

	div = max(0, (int)DIV_ROUND_UP(clk_get_rate(dev->clk),
				       2 * twi_clk) - offset);
	ckdiv = fls(div >> 8);
	cdiv = div >> ckdiv;

	if (ckdiv > max_ckdiv) {
		dev_warn(dev->dev, "%d exceeds ckdiv max value which is %d.\n",
			 ckdiv, max_ckdiv);
		ckdiv = max_ckdiv;
		cdiv = 255;
	}

	dev->twi_cwgr_reg = (ckdiv << 16) | (cdiv << 8) | cdiv;
	dev_dbg(dev->dev, "cdiv %d ckdiv %d\n", cdiv, ckdiv);
}

static void at91_twi_write_next_byte(struct at91_twi_dev *dev)
{
	if (dev->buf_len <= 0)
		return;

	at91_twi_write(dev, AT91_TWI_THR, *dev->buf);

	/* send stop when last byte has been written */
	if (--dev->buf_len == 0)
		at91_twi_write(dev, AT91_TWI_CR, AT91_TWI_STOP);

	dev_dbg(dev->dev, "wrote 0x%x, to go %d\n", *dev->buf, dev->buf_len);

	++dev->buf;
}

static void at91_twi_read_next_byte(struct at91_twi_dev *dev)
{
	if (dev->buf_len <= 0)
		return;

	*dev->buf = at91_twi_read(dev, AT91_TWI_RHR) & 0xff;
	--dev->buf_len;

	/* handle I2C_SMBUS_BLOCK_DATA */
	if (unlikely(dev->msg->flags & I2C_M_RECV_LEN)) {
		dev->msg->flags &= ~I2C_M_RECV_LEN;
		dev->buf_len += *dev->buf;
		dev->msg->len = dev->buf_len + 1;
		dev_dbg(dev->dev, "received block length %d\n", dev->buf_len);
	}

	/* send stop if second but last byte has been read */
	if (dev->buf_len == 1)
		at91_twi_write(dev, AT91_TWI_CR, AT91_TWI_STOP);

	dev_dbg(dev->dev, "read 0x%x, to go %d\n", *dev->buf, dev->buf_len);

	++dev->buf;
}

static irqreturn_t atmel_twi_interrupt(int irq, void *dev_id)
{
	struct at91_twi_dev *dev = dev_id;
	const unsigned status = at91_twi_read(dev, AT91_TWI_SR);
	const unsigned irqstatus = status & at91_twi_read(dev, AT91_TWI_IMR);

	if (!irqstatus)
		return IRQ_NONE;
	else if (irqstatus & AT91_TWI_RXRDY)
		at91_twi_read_next_byte(dev);
	else if (irqstatus & AT91_TWI_TXRDY)
		at91_twi_write_next_byte(dev);

	/* catch error flags */
	dev->transfer_status |= status;

	if (irqstatus & AT91_TWI_TXCOMP) {
		at91_disable_twi_interrupts(dev);
		complete(&dev->cmd_complete);
	}

	return IRQ_HANDLED;
}

static int at91_do_twi_transfer(struct at91_twi_dev *dev)
{
	int ret;

	dev_dbg(dev->dev, "transfer: %s %d bytes.\n",
		(dev->msg->flags & I2C_M_RD) ? "read" : "write", dev->buf_len);

	INIT_COMPLETION(dev->cmd_complete);
	dev->transfer_status = 0;
	if (dev->msg->flags & I2C_M_RD) {
		unsigned start_flags = AT91_TWI_START;

		if (at91_twi_read(dev, AT91_TWI_SR) & AT91_TWI_RXRDY) {
			dev_err(dev->dev, "RXRDY still set!");
			at91_twi_read(dev, AT91_TWI_RHR);
		}

		/* if only one byte is to be read, immediately stop transfer */
		if (dev->buf_len <= 1 && !(dev->msg->flags & I2C_M_RECV_LEN))
			start_flags |= AT91_TWI_STOP;
		at91_twi_write(dev, AT91_TWI_CR, start_flags);
		at91_twi_write(dev, AT91_TWI_IER,
			       AT91_TWI_TXCOMP | AT91_TWI_RXRDY);
	} else {
		at91_twi_write_next_byte(dev);
		at91_twi_write(dev, AT91_TWI_IER,
			       AT91_TWI_TXCOMP | AT91_TWI_TXRDY);
	}

	ret = wait_for_completion_interruptible_timeout(&dev->cmd_complete,
							dev->adapter.timeout);
	if (ret == 0) {
		dev_err(dev->dev, "controller timed out\n");
		at91_init_twi_bus(dev);
		return -ETIMEDOUT;
	}
	if (dev->transfer_status & AT91_TWI_NACK) {
		dev_dbg(dev->dev, "received nack\n");
		return -EREMOTEIO;
	}
	if (dev->transfer_status & AT91_TWI_OVRE) {
		dev_err(dev->dev, "overrun while reading\n");
		return -EIO;
	}
	if (dev->transfer_status & AT91_TWI_UNRE && dev->ip_id == RM9200) {
		dev_err(dev->dev, "underrun while writing\n");
		return -EIO;
	}
	dev_dbg(dev->dev, "transfer complete\n");

	return 0;
}

static int at91_twi_xfer(struct i2c_adapter *adap, struct i2c_msg *msg, int num)
{
	struct at91_twi_dev *dev = i2c_get_adapdata(adap);
	int ret;
	unsigned int_addr_flag = 0;
	struct i2c_msg *m_start = msg;

	dev_dbg(&adap->dev, "at91_xfer: processing %d messages:\n", num);

	/*
	 * The hardware can handle at most two messages concatenated by a
	 * repeated start via it's internal address feature.
	 */
	if (num > 2) {
		dev_err(dev->dev,
			"cannot handle more than two concatenated messages.\n");
		return 0;
	} else if (num == 2) {
		int internal_address = 0;
		int i;

		if (msg->flags & I2C_M_RD) {
			dev_err(dev->dev, "first transfer must be write.\n");
			return -EINVAL;
		}
		if (msg->len > 3) {
			dev_err(dev->dev, "first message size must be <= 3.\n");
			return -EINVAL;
		}

		/* 1st msg is put into the internal address, start with 2nd */
		m_start = &msg[1];
		for (i = 0; i < msg->len; ++i) {
			const unsigned addr = msg->buf[msg->len - 1 - i];

			internal_address |= addr << (8 * i);
			int_addr_flag += AT91_TWI_IADRSZ_1;
		}
		at91_twi_write(dev, AT91_TWI_IADR, internal_address);
	}

	at91_twi_write(dev, AT91_TWI_MMR, (m_start->addr << 16) | int_addr_flag
		       | ((m_start->flags & I2C_M_RD) ? AT91_TWI_MREAD : 0));

	dev->buf_len = m_start->len;
	dev->buf = m_start->buf;
	dev->msg = m_start;

	ret = at91_do_twi_transfer(dev);

	return (ret < 0) ? ret : num;
}

static u32 at91_twi_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL
		| I2C_FUNC_SMBUS_READ_BLOCK_DATA;
}

static struct i2c_algorithm at91_twi_algorithm = {
	.master_xfer	= at91_twi_xfer,
	.functionality	= at91_twi_func,
};

static int __devinit at91_twi_probe(struct platform_device *pdev)
{
	struct at91_twi_dev *dev;
	struct resource *mem, *ioarea;
	int irq, rc;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ioarea = request_mem_region(mem->start, resource_size(mem), pdev->name);
	if (!ioarea)
		return -EBUSY;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		rc = -ENOMEM;
		goto err_release_region;
	}

	if (pdev->id_entry)
		dev->ip_id = pdev->id_entry->driver_data;

	init_completion(&dev->cmd_complete);

	dev->dev = &pdev->dev;
	dev->irq = irq;
	platform_set_drvdata(pdev, dev);

	dev->clk = clk_get(dev->dev, "twi_clk");
	if (IS_ERR(dev->clk)) {
		dev_err(dev->dev, "no clock defined\n");
		rc = -ENODEV;
		goto err_free_mem;
	}
#ifdef CONFIG_HAVE_CLK_PREPARE
	clk_prepare(dev->clk);
#endif
	clk_enable(dev->clk);

	dev->base = ioremap(mem->start, resource_size(mem));
	if (!dev->base) {
		rc = -EBUSY;
		goto err_mem_ioremap;
	}

	at91_calc_twi_clock(dev, TWI_CLK_HZ);
	at91_init_twi_bus(dev);

	rc = request_irq(dev->irq, atmel_twi_interrupt, 0,
			 dev_name(dev->dev), dev);
	if (rc) {
		dev_err(dev->dev, "Cannot get irq %d: %d\n", dev->irq, rc);
		goto err_unuse_clocks;
	}

	snprintf(dev->adapter.name, sizeof(dev->adapter.name), "AT91");
	i2c_set_adapdata(&dev->adapter, dev);
	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &at91_twi_algorithm;
	dev->adapter.dev.parent = dev->dev;
	dev->adapter.nr = pdev->id;
	dev->adapter.timeout = AT91_I2C_TIMEOUT;

	rc = i2c_add_numbered_adapter(&dev->adapter);
	if (rc) {
		dev_err(dev->dev, "Adapter %s registration failed\n",
			dev->adapter.name);
		goto err_free_irq;
	}

	dev_info(dev->dev, "AT91 i2c bus driver.\n");
	return 0;

err_free_irq:
	free_irq(dev->irq, dev);
err_unuse_clocks:
	iounmap(dev->base);
err_mem_ioremap:
	clk_disable(dev->clk);
#ifdef CONFIG_HAVE_CLK_PREPARE
	clk_unprepare(dev->clk);
#endif
	clk_put(dev->clk);
err_free_mem:
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, resource_size(mem));

	return rc;
}

static int __devexit at91_twi_remove(struct platform_device *pdev)
{
	struct at91_twi_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;
	int rc;

	rc = i2c_del_adapter(&dev->adapter);
	clk_disable(dev->clk);
#ifdef CONFIG_HAVE_CLK_PREPARE
	clk_unprepare(dev->clk);
#endif
	clk_put(dev->clk);
	free_irq(dev->irq, dev);
	iounmap(dev->base);
	kfree(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	return rc;
}

#ifdef CONFIG_PM

#ifdef CONFIG_PM_RUNTIME
static int at91_twi_runtime_suspend(struct device *dev)
{
	struct at91_twi_dev *twi_dev = dev_get_drvdata(dev);

	clk_disable(twi_dev->clk);

	return 0;
}

static int at91_twi_runtime_resume(struct device *dev)
{
	struct at91_twi_dev *twi_dev = dev_get_drvdata(dev);

	return clk_enable(twi_dev->clk);
}
#endif

static struct dev_pm_ops at91_twi_pm = {
// JMB: Runtimte power management not supported in this version of the kernel
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend	= at91_twi_runtime_suspend,
	.runtime_resume		= at91_twi_runtime_resume,
#endif
};

#define at91_twi_pm_ops (&at91_twi_pm)
#else
#define at91_twi_pm_ops NULL
#endif

static struct platform_driver at91_twi_driver = {
	.probe		= at91_twi_probe,
	.remove		= __devexit_p(at91_twi_remove),
	.driver		= {
		.name	= "at91_i2c",
		.owner	= THIS_MODULE,
		.pm	= at91_twi_pm_ops,
	},
};

static int __init at91_twi_init(void)
{
	return platform_driver_register(&at91_twi_driver);
}

static void __exit at91_twi_exit(void)
{
	platform_driver_unregister(&at91_twi_driver);
}

module_init(at91_twi_init);
module_exit(at91_twi_exit);

MODULE_AUTHOR("Nikolaus Voss <n.voss@weinmann.de>");
MODULE_DESCRIPTION("I2C (TWI) driver for Atmel AT91");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:at91_i2c");
