/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
 *  Copyright (C) 2009 Rob Emanuele
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <mach/at91sam9_smc.h>

#include "sam9_smc.h"
#include "generic.h"
#include "../../../drivers/spi/atmel_spi.h"
#include "ov3642.h"

#define ISI_MEM_SIZE 8*1024*1024

static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
	// at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS
//			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD
//			   | ATMEL_UART_RI);
	at91_register_uart(AT91SAM9260_ID_US0, 1, 0);

	/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
	// at91_register_uart(AT91SAM9260_ID_US1, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);

   /* USART3 on ttyS3 (Rx, Tx) */
   // Used for LS 
	at91_register_uart(AT91SAM9260_ID_US2, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);
	at91_register_uart(AT91SAM9260_ID_US3, 3, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};


/*
static struct atmelspimux spi_transceiver_mux[] = {
   // this pin logic is inverted. (1's are pulled low when cs is activated)
   // todo: make sure to clarify that first bit in list is high bit or low bit
   { at91_pin_pc0, 0 },
   { at91_pin_pc1, 1 },
   { at91_pin_pc2, 1 },
   { at91_pin_pc3, 1 },
   { -1, 0 },
};
*/

/*
 * SPI devices.
 */
static struct spi_board_info ek_spi_devices[] = {
   
	{	// Transceiver spidev
		// .modalias	= "spidev",
		.modalias	= "AX5042",
      //.controller_data = &spi_transceiver_mux[0],
      .controller_data = NULL, 
		.chip_select	= 0,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
      // .irq = AT91_PIN_PC2,
      .irq = 29,
	},
	{	// Transceiver spidev
		.modalias	= "spidev",
		// .modalias	= "AX5042",
      //.controller_data = &spi_transceiver_mux[0],
      .controller_data = NULL, 
		.chip_select	= 2,
		.max_speed_hz	= 10 * 1000 * 1000,
		.bus_num	= 1,
      .irq = 0,
	},
   /*
	{	// Another spidev
		.modalias	= "spidev",
      .controller_data = &spi_test_mux[0],
		.chip_select	= 2,
		.max_speed_hz	= 20 * 1000 * 1000,
		.bus_num	= 1,
	}, */
//#if !(defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_AT91))
	{	 //AT45 DataFlash chip 
		// .modalias	= "mtd_dataflash",
      .modalias   = "spidev",
      .controller_data = NULL,
		.chip_select	= 1,
		.max_speed_hz	= 66 * 1000 * 1000,
		.bus_num	= 0,
		.mode = SPI_MODE_0,
	},
//#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
/*	{	//DataFlash card 
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	}, */
//#endif 
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PB0,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name   = "Bootstrap",
		.offset = 0,
		.size   = 4 * SZ_1M,
	},
	{
		.name	= "Partition 1",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 60 * SZ_1M,
	},
	{
		.name	= "Partition 2",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init ek_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}


/*
 * MCI (SD/MMC)
 * wp_pin is not connected
 */
#if defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_ATMELMCI_MODULE)
static struct mci_platform_data __initdata ek_mmc_data = {
	.slot[0] = {
		.bus_width	= 4,
		// .detect_pin	= AT91_PIN_PC2,
		.detect_pin	= -ENODEV, // Turn off detect pin, to match MB
		.wp_pin		= -ENODEV,
	},
	.slot[1] = {
		// .bus_width	= 4,
		.bus_width	= 0, // Disable mmc1
		.detect_pin	= AT91_PIN_PC9,
		.wp_pin		= -ENODEV,
	},

};
#else
static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 1,	/* Only one slot so use slot B */
	.wire4		= 1,
	.det_pin	= AT91_PIN_PC9,
};
#endif

/*
 * LEDs
 */
static struct gpio_led ek_leds[] = {
	{	/* "bottom" led, green, userled1 to be defined */
		.name			= "ds5",
		.gpio			= AT91_PIN_PB8,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* "power" led, yellow */
		.name			= "ds1",
		.gpio			= AT91_PIN_PB9,
		.default_trigger	= "heartbeat",
	}
};

/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
	{
		.gpio		= AT91_PIN_PA30,
		.code		= BTN_3,
		.desc		= "Button 3",
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		.gpio		= AT91_PIN_PA31,
		.code		= BTN_4,
		.desc		= "Button 4",
		.active_low	= 1,
		.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data ek_button_data = {
	.buttons	= ek_buttons,
	.nbuttons	= ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_button_data,
	}
};

static void __init ek_add_device_buttons(void)
{
	at91_set_gpio_input(AT91_PIN_PA30, 1);	/* btn3 */
	at91_set_deglitch(AT91_PIN_PA30, 1);
	at91_set_gpio_input(AT91_PIN_PA31, 1);	/* btn4 */
	at91_set_deglitch(AT91_PIN_PA31, 1);

	platform_device_register(&ek_button_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif

/*
 * I2C devices
 */
static struct at24_platform_data at24c1024 = {
	.byte_len	= (SZ_1M)/ 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};

static struct pca953x_platform_data pca9535 = {
	.gpio_base	= 200,
	.invert		= 0,
};

static struct ov3642_platform_data dev_ov3642_camera_plat_data = {
   .enable_gpio = AT91_PIN_PB19,
   .enable_active_high = 0,
   .name = "ov3642",
   .dataorder = 0x02,
};

static struct i2c_board_info __initdata ek_i2c_devices[] = {
/*	{
		I2C_BOARD_INFO("wm8731", 0x1b),
	},*/
	{ /* 1Mbit EEPROM device,
	   * modified version of at24c512 as found in
           * board-sam9260ek.c
           */
		I2C_BOARD_INFO("at24c1024", 0x50),
		.platform_data = &at24c1024
 
	},
	{ /* Test Device */
      I2C_BOARD_INFO("i2c-0", 0x00),
   },
	{
      I2C_BOARD_INFO("pca9535", 0x26),
		.platform_data = &pca9535
	},
   {
      I2C_BOARD_INFO("ov3642", 0x3C),
      .platform_data = &dev_ov3642_camera_plat_data,
   },
};

static struct i2c_board_info __initdata ek_i2c_devices1[] = {
	{ /* Test Device */
    	  	I2C_BOARD_INFO("i2c-1", 0x00),
		//.platform_data = &at24c1024
   },
};


static void __init ek_board_init(void)
{
   struct sysinfo si;
   unsigned int memsize = 64 * 1024 * 1024;

   /* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* NAND */
	ek_add_device_nand();
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
#if defined(CONFIG_MMC_ATMELMCI) || defined(CONFIG_MMC_ATMELMCI_MODULE)
	at91_add_device_mci(0, &ek_mmc_data);
#else
	at91_add_device_mmc(0, &ek_mmc_data);
#endif
   // Physical address of dedicated ISI DMA memory.  Start of DRAM +
   // Length of DRAM (64Mb) - buffer size (8MB for bootstrap).
   si_meminfo(&si);
   if (si.totalram * si.mem_unit > 64*1024*1024)
      memsize = 128*1024*1024;

   at91_add_device_isi(0x20000000 + memsize - ISI_MEM_SIZE,
         ISI_MEM_SIZE);

	/* I2C */
	at91_add_device_i2c(ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
   
   // Additional Polysat I2C devices
	at91_add_device_i2c(ek_i2c_devices1, ARRAY_SIZE(ek_i2c_devices1));

	/* LEDs */
	// at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	/* Push Buttons */
	ek_add_device_buttons();

   // Audio DAC not necessary for Dev. board
	/* PCK0 provides MCLK to the WM8731 */
   // at91_set_B_periph(AT91_PIN_PC1, 0);
	/* SSC (for WM8731) */
	// at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX);

   // IRQ line for transceiver
   at91_set_gpio_input(AT91_PIN_PC2, 0);
   at91_set_A_periph(AT91_PIN_PC12, 0);
   // gpio_direction_input(AT91_PIN_PC2);

}

MACHINE_START(AT91SAM9G20EK_2MMC, "Atmel AT91SAM9G20-EK 2 MMC Slot Mod")
	/* Maintainer: Rob Emanuele */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
