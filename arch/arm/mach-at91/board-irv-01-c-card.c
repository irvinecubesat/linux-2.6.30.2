#include <linux/module.h>  /* Needed by all modules */
#include <linux/kernel.h>  /* Needed for KERN_INFO */
#include <linux/init.h>    /* Needed for the macros */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>
#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

// #include <mach/board.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <mach/at91sam9_smc.h>
#include <linux/i2c-gpio.h>

#include <../mach-at91/sam9_smc.h>
#include <../mach-at91/generic.h>

MODULE_DESCRIPTION("Irvine C Card Configuration module support for i2c expander");
MODULE_AUTHOR("Maurice Ling");
MODULE_LICENSE("GPL");


#define IRV_CCARD_EXPANDER_BASE 296


/**
 * Setup the gpio pins to proper initialize states.
 * 
 * Ports are set up to be output pins
 *
 * Function    Port  gpio
 * ----------------------
 * DSA1_RLS_B  P0    296
 * DSA1_DPLY_B P1    297
 * DSA2_RLS_B  P2    298
 * DSA2_DPLY_B P3    299
 * MT01_EN     P4    300 <--- set low at initialization
 * MT02_EN     P5    301 <--- set low at initialization
 * MT03_EN     P6    302 <--- set low at initialization
 * DSA_EN_BAR  P7    303
 **/
int pcf857x_setup(struct i2c_client *client,
		    int gpio, unsigned ngpio,
		    void *context)
{
  int result=0;
  int i = gpio;
  for (i = gpio; i < ngpio; i++)
    {
      int setValue=1;
      int status=0;
      if (i >=300 && i <= 302)	/* MT01-03 should be turned off to start*/
	{
	  setValue=0;
	}
      status=gpio_direction_output(gpio, setValue);
      if (status != 0)
	{
	  dev_err(&client->dev, "Error %d setting gpio %d to output\n",
		  status, i);
	  result=-1;
	}
    }
  return result;
}

static struct pcf857x_platform_data irv_ccard_expander_plat_data = {
   .gpio_base = IRV_CCARD_EXPANDER_BASE,
   .setup = pcf857x_setup,
   .n_latch=0
};

struct i2c_info {
   struct i2c_board_info board_info;
   struct i2c_client *client;
};

static struct i2c_info i2c_irv_ccard_devices[] = {
   {
      .client = NULL,
      .board_info = {
         I2C_BOARD_INFO("tca9554a", 0x38),
         .platform_data = &irv_ccard_expander_plat_data,
      }
   },
};
#define NUM_IRV_CCARD_I2C_DEVICES (sizeof(i2c_irv_ccard_devices) / sizeof(i2c_irv_ccard_devices[0]))

static void __exit cleanup_board(void)
{
   int i;
   for (i=0; i <NUM_IRV_CCARD_I2C_DEVICES; i++)
     {
       if (i2c_irv_ccard_devices[i].client) {
         i2c_unregister_device(i2c_irv_ccard_devices[i].client);
         i2c_irv_ccard_devices[i].client = NULL;
       }
     }
}

static int __init setup_board(void)
{
   int i=0;
   struct i2c_adapter *adap = NULL;

   adap = i2c_get_adapter(1);
   if (!adap) {
      printk("No irv_ccard i2c bus adapter!\n");
      return -1;
   }
   printk("Registering irv_ccard i2c bus adapter\n");

   for (i = 0; i < NUM_IRV_CCARD_I2C_DEVICES; i++) {
      i2c_irv_ccard_devices[i].client =
                  i2c_new_device(adap, &i2c_irv_ccard_devices[i].board_info);

      if (NULL == i2c_irv_ccard_devices[i].client) {
         printk("Failed to register irv_ccard i2c device %d!\n", i);
         goto err;
      }
   }

   return 0;
err:

   return -1;
}

module_init(setup_board);
module_exit(cleanup_board);

