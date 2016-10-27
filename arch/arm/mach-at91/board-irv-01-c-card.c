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


#define DBB_EXPANDER_BASE 296

/**
 * Setup the expander control register
 **/
int pcf857x_setup(struct i2c_client *client,
		  int gpio, unsigned ngpio,
		  void *context)
{
}

static struct pcf857x_platform_data dbb_expander_plat_data = {
   .gpio_base = DBB_EXPANDER_BASE,
   .invert = 0,
};

struct i2c_info {
   struct i2c_board_info board_info;
   struct i2c_client *client;
};

static struct i2c_info i2c_dbb_devices[] = {
   {
      .client = NULL,
      .board_info = {
         I2C_BOARD_INFO("tca9554a", 0x70),
         .platform_data = &dbb_expander_plat_data,
      },
   },
};
#define NUM_DBB_I2C_DEVICES (sizeof(i2c_dbb_devices) / sizeof(i2c_dbb_devices[0]))

static void __exit cleanup_board(void)
{
   int i;

   for (i = 0; i < NUM_DBB_I2C_DEVICES; i++) {
      if (i2c_dbb_devices[i].client) {
         i2c_unregister_device(i2c_dbb_devices[i].client);
         i2c_dbb_devices[i].client = NULL;
      }
   }
}

static int __init setup_board(void)
{
   int i;
   struct i2c_adapter *adap = NULL;
   int res;

   adap = i2c_get_adapter(1);
   if (!adap) {
      printk("No dbb i2c bus adapter!\n");
      return -1;
   }

   for (i = 0; i < NUM_DBB_I2C_DEVICES; i++) {
      i2c_dbb_devices[i].client =
                  i2c_new_device(adap, &i2c_dbb_devices[i].board_info);

      if (NULL == i2c_dbb_devices[i].client) {
         printk("Failed to register dbb i2c device %d!\n", i);
         goto err;
      }
   }

   return 0;
err:

   cleanup_board();
   return -1;
}

module_init(setup_board);
module_exit(cleanup_board);

