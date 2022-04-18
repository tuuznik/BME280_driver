#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
 
// static int __init bme280_init(void) { 
//     pr_info("Hello world!\n"); 
//     return 0; 
// } 
 
// static void __exit bme280_exit(void) { 
//     pr_info("Bye world\n"); 
// } 
 
// module_init(bme280_init); 
// module_exit(bme280_exit); 

// TODO: fill the id table

struct bme280 { 
   struct i2c_client  *client; 
   struct gpio_chip   chip; 
   struct mutex      lock; 
}; 

static struct i2c_device_id bme280_idtable[] = { 
   { "bme280", my_id_for_foo }, 
   { "bar", my_id_for_bar }, 
   { } 
}; 
 
MODULE_DEVICE_TABLE(i2c, bme280_idtable); 
static struct i2c_driver bme280_driver = { 
   .driver = {
   .name = "bme280", 
   }, 
 
   .id_table = bme280_idtable, 
   .probe    = bme280_probe, 
   .remove   = bme280_remove, 
} 

static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id) {}

static int bme280_remove(struct i2c_client *client) {}

// register the driver with the I2C core
module_i2c_driver(bme280_driver); 

MODULE_DESCRIPTION("A driver for BME280 Temperature, Humidity and Air Pressure Sensor");
MODULE_AUTHOR("Agnieszka Tuznik <tuznikagnieszka@gmail.com>"); 
MODULE_LICENSE("GPL"); 