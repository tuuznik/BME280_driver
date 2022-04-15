#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
 
static int __init bme280_init(void) { 
    pr_info("Hello world!\n"); 
    return 0; 
} 
 
static void __exit bme280_exit(void) { 
    pr_info("Bye world\n"); 
} 
 
module_init(bme280_init); 
module_exit(bme280_exit); 

MODULE_DESCRIPTION("A driver for BME280 Temperature, Humidity and Air Pressure Sensor");
MODULE_AUTHOR("Agnieszka Tuznik <tuznikagnieszka@gmail.com>"); 
MODULE_LICENSE("GPL"); 