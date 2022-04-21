#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/gpio.h>  
 
// static int __init bme280_init(void) { 
//     pr_info("Hello world!\n"); 
//     return 0; 
// } 
 
// static void __exit bme280_exit(void) { 
//     pr_info("Bye world\n"); 
// } 
 
// module_init(bme280_init); 
// module_exit(bme280_exit); 

struct bme280 { 
   struct i2c_client  *client; 
   struct gpio_chip   chip; 
   struct mutex      lock; 
}; 


static const struct of_device_id bme280_of_match[] = { 
        { .compatible = "bme280" }, 
        {} 
}; 
MODULE_DEVICE_TABLE(of, bme280_of_match); 

static struct i2c_device_id bme280_idtable[] = { 
   { "bme280", 0 },
   { }
}; 
 
MODULE_DEVICE_TABLE(i2c, bme280_idtable); 
static struct i2c_driver bme280_driver = { 
   .driver = {
        .name = "bme280", 
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(bme280_of_match),
   }, 
 
   .id_table = bme280_idtable, 
   .probe    = bme280_probe, 
   .remove   = bme280_remove, 
} 

static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    
    struct bme280 *bme280; 

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
        return -EIO; 
    bme280 = devm_kzalloc(&client->dev, sizeof(*bme280), GFP_KERNEL); 
    if (!bme280) 
        return -ENOMEM; 
 

    bme280->client = client; 
    i2c_set_clientdata(client, bme280); 
 
    return gpiochip_add(&bme280->chip); 
}

static int bme280_remove(struct i2c_client *client) {

    struct bme280 *bme280; 
 
    /* We retrieve our private data */ 
    bme280 = i2c_get_clientdata(client); 
 
    /* Which hold gpiochip we want to work on */ 
   return gpiochip_remove(&bme280->chip); 
}

// register the driver with the I2C core
module_i2c_driver(bme280_driver); 

MODULE_DESCRIPTION("A driver for BME280 Temperature, Humidity and Air Pressure Sensor");
MODULE_AUTHOR("Agnieszka Tuznik <tuznikagnieszka@gmail.com>"); 
MODULE_LICENSE("GPL"); 