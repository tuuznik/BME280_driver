#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/gpio.h> 
#include <linux/i2c.h> 
 
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

static ssize_t bme280_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 99);
}

static DEVICE_ATTR(temperature,S_IRUSR,bme280_temp_show,NULL);

static ssize_t bme280_humidity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 100);
}

static DEVICE_ATTR(humidity,S_IRUSR,bme280_humidity_show,NULL);

static ssize_t bme280_pressure_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 101);
}

static DEVICE_ATTR(pressure,S_IRUSR,bme280_pressure_show,NULL);

static const struct of_device_id bme280_of_match[] = { 
        { .compatible = "bosch,bme280" },
        {} 
}; 
MODULE_DEVICE_TABLE(of, bme280_of_match); 

static struct i2c_device_id bme280_idtable[] = { 
   { "bme280", 0 },
   { }
}; 
 
MODULE_DEVICE_TABLE(i2c, bme280_idtable); 

static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    
    struct bme280 *bme280; 
    int err;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
        return -EIO; 
    bme280 = devm_kzalloc(&client->dev, sizeof(*bme280), GFP_KERNEL); 
    if (!bme280) 
        return -ENOMEM; 
 

    bme280->client = client; 
    i2c_set_clientdata(client, bme280); 
 
    err=device_create_file(&client->dev, &dev_attr_temperature);
    if (err){
        pr_info("Temp file creation failed\n"); 
        goto out_err;
    }
    
    err=device_create_file(&client->dev, &dev_attr_humidity);
    if (err){
        pr_info("Humidity file creation failed\n"); 
        goto out_remove_temp;
    }
    
    err=device_create_file(&client->dev, &dev_attr_pressure);
    if (err){
        pr_info("Pressure file creation failed\n"); 
        goto out_remove_temp_hum;
    }
 
    return 0; 
    
out_remove_temp:
    device_remove_file(&client->dev, &dev_attr_temperature);
out_remove_temp_hum:
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);
out_err:
    return err;
    
}

static int bme280_remove(struct i2c_client *client) {

    struct bme280 *bme280; 
 
    /* We retrieve our private data */ 
    bme280 = i2c_get_clientdata(client); 
    
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);
    device_remove_file(&client->dev, &dev_attr_pressure);
 
    /* Which hold gpiochip we want to work on */ 
   return 0; 
}

static struct i2c_driver bme280_driver = { 
   .driver = {
        .name = "bme280", 
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(bme280_of_match),
   }, 
 
   .id_table = bme280_idtable, 
   .probe    = bme280_probe, 
   .remove   = bme280_remove, 
};

// register the driver with the I2C core
module_i2c_driver(bme280_driver); 

MODULE_DESCRIPTION("A driver for BME280 Temperature, Humidity and Air Pressure Sensor");
MODULE_AUTHOR("Agnieszka Tuznik <tuznikagnieszka@gmail.com>"); 
MODULE_LICENSE("GPL");
