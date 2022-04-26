#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/gpio.h> 
#include <linux/i2c.h> 
#include <asm/div64.h>

// static int __init bme280_init(void) { 
//     pr_info("Hello world!\n"); 
//     return 0; 
// } 
 
// static void __exit bme280_exit(void) { 
//     pr_info("Bye world\n"); 
// } 
 
// module_init(bme280_init); 
// module_exit(bme280_exit); 

struct bme280_measurements {
    int press;
    int hum;
    int temp;
};

struct bme280_compensation_params{
    unsigned short dig_T1;
    signed short dig_T2;
    signed short dig_T3;
    unsigned short dig_P1;
    signed short dig_P2;
    signed short dig_P3;
    signed short dig_P4;
    signed short dig_P5;
    signed short dig_P6;
    signed short dig_P7;
    signed short dig_P8;
    signed short dig_P9;
    unsigned char dig_H1;
    signed short dig_H2;
    unsigned char dig_H3;
    signed short dig_H4;
    signed short dig_H5;
    signed char dig_H6;	
};

struct bme280 { 
   struct i2c_client  *client; 
   struct gpio_chip   chip; 
   struct mutex      lock; 
   struct bme280_measurements measurements;
   struct bme280_compensation_params params;
}; 

static s32 fine_t = 0;
// 0 - forced, 1 - normal
static bool normal_mode = 0;
//static char mybuf[100] = "mydevice";


static void bme280_calibrate_temp(u32 temp, struct bme280 *bme280){

    s32 tmp1, tmp2;

    tmp1 = ((((temp >> 3) - ((s32)bme280->params.dig_T1 << 1))) * 
	    ((s32)bme280->params.dig_T2)) >> 11;
    tmp2 = (((((temp >> 4) - ((s32)bme280->params.dig_T1)) * 
	    ((temp >> 4) - ((s32)bme280->params.dig_T1))) >> 12) *
	    ((s32)bme280->params.dig_T3)) >> 14;
    fine_t = tmp1 + tmp2;
	
    bme280->measurements.temp = (fine_t * 5 + 128) >> 8; 
}


static void bme280_calibrate_hum(u32 hum, struct bme280 *bme280){
    
    s32 tmp1;

    tmp1 = (fine_t - ((s32)76800));
    tmp1 = ((((hum << 14) - (((s32)bme280->params.dig_H4 << 20) -
	    (((s32)bme280->params.dig_H5) * tmp1)) +
	    ((s32)16384)) >> 15) * (((((((tmp1 * ((s32)bme280->params.dig_H6)) >> 10) *
	    (((tmp1 * ((s32)bme280->params.dig_H3)) >> 11) + ((s32)32768))) >> 10) + 
	    ((s32)2097152)) * ((s32)bme280->params.dig_H2) + 8192) >> 14));
    tmp1 = (tmp1 - (((((tmp1 >> 15) * (tmp1 >> 15)) >> 7) *
	    (( s32)bme280->params.dig_H1)) >> 4));
    tmp1 = (tmp1 < 0 ? 0 : tmp1);
    tmp1 = (tmp1 > 419430400 ? 419430400 : tmp1);

    bme280->measurements.hum = (u32)(tmp1 >> 12);
}

static void bme280_calibrate_press(u32 press, struct bme280 *bme280){
       
    s64 tmp1, tmp2, tmp3; //, tmp4;

    tmp1 = (s64)fine_t - 128000;
    tmp2 = tmp1 * tmp1 * (s64)bme280->params.dig_P6;
    tmp2 = tmp2 + ((tmp1 * (s64)bme280->params.dig_P5) << 17);
    tmp2 = tmp2 + (((s64)bme280->params.dig_P4) << 35);
    tmp1 = ((tmp1 * tmp1 * (s64)bme280->params.dig_P3) >> 8) + 
	    ((tmp1 * (s64)bme280->params.dig_P2) << 12);
    tmp1 = (((((s64)1) << 47) + tmp1)) * ((s64)bme280->params.dig_P1) >> 33;
    if(tmp1 == 0){
	    bme280->measurements.press = 0;
	    return;	/* Avoid exception caused by division by zero */
    }
    tmp3 = 1048576 - (s32)press;
    tmp3 = (((tmp3 << 31) - tmp2) * 3125);
    //tmp4 = do_div(tmp3, tmp1);
    tmp3 = div_s64(tmp3, tmp1);
    tmp1 = (((s64)bme280->params.dig_P9) * (tmp3 >> 13) * (tmp3 >> 13)) >> 25;
    tmp2 = (((s64)bme280->params.dig_P8) * tmp3) >> 19;
    tmp3 = ((tmp3 + tmp1 + tmp2) >> 8) + (((s64)bme280->params.dig_P7) << 4);
    bme280->measurements.press = (u32)tmp3;
}

static int bme280_read(struct bme280 *bme280){

    int tmp;
    int i = 0;
    u32 pressure, temperature, humidity;
    u8 data_readout[8];
    
    do{
	tmp = i2c_smbus_read_byte_data(bme280->client, 0xF3);
	if (tmp < 0){
	    return tmp;
	}
    } while (tmp & 0x8); // waiting for measuring to be set to 0
    
    // TODO check current mode from registers and determine whether change is needed

    if(normal_mode == 0){
	// Switch to force mode
	printk("BME280_driver: Reading measurements in forced mode.\n");
	tmp = i2c_smbus_write_byte_data(bme280->client, 0xF4, 0x26);
	if (tmp < 0){
	    return tmp;
	}
	tmp = i2c_smbus_write_byte_data(bme280->client, 0xF2, 0x1);
	if (tmp < 0){
	    return tmp;
	}
    }
    else if (normal_mode == 1){
	// Switch to normal mode
	printk("BME280_driver: Reading measurements in normal mode.\n");
	tmp = i2c_smbus_write_byte_data(bme280->client, 0xF4, 0x27);
	if (tmp < 0){
	    return tmp;
	}
	tmp = i2c_smbus_write_byte_data(bme280->client, 0xF2, 0x1);
	if (tmp < 0){
	    return tmp;
	}
	// tstandby = 500 ms, filter off
	tmp = i2c_smbus_write_byte_data(bme280->client, 0xF5, 0x80);
	if (tmp < 0){
	    return tmp;
	}
    }
    else{
	printk("BME280_driver: Invalid mode set. Reading measurements stopped.\n");
	return EINVAL;
    }

    while(i < 8){
        tmp = i2c_smbus_read_byte_data(bme280->client, 0xF7 + i);
        data_readout[i] = (u8)(tmp & 0xFF);
        i ++;
    }
    
    pressure = (data_readout[0] << 12) | (data_readout[0] << 4)  | (data_readout[0] >> 4); 
    temperature = (data_readout[3] << 12) | (data_readout[4] << 4)  | (data_readout[5] >> 4);
    humidity = (data_readout[6] << 8) | data_readout[7];
    bme280_calibrate_temp(temperature, bme280);
    bme280_calibrate_hum(humidity, bme280);
    bme280_calibrate_press(pressure, bme280);

    return 0;
}

static ssize_t bme280_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int temperature;
    
    struct bme280 *bme280 = (struct bme280 *)dev->driver_data;
    bme280_read(bme280);
    temperature = bme280->measurements.temp;
    return sprintf(buf, "%d\n", temperature);
}

static DEVICE_ATTR(temperature,S_IRUSR,bme280_temp_show,NULL);

static ssize_t bme280_humidity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int humidity;
    
    struct bme280 *bme280 = (struct bme280 *)dev->driver_data;
    bme280_read(bme280);
    humidity = bme280->measurements.hum;
    return sprintf(buf, "%d\n", humidity);
}

static DEVICE_ATTR(humidity,S_IRUSR,bme280_humidity_show,NULL);

static ssize_t bme280_pressure_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int pressure;
    
    struct bme280 *bme280 = (struct bme280 *)dev->driver_data;
    bme280_read(bme280);
    pressure = bme280->measurements.press;
    return sprintf(buf, "%d\n", pressure);
}

static DEVICE_ATTR(pressure,S_IRUSR,bme280_pressure_show,NULL);

static ssize_t bme280_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t buflen)
{
    int res, mode;
    
    res = kstrtoint(buf, 10, &mode);
    
    if (res < 0){
	printk("BME280_driver: Illegal value written for mode parameter.\n");
	return EINVAL;
    }
    
    if (mode == 0 || mode == 1){
	normal_mode = mode;
    }
    else{
	printk("BME280_driver: Mode not supported. Mode can be set to either 0 or 1.\n");
	return ENOTSUPP;
    }
	
    return buflen;
}

static ssize_t bme280_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    
    return sprintf(buf, "%d\n", normal_mode);
}

static DEVICE_ATTR(mode, 0664, bme280_mode_show, bme280_mode_store);


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


static int get_compensation_params(struct bme280 *bme280){
    
    u8 buf[24];
    int i;
    
    while (i < 24){
        buf[i] = (u8)(i2c_smbus_read_byte_data(bme280->client, 0x88 + i) & 0xFF);
        i++;
    }

    bme280->params.dig_T1 = (unsigned short)((buf[1] << 8) | buf[0]);
    bme280->params.dig_T2 = (signed short)((buf[3] << 8) | buf[2]);
    bme280->params.dig_T3 = (signed short)((buf[5] << 8) | buf[4]);
    bme280->params.dig_P1 = (unsigned short)((buf[7] << 8) | buf[6]);
    bme280->params.dig_P2 = (signed short)((buf[9] << 8) | buf[8]);
    bme280->params.dig_P3 = (signed short)((buf[11] << 8) | buf[10]);
    bme280->params.dig_P4 = (signed short)((buf[13] << 8) | buf[12]);
    bme280->params.dig_P5 = (signed short)((buf[15] << 8) | buf[14]);
    bme280->params.dig_P6 = (signed short)((buf[17] << 8) | buf[16]);
    bme280->params.dig_P7 = (signed short)((buf[19] << 8) | buf[18]);
    bme280->params.dig_P8 = (signed short)((buf[21] << 8) | buf[20]);
    bme280->params.dig_P9 = (signed short)((buf[23] << 8) | buf[22]);
      
    i = 0;
    while (i < 8){
        buf[i] = (u8)(i2c_smbus_read_byte_data(bme280->client, 0xE1 + i) & 0xFF);
        i++;
    }
    
    bme280->params.dig_H1 = (unsigned char)(i2c_smbus_read_byte_data(bme280->client, 0xA1) & 0xFF);
    bme280->params.dig_H2 = (signed short)((buf[1] << 8) | buf[0]);
    bme280->params.dig_H3 = (unsigned char)(buf[2]);
    bme280->params.dig_H4 = (signed short)((buf[4] & 0xF) | (buf[3] << 4));
    bme280->params.dig_H5 = (signed short)((buf[6] << 4) | (buf[5] & 0xF));
    bme280->params.dig_H6 = (signed char)(buf[7]);
    
    return 0;
}


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
    
    //TODO" read chip id, check chip id, reset the sensor, wait to 2ms according to table 1 and check status (im update)
    
    get_compensation_params(bme280);
 
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
    err=device_create_file(&client->dev, &dev_attr_mode);
    if (err){
        pr_info("Mode file creation failed\n"); 
        goto out_remove_temp_hum_press;
    }
    return 0; 
    
out_remove_temp:
    device_remove_file(&client->dev, &dev_attr_temperature);
out_remove_temp_hum:
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);
out_remove_temp_hum_press:
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);
    device_remove_file(&client->dev, &dev_attr_pressure);
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
    device_remove_file(&client->dev, &dev_attr_mode);
 
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
