#include <linux/init.h> 
#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/gpio.h> 
#include <linux/i2c.h> 
#include <asm/div64.h>
#include <linux/delay.h>

#define BME280_CHIP_ID      	0xD0
#define BME280_CTRL_HUM     	0xF2
#define BME280_STATUS       	0xF3
#define BME280_CTRL_MEAS    	0xF4
#define BME280_CONFIG       	0xF5
#define BME280_PRESS_MSB    	0xF7
#define BME280_TEMP_MSB     	0xFA
#define BME280_HUM_MSB      	0xFD
#define BME280_DIG_T1_LSB   	0x88   
#define BME280_DIG_H1       	0xA1
#define BME280_DIG_H2_LSB   	0xE1    

#define BME280_CHIP_ID_VALUE    0x60
#define T_P_CALIB_REGS_COUNT	0x18
#define H_CALIB_REGS_COUNT	0x9

struct bme280_measurements {
    u32 press;
    u32 hum;
    s32 temp;
};

struct bme280_compensation_params {
    u16 dig_T1;
    s16 dig_T2;
    s16 dig_T3;
    u16 dig_P1;
    s16 dig_P2;
    s16 dig_P3;
    s16 dig_P4;
    s16 dig_P5;
    s16 dig_P6;
    s16 dig_P7;
    s16 dig_P8;
    s16 dig_P9;
    u8 dig_H1;
    s16 dig_H2;
    u8 dig_H3;
    s16 dig_H4;
    s16 dig_H5;
    s8 dig_H6;	
};

struct bme280 { 
   struct i2c_client  *client; 
   struct gpio_chip   chip; 
   struct mutex      lock; 
   struct bme280_measurements measurements;
   struct bme280_compensation_params params;
}; 

static s32 t_fine = 0;
// 0 - forced, 1 - normal
static bool sensor_mode = 0;


static void bme280_calibrate_temp(s32 adc_T, struct bme280 *bme280)
{
    s32 tmp1, tmp2;

    tmp1 = ((((adc_T >> 3) - ((s32)bme280->params.dig_T1 << 1))) * 
	    ((s32)bme280->params.dig_T2)) >> 11;
    tmp2 = (((((adc_T >> 4) - ((s32)bme280->params.dig_T1)) * 
	    ((adc_T >> 4) - ((s32)bme280->params.dig_T1))) >> 12) *
	    ((s32)bme280->params.dig_T3)) >> 14;
    t_fine = tmp1 + tmp2;
	
    bme280->measurements.temp = (t_fine * 5 + 128) >> 8; 
}


static void bme280_calibrate_hum(s32 adc_H, struct bme280 *bme280)
{    
    s32 tmp1;

    tmp1 = (t_fine - ((s32)76800));
    tmp1 = ((((adc_H << 14) - (((s32)bme280->params.dig_H4 << 20) -
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

static void bme280_calibrate_press(u32 adc_P, struct bme280 *bme280)
{       
    s64 tmp1, tmp2, tmp3;

    tmp1 = (s64)t_fine - 128000;
    tmp2 = tmp1 * tmp1 * (s64)bme280->params.dig_P6;
    tmp2 = tmp2 + ((tmp1 * (s64)bme280->params.dig_P5) << 17);
    tmp2 = tmp2 + (((s64)bme280->params.dig_P4) << 35);
    tmp1 = ((tmp1 * tmp1 * (s64)bme280->params.dig_P3) >> 8) + 
	    ((tmp1 * (s64)bme280->params.dig_P2) << 12);
    tmp1 = (((((s64)1) << 47) + tmp1)) * ((s64)bme280->params.dig_P1) >> 33;
    if(tmp1 == 0) {
	    bme280->measurements.press = 0;
	    return;	/* Avoid exception caused by division by zero */
    }
    tmp3 = 1048576 - (s32)adc_P;
    tmp3 = (((tmp3 << 31) - tmp2) * 3125);
    tmp3 = div_s64(tmp3, tmp1);
    tmp1 = (((s64)bme280->params.dig_P9) * (tmp3 >> 13) * (tmp3 >> 13)) >> 25;
    tmp2 = (((s64)bme280->params.dig_P8) * tmp3) >> 19;
    tmp3 = ((tmp3 + tmp1 + tmp2) >> 8) + (((s64)bme280->params.dig_P7) << 4);
    bme280->measurements.press = (u32)tmp3;
}

static int bme280_set_sensor_mode(struct bme280 *bme280)
{    
    int res;
    
    res = i2c_smbus_read_byte_data(bme280->client, BME280_CTRL_MEAS);
    if (res < 0) {
	    return res;
    }

    if (sensor_mode != 0 && sensor_mode != 1) {
	    pr_err("BME280 driver: Invalid mode set. Reading measurements stopped.\n");
	    return EINVAL;
    }

    if(sensor_mode == 0) {
        // Switch to force mode
        pr_info("BME280 driver: Reading measurements in forced mode.\n");
        res = i2c_smbus_write_byte_data(bme280->client, BME280_CTRL_MEAS, 0x26);
        if (res < 0) {
            return res;
        }
        res = i2c_smbus_write_byte_data(bme280->client, BME280_CTRL_HUM, 0x1);
        if (res < 0) {
            return res;
        }
    }
    else if (sensor_mode == 1 && (~res & 0x3)) {
        // If normal mode is not set, switch to normal mode
        pr_info("BME280 driver: Reading measurements in normal mode.\n");
        res = i2c_smbus_write_byte_data(bme280->client, BME280_CTRL_MEAS, 0x27);
        if (res < 0) {
            return res;
        }
        res = i2c_smbus_write_byte_data(bme280->client, BME280_CTRL_HUM, 0x1);
        if (res < 0) {
            return res;
        }
        // tstandby = 500 ms, filter off
        res = i2c_smbus_write_byte_data(bme280->client, BME280_CONFIG, 0x80);
        if (res < 0) {
            return res;
        }
    }

    return 0;
}

static int bme280_read(struct bme280 *bme280)
{
    int res;
    int i = 0;
    u32 pressure, temperature, humidity;
    u8 data[8];
    
    do {
        res = i2c_smbus_read_byte_data(bme280->client, BME280_STATUS);
        if (res < 0) {
            return res;
        }
    } while (res & 0x8); // waiting for measuring to be set to 0
    
    res = bme280_set_sensor_mode(bme280);
    if (res < 0) {
	pr_err("BME280 driver: Failed to set correct mode\n");
        return res;
    }

    while(i < 8 && !(res < 0)) {
        res = i2c_smbus_read_byte_data(bme280->client, BME280_PRESS_MSB + i);
        data[i] = (u8)(res & 0xFF);
        i ++;
    }
    
    if (res < 0) {
	pr_err("BME280 driver: Failed to read measurements\n");
        return res;
    }
    
    pressure = (data[0] << 12) | (data[0] << 4)  | (data[0] >> 4); 
    temperature = (data[3] << 12) | (data[4] << 4)  | (data[5] >> 4);
    humidity = (data[6] << 8) | data[7];

    bme280_calibrate_temp(temperature, bme280);
    bme280_calibrate_hum(humidity, bme280);
    bme280_calibrate_press(pressure, bme280);

    return 0;
}

static ssize_t bme280_temp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int temperature, res;    
    struct bme280 *bme280 = (struct bme280 *)dev->driver_data;
    
    mutex_lock(&bme280->lock);
    res = bme280_read(bme280);
    if (res < 0) {
	pr_err("BME280 driver: Failed to read temperature measurements\n");
        return res;
    }
    temperature = bme280->measurements.temp;
    mutex_unlock(&bme280->lock);
    
    return sprintf(buf, "%d\n", temperature);
}

static DEVICE_ATTR(temperature,S_IRUSR,bme280_temp_show,NULL);

static ssize_t bme280_humidity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int humidity, res;    
    struct bme280 *bme280 = (struct bme280 *)dev->driver_data;
    
    mutex_lock(&bme280->lock);
    res = bme280_read(bme280);
    if (res < 0) {
	pr_err("BME280 driver: Failed to read temperature measurements\n");
        return res;
    }
    humidity = bme280->measurements.hum;
    mutex_unlock(&bme280->lock);
    
    return sprintf(buf, "%d\n", humidity);
}

static DEVICE_ATTR(humidity,S_IRUSR,bme280_humidity_show,NULL);

static ssize_t bme280_pressure_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int pressure,  res;    
    struct bme280 *bme280 = (struct bme280 *)dev->driver_data;
    
    mutex_lock(&bme280->lock);
    res = bme280_read(bme280);
    if (res < 0) {
	pr_err("BME280 driver: Failed to read temperature measurements\n");
        return res;
    }
    pressure = bme280->measurements.press;
    mutex_unlock(&bme280->lock);
    
    return sprintf(buf, "%d\n", pressure);
}

static DEVICE_ATTR(pressure,S_IRUSR,bme280_pressure_show,NULL);

static ssize_t bme280_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t buflen)
{
    int res, mode;
    
    res = kstrtoint(buf, 10, &mode);
    
    if (res < 0) {
	pr_err("BME280_driver: Illegal value written for mode parameter.\n");
	return EINVAL;
    }
    
    if (mode == 0 || mode == 1 || mode == 2) {
	sensor_mode = mode;
    }
    else {
	pr_warn("BME280_driver: Mode not supported. Mode can be set to either 0 or 1.\n");
	return ENOTSUPP;
    }
	
    return buflen;
}

static ssize_t bme280_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    
    return sprintf(buf, "%d\n", sensor_mode);
}

static DEVICE_ATTR(mode, 0664, bme280_mode_show, bme280_mode_store);


static const struct of_device_id bme280_of_match[] = { 
        { .compatible = "bosch,bme280" },
        {} 
}; 
MODULE_DEVICE_TABLE(of, bme280_of_match); 

static int get_compensation_params(struct bme280 *bme280)
{    
    u8 buf[T_P_CALIB_REGS_COUNT];
    int i = 0;
    
    while (i < T_P_CALIB_REGS_COUNT) {
        buf[i] = (u8)(i2c_smbus_read_byte_data(bme280->client, BME280_DIG_T1_LSB + i) & 0xFF);
        i++;
    }

    bme280->params.dig_T1 = (u16)((buf[1] << 8) | buf[0]);
    bme280->params.dig_T2 = (s16)((buf[3] << 8) | buf[2]);
    bme280->params.dig_T3 = (s16)((buf[5] << 8) | buf[4]);
    bme280->params.dig_P1 = (u16)((buf[7] << 8) | buf[6]);
    bme280->params.dig_P2 = (s16)((buf[9] << 8) | buf[8]);
    bme280->params.dig_P3 = (s16)((buf[11] << 8) | buf[10]);
    bme280->params.dig_P4 = (s16)((buf[13] << 8) | buf[12]);
    bme280->params.dig_P5 = (s16)((buf[15] << 8) | buf[14]);
    bme280->params.dig_P6 = (s16)((buf[17] << 8) | buf[16]);
    bme280->params.dig_P7 = (s16)((buf[19] << 8) | buf[18]);
    bme280->params.dig_P8 = (s16)((buf[21] << 8) | buf[20]);
    bme280->params.dig_P9 = (s16)((buf[23] << 8) | buf[22]);
      
    i = 0;
    while (i < T_P_CALIB_REGS_COUNT-1) {
        buf[i] = (u8)(i2c_smbus_read_byte_data(bme280->client, BME280_DIG_H2_LSB + i) & 0xFF);
        i++;
    }
    
    bme280->params.dig_H1 = (u8)(i2c_smbus_read_byte_data(bme280->client, BME280_DIG_H1) & 0xFF);
    bme280->params.dig_H2 = (s16)((buf[1] << 8) | buf[0]);
    bme280->params.dig_H3 = (u8)(buf[2]);
    bme280->params.dig_H4 = (s16)((buf[4] & 0xF) | (buf[3] << 4));
    bme280->params.dig_H5 = (s16)((buf[6] << 4) | (buf[5] & 0xF));
    bme280->params.dig_H6 = (s8)(buf[7]);
    
    return 0;
}


static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{    
    struct bme280 *bme280; 
    int err, res;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) 
        return -EIO; 
    bme280 = devm_kzalloc(&client->dev, sizeof(*bme280), GFP_KERNEL); 
    if (!bme280) 
        return -ENOMEM; 

    bme280->client = client; 
    
    res = i2c_smbus_read_byte_data(bme280->client, BME280_CHIP_ID);
    if (res < 0) {
	goto out_remove_all;
    }
    
    if (res != BME280_CHIP_ID_VALUE) {
	pr_err("BME280 driver: Incorrect client chip ID: %x!\n", res);
	goto out_remove_all;
    }
    else{
	pr_info("BME280 driver: Client chip ID: %x\n", res);
    }
    
    i2c_set_clientdata(client, bme280);
    
    err=device_create_file(&client->dev, &dev_attr_temperature);
    if (err) {
        pr_err("BME280 driver: Temperature file creation failed\n"); 
        goto out_err;
    }
    
    err=device_create_file(&client->dev, &dev_attr_humidity);
    if (err) {
        pr_err("BME280 driver: Humidity file creation failed\n"); 
        goto out_remove_temp;
    }
    
    err=device_create_file(&client->dev, &dev_attr_pressure);
    if (err) {
        pr_err("BME280 driver: Pressure file creation failed\n"); 
        goto out_remove_temp_hum;
    }
    err=device_create_file(&client->dev, &dev_attr_mode);
    if (err) {
        pr_err("BME280 driver: Mode file creation failed\n"); 
        goto out_remove_temp_hum_press;
    }
    /*
    res = i2c_smbus_write_byte_data(bme280->client, 0xE0, 0xB6);
    if (res < 0){
	return res;
    }
    mdelay(2); 
    
    do{
	res = i2c_smbus_read_byte_data(bme280->client, 0xF3);
	if (res < 0){
	    return res;
	}
    } while (res & 0x1); // waiting for measuring to be set to 0 */

    mutex_init(&bme280->lock);

    res = get_compensation_params(bme280);
    if (res) {
        pr_err("BME280 driver: Reading compensation parameters failed!\n"); 
        goto out_remove_all;
    }

    pr_info("BME280 driver: Device probed and ready.");
    
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
out_remove_all:
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);
    device_remove_file(&client->dev, &dev_attr_pressure);
    device_remove_file(&client->dev, &dev_attr_mode);
out_err:
    return err;
    
}

static int bme280_remove(struct i2c_client *client)
{
    struct bme280 *bme280; 
 
    /* We retrieve our private data */ 
    bme280 = i2c_get_clientdata(client); 
    
    mutex_destroy(&bme280->lock);
    device_remove_file(&client->dev, &dev_attr_temperature);
    device_remove_file(&client->dev, &dev_attr_humidity);
    device_remove_file(&client->dev, &dev_attr_pressure);
    device_remove_file(&client->dev, &dev_attr_mode);

    pr_info("BME280 driver: Device removed.");

   return 0; 
}

static struct i2c_driver bme280_driver = { 
   .driver = {
        .name = "bme280", 
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(bme280_of_match),
   },
   .probe    = bme280_probe, 
   .remove   = bme280_remove, 
};

module_i2c_driver(bme280_driver); 

MODULE_DESCRIPTION("A driver for BME280 Temperature, Humidity and Air Pressure Sensor");
MODULE_AUTHOR("Agnieszka Tuznik <tuznikagnieszka@gmail.com>"); 
MODULE_LICENSE("GPL");
