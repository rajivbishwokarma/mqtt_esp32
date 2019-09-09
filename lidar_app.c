#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "debug_rts.h"
#include "sdkconfig.h"

static const char *TAG = "lidar_lite_v3";
float g_distance = 0.00;
float g_velocity = 0.00;
float distance();
float velocity();

//-- all definations

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO GPIO_NUM_22                //--gpio number for I2C master clock (gpio 22 is set)
#define I2C_MASTER_SDA_IO GPIO_NUM_21                //--gpio number for I2C master data  (gpio 21 is set)
#define I2C_MASTER_NUM I2C_NUMBER(0)  //--I2C port number for master dev   (1)
#define I2C_MASTER_FREQ_HZ 100000         			//--I2C master clock frequency        (100000 is set)
#define I2C_MASTER_TX_BUF_DISABLE 0                            //--I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                            //--I2C master doesn't need buffer

#define LIDAR_ADDR   0x62                                      //-- Addess of lidar
#define CONTINUE_DISTANCE_RAED_CMD    0x04                     //-- Take distance measurement with receiver bias correction
#define DIST_L_CMD    0x10                                     //--Distance measurement result in centimeters, high byte
#define DIST_H_CMD    0x0f                                     //--Distance measurement result in centimeters, low byte.
#define DIST_READ_CMD  0x8f                                    //--Read two bytes to obtain the 16-bit distance in cm
#define VELOCITY_CMD    0x09                                    //--Read velocity in cm/s
#define WRITE_BIT I2C_MASTER_WRITE                             //--I2C master write
#define READ_BIT I2C_MASTER_READ                               //--I2C master read
#define ACK_CHECK_EN 0x1                                       //--I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0                                      //--I2C master will not check ack from slave
#define ACK_VAL 0x0                                            //--I2C ack value
#define NACK_VAL 0x1                                           //--I2C nack value


/**
 * @brief test code to operate on lidar sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t lidar_config(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l , uint8_t *vel)
{
    int error_val;
    i2c_cmd_handle_t  cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, LIDAR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CONTINUE_DISTANCE_RAED_CMD , ACK_CHECK_EN);
    i2c_master_stop(cmd);
    error_val = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if ( error_val != ESP_OK) {
    	printf("I2C Not Initilized \n");
       return  error_val;
    }

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LIDAR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd,DIST_READ_CMD, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	error_val = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LIDAR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, data_h, ACK_VAL);
	i2c_master_read_byte(cmd, data_l, NACK_VAL);
	vTaskDelay(30 / portTICK_RATE_MS);
	i2c_master_stop(cmd);
	error_val = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LIDAR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x04, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x80 , ACK_CHECK_EN);
	i2c_master_stop(cmd);
	error_val = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, LIDAR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, vel, NACK_VAL);
	vTaskDelay(30 / portTICK_RATE_MS);
	i2c_master_stop(cmd);
	error_val = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

   return error_val;
}

/**
 * @brief i2c master initialization
 */
esp_err_t lidar_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


 static void lidar_read()
{
    int error_val;
    uint8_t sensor_data_h, sensor_data_l, velocity;
    ESP_LOGI(TAG, "Reading Distance From Lidar in cm");
	error_val = lidar_config(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l, &velocity);
	if (error_val == ESP_ERR_TIMEOUT)
	{
	  ESP_LOGE(TAG, "I2C Timeout");
	}
	else if (error_val == ESP_OK)
	{
	  g_distance = (sensor_data_h << 8 | sensor_data_l)/1.00;
	  g_velocity = velocity;
	  TRACE("Distance: %.02f [cm]\n",g_distance);
	  TRACE("Velocity: %.02f [cm/s]\n", g_velocity);
	}
	else
	{
	  ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(error_val));
	}
}

float distance()
{
	lidar_read();
    return g_distance;
}

float velocity()
{
	lidar_read();
    return g_velocity;
}


