#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c.h"

#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"


#define I2C_MASTER_SDA_IO 18 // SDA GPIO_18
#define I2C_MASTER_SCL_IO 19 // SCL GPIO_19
#define I2C_MASTER_FREQ_HZ 100000 // master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0      

#define WRITE_BIT I2C_MASTER_WRITE // master write
#define READ_BIT I2C_MASTER_READ // master read

#define slave_addr_esp 0x0A
#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                      
#define ACK_VAL 0x0                            
#define NACK_VAL 0x1        

static const char *TAG = "i2c-master";

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(0, &conf); // i2c parameters initialization
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "I2C initialized");
    return i2c_driver_install(0, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0); // i2c driver installation (last parameter: interrupt)
}


static esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size, i2c_port_t i2c_num)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr_esp << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t i2c_master_send(uint8_t* msg, size_t size, i2c_port_t i2c_num)
{
    ESP_LOGI(TAG, "Sending Data = %d", *msg);;   
    esp_err_t ret; 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create i2c command handle    
    i2c_master_start(cmd); // add start and stop conditions to the I2C command sequence
    i2c_master_write_byte(cmd, slave_addr_esp << 1 | WRITE_BIT, ACK_CHECK_EN); // add write commands to the I2C command sequence
    i2c_master_write(cmd, msg, size, ACK_CHECK_EN); // master write buffer to the i2c bus
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    uint8_t ping = 0;
    uint8_t pong[255] = {0};
    ESP_ERROR_CHECK(i2c_master_init());

  while(1)
    {
        i2c_master_send(&ping, sizeof(ping), 0); // Sending data
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        esp_err_t read_result = i2c_master_read_slave(pong, sizeof(pong), 0); // Reading data
        if (read_result == ESP_OK)
        {
            ESP_LOGI(TAG, "Received Data: %d", *pong);
            ping = *pong + 1;
        }
        else
        {
            ESP_LOGE(TAG, "Error reading from slave: %d", read_result);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}