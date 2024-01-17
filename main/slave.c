#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

static const char *TAG = "i2c-slave";

#define I2C_SLAVE_SDA_IO 18             
#define I2C_SLAVE_SCL_IO 19            
#define I2C_SLAVE_FREQ_HZ 100000       
#define I2C_SLAVE_TX_BUF_LEN 255                     
#define I2C_SLAVE_RX_BUF_LEN 255                         
#define ESP_SLAVE_ADDR 0x0A

#define WRITE_BIT I2C_MASTER_WRITE          
#define READ_BIT I2C_MASTER_READ              
#define ACK_CHECK_EN 0x1                     
#define ACK_CHECK_DIS 0x0                 
#define ACK_VAL 0x0                         
#define NACK_VAL 0x1                     

int i2c_slave_port = 0;

static esp_err_t i2c_slave_init(void)
{
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,          // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,          // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,      // address of your project
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "I2C Slave initialized");
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void app_main(void)
{
    uint8_t pong[I2C_SLAVE_RX_BUF_LEN] = {0};

    ESP_ERROR_CHECK(i2c_slave_init());

    while (1)
    {
        // Wait for the master to write data
        int bytes_received = i2c_slave_read_buffer(i2c_slave_port, pong, I2C_SLAVE_RX_BUF_LEN, 100 / portTICK_PERIOD_MS);
        if (bytes_received > 0)
        {
            for (int i = 0; i < bytes_received; i++)
            {
                pong[i] += 1; // Process the received data (incrementing each byte by 1 in this case)
            }
            i2c_slave_write_buffer(i2c_slave_port, pong, bytes_received, 100 / portTICK_PERIOD_MS); // Send the modified data back to the master
            ESP_LOGI(TAG, "Received Data: %d", *pong);
        }
        memset(pong, 0, I2C_SLAVE_RX_BUF_LEN); // Clear the received data buffer
    }
}
