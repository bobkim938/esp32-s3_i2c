#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>

#define I2C_M_SDA GPIO_NUM_18
#define I2C_M_SCL GPIO_NUM_19
#define I2C_clk_speed 100000

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define WRITE_BIT I2C_MASTER_WRITE             
#define READ_BIT I2C_MASTER_READ

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define ESP_SLAVE_ADDR 0x04                     /*!< NEED TO BE CONFIGURED */

static const char *TAG = "i2c-master";

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_M_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_M_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_clk_speed
    };

    i2c_param_config(i2c_master_port, &config);

    return i2c_driver_install(i2c_master_port, config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t __attribute__((unused)) i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}



void app_main()
{
    
}