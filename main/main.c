#include <stdio.h>
#include "driver/i2c.h"

#define I2C_M_SDA GPIO_NUM_8
#define I2C_M_SCL GPIO_NUM_9
#define I2C_clk_speed 100000

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

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}



void app_main()
{
    
}