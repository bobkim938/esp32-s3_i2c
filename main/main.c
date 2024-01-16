#include <stdio.h>
#include "driver/i2c.h"

#define I2C_M_SDA GPIO_NUM_8
#define I2C_M_SCL GPIO_NUM_9
#define I2C_clk_speed 100000

i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_M_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_M_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_clk_speed
};




void app_main()
{
    
}