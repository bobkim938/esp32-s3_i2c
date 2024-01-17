#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SDA_IO 18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define SLAVE_ADDRESS 0x0A

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static const char *TAG = "i2c-master";

int i2c_master_port = 0;
static esp_err_t i2c_master_init(void)
{
  
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf); // i2c parameters initialization
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0); // i2c driver installation (last parameter: interrupt)
}

static esp_err_t i2c_master_send(uint8_t message[], int len)
{
    ESP_LOGI(TAG, "Sending Message = %s", message);   
    
    esp_err_t ret; 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // create i2c command handle    
    i2c_master_start(cmd); // Add start and stop conditions to the I2C command sequence
    i2c_master_write_byte(cmd, SLAVE_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN); // Add write commands to the I2C command sequence
    i2c_master_write(cmd, message, len, ACK_CHECK_EN); // master write buffer to the i2c bus
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    printf("Initiating\n");
    const uint8_t  on_command[] = "LED_ON";
    const uint8_t  off_command[] = "LED_OFF";
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    while(1)
    {
        i2c_master_send(on_command, sizeof(on_command));
        vTaskDelay(1000/ portTICK_PERIOD_MS);
        i2c_master_send(off_command, sizeof(off_command));
        vTaskDelay(1000/ portTICK_PERIOD_MS);  
    }
}