#include "driver/i2c.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int8.h>
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
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
static const char *TAG = "i2c-master";

rcl_publisher_t publisher;
rclc_executor_t executor;
std_msgs__msg__Int8 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
	}
}

void ROS_init(void* arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator(); // Micro_ROS initialization
    // support object initialization
    rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // node initialization
    rcl_node_t test_node;
    const char* node_name = "esp32_test";
    RCCHECK(rclc_node_init_default(&test_node, node_name, "", &support));

    const char* topic_name = "esp32_ping";
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8);
    // publisher init
    RCCHECK(rclc_publisher_init_default(&publisher, &test_node, type_support, topic_name));
    msg.data = 0;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

static esp_err_t i2c_master_init(void) {
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

static esp_err_t i2c_master_read_slave(uint8_t *data_rd, size_t size, i2c_port_t i2c_num) {
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

static esp_err_t i2c_master_send(uint8_t* msg, size_t size, i2c_port_t i2c_num) {
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

void app_main(void) {
    uint8_t ping = 0;
    uint8_t pong[255] = {0};
    ESP_ERROR_CHECK(i2c_master_init());

    static size_t uart_port = UART_NUM_0;
    #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
        rmw_uros_set_custom_transport(
            true,
            (void *) &uart_port,
            esp32_serial_open,
            esp32_serial_close,
            esp32_serial_write,
            esp32_serial_read
        );
    #else
    #error micro-ROS transports misconfigured
    #endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    ROS_init(NULL);  // Initialize micro-ROS

    while(1) {
        i2c_master_send(&ping, sizeof(ping), 0); // Sending data
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        esp_err_t read_result = i2c_master_read_slave(pong, sizeof(pong), 0); // Reading data
        if (read_result == ESP_OK) {
            ESP_LOGI(TAG, "Received Data: %d", *pong);
            ping = *pong + 1;
            msg.data = ping;
            RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        } else {
            ESP_LOGE(TAG, "Error reading from slave: %d", read_result);
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
    }
}
