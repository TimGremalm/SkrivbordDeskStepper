#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "driver/gpio.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include <vl53l0x_platform.h>
#include "vl53l0x_helper.h"
#include <driver/i2c.h>
#include <vl53l0x_def.h>

static const char *TAG = "SkrivbordDesk";

static EventGroupHandle_t connection_event_group;
const static int WIFI_CONNECTED_BIT = BIT0;
static const int MQTT_CONNECTED_BIT = BIT1;

static VL53L0X_Dev_t vl53l0x_dev;
static TimerHandle_t sensor_timer_h = NULL;

static void read_sensor(TimerHandle_t timer_handle);

uint32_t distance = 0;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	// your_context_t *context = event->context;
	switch (event->event_id) {
		case MQTT_EVENT_CONNECTED:
			xEventGroupSetBits(connection_event_group, MQTT_CONNECTED_BIT);

			ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
			msg_id = esp_mqtt_client_publish(client, "/Skrivbord/desk/hello", "Desk started", 0, 1, 0);
			ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_subscribe(client, "/Skrivbord/desk/setheight", 0);
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_subscribe(client, "/Skrivbord/desk/getheight", 0);
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			//msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
			//ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			//msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
			//ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
			break;
		case MQTT_EVENT_DISCONNECTED:
			xEventGroupClearBits(connection_event_group, MQTT_CONNECTED_BIT);
			ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
			break;

		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			//msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
			//ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(TAG, "MQTT_EVENT_DATA");
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);
			if (strncmp(event->topic, "/Skrivbord/desk/getheight", event->topic_len) == 0) {
				ESP_LOGI(TAG, "Get height request, publish height");
				char buf [100];
				snprintf(buf, sizeof buf, "%imm", distance);
				msg_id = esp_mqtt_client_publish(client, "/Skrivbord/desk/height", buf, 0, 1, 0);
			}
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
			break;
	}
	return ESP_OK;
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
	switch (event->event_id) {
		case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
			xEventGroupSetBits(connection_event_group, WIFI_CONNECTED_BIT);

			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			esp_wifi_connect();
			xEventGroupClearBits(connection_event_group, WIFI_CONNECTED_BIT);
			break;
		default:
			break;
	}
	return ESP_OK;
}

static void wifi_init(void) {
	tcpip_adapter_init();
	connection_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASSWORD,
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_LOGI(TAG, "Waiting for wifi");
	xEventGroupWaitBits(connection_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}

static void mqtt_app_start(void) {
	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
		.event_handle = mqtt_event_handler,
		// .user_context = (void *)your_context
	};
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_start(client);
}

static void i2c_master_init() {
	i2c_port_t i2c_master_port = I2C_NUM_1;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = GPIO_NUM_22;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = 400000;

	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
}

static void read_sensor(TimerHandle_t timer_handle) {
	VL53L0X_Error status;
	VL53L0X_RangingMeasurementData_t measurement_data;
	status = take_reading(&vl53l0x_dev, &measurement_data);
	if (status != VL53L0X_ERROR_NONE) {
		ESP_LOGI(TAG, "[APP] Couln't take reading");
	}
	distance = measurement_data.RangeMilliMeter;
	ESP_LOGI(TAG, "[APP] %imm", distance);
}

static void measure_timer_start(void) {
	sensor_timer_h = xTimerCreate(
		"reading_timer",
		pdMS_TO_TICKS(300),
		true,
		NULL,
		read_sensor
	);

	if (!sensor_timer_h || (xTimerStart(sensor_timer_h, pdMS_TO_TICKS(1000)) == pdFAIL)) {
		ESP_LOGE(TAG, "Failed to start timer");
		//esp_restart();
	}
}

void gpio_init_high(int pin, int highlow) {
	gpio_pad_select_gpio(pin);
	gpio_set_direction(pin, GPIO_MODE_OUTPUT);
	gpio_set_level(pin, highlow);
}

#define STEPPERS_ENABLE 27
#define STEPPERS_MS1 14
#define STEPPERS_MS2 26
#define STEPPERS_STEP 13
#define STEPPER_LEFT_DIR 12
#define STEPPER_RIGHT_DIR 25
static void motor_control(void *arg) {
	gpio_init_high(STEPPERS_MS1, 0);
	gpio_init_high(STEPPERS_MS2, 0);
	gpio_init_high(STEPPER_LEFT_DIR, 1);
	gpio_init_high(STEPPER_RIGHT_DIR, 1);
	gpio_init_high(STEPPERS_ENABLE, 0);
	gpio_init_high(STEPPERS_STEP, 0);
	while(1) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(STEPPERS_STEP, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		gpio_set_level(STEPPERS_STEP, 0);
	}
}

void app_main() {
	ESP_LOGI(TAG, "[APP] Startup Skrivbord Desk");
	ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
	ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

	esp_log_level_set("*", ESP_LOG_INFO);
	esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
	esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

	nvs_flash_init();
	wifi_init();
	mqtt_app_start();

	i2c_master_init();
	vl53l0x_dev.i2c_port_num = I2C_NUM_1;
	vl53l0x_dev.i2c_address = 0x29;

	VL53L0X_Error status = init_vl53l0x(&vl53l0x_dev);
	if (status != VL53L0X_ERROR_NONE) {
		ESP_LOGE(TAG, "[APP] Couldn't init VL53L0X Sensor");
		//esp_restart();
	}

	measure_timer_start();

	xTaskCreate(motor_control, "motor_control", 4096, NULL, 5, NULL);
}

