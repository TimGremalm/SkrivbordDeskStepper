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

//States for measured desk height
static int distance = 0;
static int deskCurrentHeight = 0;

//Desk height
#define DISTANCEOFFSET 550
#define DESKHEIGHTMIN 600
#define DESKHEIGHTMAX 1200

//Desk raise lowering ramping speed
#define RAMPSPEEDMIN 50
#define RAMPSPEEDMAX 4000

typedef enum {
	IDLE = 0,
	MOVING_UP,
	MOVING_DOWN,
	OVERRIDE_UP,
	OVERRIDE_DOWN,
	OVERRIDE_TILT_LEFT,
	OVERRIDE_TILT_RIGHT
} DESKSTATES;
static DESKSTATES deskState = IDLE;
static int deskGoalHeight = 0;
static int deskEndSwitchTop = 0;
static int deskEndSwitchBottom = 0;

typedef enum {
	BTN_NO = 0,
	BTN_UP,
	BTN_DOWN,
	BTN_TILT_LEFT,
	BTN_TILT_RIGHT
} BUTTONSTATES;
BUTTONSTATES button = BTN_NO;

void setDeskHeight(int height) {
	//Validate height input, if not valid discard
	if (height >= DESKHEIGHTMIN) {
		if (height <= DESKHEIGHTMAX) {
			//Only set goal height if state is idle or moving
			switch (deskState) {
				case IDLE:
				case MOVING_UP:
				case MOVING_DOWN:
					deskGoalHeight = height;
					break;
				default:
					break;
			}
		}
	}
}

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
			char buf[100] = {0};
			if (strncmp(event->topic, "/Skrivbord/desk/getheight", event->topic_len) == 0) {
				ESP_LOGI(TAG, "Get height request, publish height");
				snprintf(buf, sizeof buf, "%imm", deskCurrentHeight);
				msg_id = esp_mqtt_client_publish(client, "/Skrivbord/desk/height", buf, 0, 1, 0);
			}
			if (strncmp(event->topic, "/Skrivbord/desk/setheight", event->topic_len) == 0) {
				if (event->topic_len < sizeof buf) {
					memcpy(buf, event->data, event->data_len);
					int distanceToSet = atoi(buf);
					setDeskHeight(distanceToSet);
				}
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
	//Sensor can only read up to 1.2m, if it reporting more it's out of range
	if (distance < 1200) {
		deskCurrentHeight = distance + DISTANCEOFFSET;
	}
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

void gpio_init_state(int pin, int highlow) {
	gpio_pad_select_gpio(pin);
	gpio_set_direction(pin, GPIO_MODE_OUTPUT);
	gpio_set_level(pin, highlow);
}

#define UP_PIN 35
#define DOWN_PIN 34
#define LEFT_PIN 32
#define RIGHT_PIN 33
#define END_SWITCH_TOP_PIN 18
#define END_SWITCH_BOTTOM_PIN 19
void readInputs() {
	//Button inputs
	button = BTN_NO;
	if (!gpio_get_level(UP_PIN)) {
		button = BTN_UP;
	}
	if (!gpio_get_level(DOWN_PIN)) {
		button = BTN_DOWN;
	}
	if (!gpio_get_level(LEFT_PIN)) {
		button = BTN_TILT_LEFT;
	}
	if (!gpio_get_level(RIGHT_PIN)) {
		button = BTN_TILT_RIGHT;
	}
	//End switches
	if (!gpio_get_level(END_SWITCH_TOP_PIN)) {
		deskEndSwitchTop = 1;
	} else {
		deskEndSwitchTop = 0;
	}
	if (!gpio_get_level(END_SWITCH_BOTTOM_PIN)) {
		deskEndSwitchBottom = 1;
	} else {
		deskEndSwitchBottom = 0;
	}
}

//Stepper driver pins for 2xA3967 drivers
#define STEPPERS_ENABLE_PIN 27
#define STEPPERS_MS1_PIN 14
#define STEPPERS_MS2_PIN 26
#define STEPPERS_STEP_PIN 13
#define STEPPER_LEFT_DIR_PIN 12
#define STEPPER_RIGHT_DIR_PIN 25
static void motor_control(void *arg) {
	//Init pins
	gpio_init_state(STEPPERS_MS1_PIN, 1);
	gpio_init_state(STEPPERS_MS2_PIN, 1);
	gpio_init_state(STEPPER_LEFT_DIR_PIN, 1);
	gpio_init_state(STEPPER_RIGHT_DIR_PIN, 1);
	gpio_init_state(STEPPERS_ENABLE_PIN, 0);
	gpio_set_direction(UP_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(DOWN_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(LEFT_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(RIGHT_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(END_SWITCH_TOP_PIN, GPIO_MODE_INPUT);
	gpio_set_direction(END_SWITCH_BOTTOM_PIN, GPIO_MODE_INPUT);
	//Init pwm to steppers
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, STEPPERS_STEP_PIN);
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 20;
	pwm_config.cmpr_a = 0;
	pwm_config.cmpr_b = 0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
	//States
	int goalBefore = 0;
	DESKSTATES deskStateBefore = 0;
	BUTTONSTATES buttonBefore = 0;
	int speed = 0;
	int speedBefore = 0;
	//Regulate motors
	while(1) {
		//Have target height changed?
		if (deskGoalHeight != goalBefore) {
			int delta = deskGoalHeight - deskCurrentHeight;
			if (delta > 0) {
				//Go up
				ESP_LOGI(TAG, "[APP] Request to raise desk to %d", deskGoalHeight);
				deskState = MOVING_UP;
			} else {
				//Go down
				ESP_LOGI(TAG, "[APP] Request to lower desk to %d", deskGoalHeight);
				deskState = MOVING_DOWN;
			}
			goalBefore = deskGoalHeight;
		}
		//Check inputs for override
		readInputs();
		if (button != buttonBefore) {
			switch (button) {
				case BTN_UP:
					deskState = OVERRIDE_UP;
					break;
				case BTN_DOWN:
					deskState = OVERRIDE_DOWN;
					break;
				case BTN_TILT_LEFT:
					deskState = OVERRIDE_TILT_LEFT;
					break;
				case BTN_TILT_RIGHT:
					deskState = OVERRIDE_TILT_RIGHT;
					break;
				default:
					deskState = IDLE;
					break;
			}
		}
		//Have hit limit switch
		if (deskEndSwitchTop == 1) {
			if (deskState==MOVING_UP || deskState==OVERRIDE_UP || deskState== OVERRIDE_TILT_LEFT || deskState==OVERRIDE_TILT_RIGHT) {
				deskState = IDLE;
				ESP_LOGI(TAG, "[APP] Can't raise if top limit switch is hit");
			}
		}
		if (deskEndSwitchBottom == 1) {
			if (deskState==MOVING_DOWN || deskState==OVERRIDE_DOWN || deskState== OVERRIDE_TILT_LEFT || deskState==OVERRIDE_TILT_RIGHT) {
				deskState = IDLE;
				ESP_LOGI(TAG, "[APP] Can't lower if bottom limit switch is hit");
			}
		}
		//Have reached goal distance
		if (deskState == MOVING_UP && deskCurrentHeight >= deskGoalHeight) {
			deskState = IDLE;
			ESP_LOGI(TAG, "[APP] At goal, stop");
		}
		if (deskState == MOVING_DOWN && deskCurrentHeight <= deskGoalHeight) {
			deskState = IDLE;
			ESP_LOGI(TAG, "[APP] At goal, stop");
		}
		//Change state of stepper driver
		if (deskState != deskStateBefore) {
			switch (deskState) {
				case IDLE:
					gpio_set_level(STEPPERS_ENABLE_PIN, 1);
					gpio_set_level(STEPPER_LEFT_DIR_PIN, 0);
					gpio_set_level(STEPPER_RIGHT_DIR_PIN, 0);
					speed = 0;
					ESP_LOGI(TAG, "[APP] Idle");
					break;
				case MOVING_UP:
				case OVERRIDE_UP:
					gpio_set_level(STEPPERS_ENABLE_PIN, 0);
					gpio_set_level(STEPPER_LEFT_DIR_PIN, 1);
					gpio_set_level(STEPPER_RIGHT_DIR_PIN, 1);
					speed = RAMPSPEEDMIN;
					ESP_LOGI(TAG, "[APP] Raise");
					break;
				case MOVING_DOWN:
				case OVERRIDE_DOWN:
					gpio_set_level(STEPPERS_ENABLE_PIN, 0);
					gpio_set_level(STEPPER_LEFT_DIR_PIN, 0);
					gpio_set_level(STEPPER_RIGHT_DIR_PIN, 0);
					speed = RAMPSPEEDMIN;
					ESP_LOGI(TAG, "[APP] Lower");
					break;
				case OVERRIDE_TILT_LEFT:
					gpio_set_level(STEPPERS_ENABLE_PIN, 0);
					gpio_set_level(STEPPER_LEFT_DIR_PIN, 1);
					gpio_set_level(STEPPER_RIGHT_DIR_PIN, 0);
					speed = RAMPSPEEDMIN;
					ESP_LOGI(TAG, "[APP] Tilt left");
					break;
				case OVERRIDE_TILT_RIGHT:
					gpio_set_level(STEPPERS_ENABLE_PIN, 0);
					gpio_set_level(STEPPER_LEFT_DIR_PIN, 0);
					gpio_set_level(STEPPER_RIGHT_DIR_PIN, 1);
					speed = RAMPSPEEDMIN;
					ESP_LOGI(TAG, "[APP] Tilt right");
					break;
			}
		}
		//Ramp motor
		if (deskState != IDLE ) {
			if (speed < RAMPSPEEDMAX) {
				speed += 50;
			}
			if (speed != speedBefore) {
				mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
				ESP_LOGI(TAG, "[APP] Speed: %d", speed);
			}
		}
		if (deskState != deskStateBefore) {
			if (deskState == IDLE) {
				mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
				ESP_LOGI(TAG, "[APP] Stop PWM");
			} else {
				mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
				ESP_LOGI(TAG, "[APP] Start PWM");
			}
		}
		deskStateBefore = deskState;
		buttonBefore = button;
		speedBefore = speed;
		vTaskDelay(100 / portTICK_PERIOD_MS);
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

	wifi_init();
	mqtt_app_start();
}

