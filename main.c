#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include <stdbool.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "sdkconfig.h"
#include "mqtt-client.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


/* External functions in lidar_app */
extern float distance();
extern float velocity();
extern esp_err_t lidar_init();

//extern _E_MQTT_ERRORS mqtt_client_publish(_S_MQTT_CLIENT_INFO * info, char * _topic, const void * payload, uint32_t payload_len, uint32_t retry);


#define ESP_WIFI_SSID	"wifi_name"
#define ESP_WIFI_PASS	"wifi_password"
#define ESP_MAX_RETRY	5

#define GPIO_INPUT_PIN_SEL	(1ULL << 4)

static EventGroupHandle_t g_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "wifi station";
static int g_retry_count = 0;
static SemaphoreHandle_t g_sem_handle = NULL;
static SemaphoreHandle_t g_reconnect_semphr = NULL;
static _S_MQTT_CLIENT_INFO * g_mqtt_handle = NULL;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void connection_failed(_S_MQTT_CLIENT_INFO * p_mqtt_client);

static uint32_t x_count = 1;

ip_addr_t mqtt_get_interface_ip(ip_addr_t * dest_ip)
{
	ip_addr_t ip_addr0;
	ip_addr0.type = IPADDR_TYPE_V4;
	ip_addr0.u_addr.ip4.addr = 0;

	return ip_addr0;
}

void wifi_init_sta( void )
{
	g_wifi_event_group = xEventGroupCreate();
	tcpip_adapter_init();
	ESP_ERROR_CHECK( esp_event_loop_init( event_handler, NULL ) );

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config =
	{
			.sta =
			{
					.ssid = ESP_WIFI_SSID,
					.password = ESP_WIFI_PASS
			}
	};

	ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
	ESP_ERROR_CHECK( esp_wifi_set_config( ESP_IF_WIFI_STA, &wifi_config ) );
	ESP_ERROR_CHECK( esp_wifi_start() );

	ESP_LOGI(TAG, "wifi_init_sta finished");
	ESP_LOGI(TAG, "connect to ap SSID: %s password: %s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

void vMessageSendTask( void * pvParameter )
{
	char * mqtt_topic = "esp32/component/odd_even";
	char * mqtt_data;
	char dist[20];
	char vel[20];
	float lidar_distance = 0;
	float lidar_velocity = 0;

	printf("2. vMessageSendTask xSemaphoreTake Ready\n");
	while (1)
	{
		xSemaphoreTake(g_sem_handle, portMAX_DELAY);
		printf("3. vMessageSendTask xSemaphoreTake Completed\n");

		// Get data from lidar
		lidar_distance = distance();
		lidar_velocity = velocity();
		x_count++;

		printf("String Conversion Error!\n");
		gcvt(lidar_distance, 6, dist);
		gcvt(lidar_velocity, 6, vel);

		printf("String Concatenation Error!\n");
		mqtt_data = dist;

		_E_MQTT_ERRORS e_err = mqtt_client_publish( g_mqtt_handle, mqtt_topic, mqtt_data, strlen( mqtt_data ), 5 );
		vTaskDelay( 1000 );
		if ( MQTT_ERROR_NONE == e_err )
		{
			printf( "Message Published\n" );
		}
		else
		{
			printf("Message Not Published\n\n");
			printf(mqtt_client_error_to_string(e_err));
			printf("\n\n");
		}
		xSemaphoreGive(g_sem_handle);
	}
}

void vReconnectTask(void * pvParameter )
{
	g_mqtt_handle = mqtt_client_init(512, 10000, 10000, &connection_failed);
	MQTTPacket_connectData * p_option = mqtt_client_connect_options("red", 60, 1, "blue", "white");
	mqtt_client_set_publish_info(g_mqtt_handle, 2, 0);
	_E_MQTT_ERRORS e_err;



	xEventGroupWaitBits(g_wifi_event_group, WIFI_CONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

	do
	{
		e_err = mqtt_client_connect(g_mqtt_handle, "localhost:8080", 8080, p_option, 0);
		if (e_err)
		{
			vTaskDelay(1000);
		}
	} while (e_err != MQTT_ERROR_NONE);

	xSemaphoreGive(g_sem_handle);

	do
	{
		xSemaphoreTake(g_reconnect_semphr, portMAX_DELAY);
		do
		{
			e_err = mqtt_client_connect( g_mqtt_handle, "localhost", 8080, p_option, NULL );
			if ( e_err )
			{
				vTaskDelay( 1000 );
			}
		} while ( e_err != MQTT_ERROR_NONE );

	}	while (1);
}

void app_main( void )
{
	printf("1. APP_MAIN STARTED\n");
	esp_err_t ret = nvs_flash_init();
	if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND )
	{
		ESP_ERROR_CHECK( nvs_flash_erase() );
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK( ret );

	g_reconnect_semphr = xSemaphoreCreateBinary();
	g_sem_handle = xSemaphoreCreateBinary();


	wifi_init_sta();
	ESP_ERROR_CHECK(lidar_init());
	xTaskCreate(&vMessageSendTask, "Message Send", 2048, NULL, 2, NULL);
	xTaskCreate(&vReconnectTask, "Reconnect Task", 4096, NULL, 3, NULL);
	printf("APP_MAIN ENDED\n");
}

static esp_err_t event_handler( void *ctx, system_event_t *event )
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG, "got ip: %s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		g_retry_count = 0;
		xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		if (g_retry_count < ESP_MAX_RETRY)
		{
			esp_wifi_connect();
			xEventGroupClearBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
			g_retry_count++;
		}
		break;
	default:
		break;
	}
	return ESP_OK;
}

static void connection_failed( _S_MQTT_CLIENT_INFO * p_mqtt_client )
{
	printf("Unexpected Connection Failure\n");
	xSemaphoreGive( g_reconnect_semphr );
}
