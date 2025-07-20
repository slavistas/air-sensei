/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp32s2/rom/ets_sys.h"
#include <sys/time.h>
#include <esp_netif_sntp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/******************************************************************************/
/*!                         MQTT Driver Includes                              */

#include "mqtt_client.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

/******************************************************************************/
/*!                       BME280 Driver Includes                              */
#include "bme280.h"
#define SAMPLE_COUNT  UINT8_C(25)

/******************************************************************************/
/*!                        WI-FI Driver Includes                              */
#include "wifi_station.h"
#include "esp_mac.h"

static const char *TAG = "MQTT_BME280";

struct tph_data
{
    double temperature;
    double pressure;
    double humidity;
};


/*!
 *  @brief This internal API is used to get compensated humidity data.
 */
static int8_t get_humidity(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    double cumulative_humidity = 0;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.humidity = comp_data.humidity / 1000;
#endif

            cumulative_humidity = cumulative_humidity + comp_data.humidity;


            idx++;
        }
    }

#ifdef BME280_DOUBLE_ENABLE
    printf("Averaged Double Precision Humidity[%d]:   %lf %%RH\n", idx, cumulative_humidity / SAMPLE_COUNT);
#else
    printf("Averaged Humidity[%d]:   %lu %%RH\n", idx, (long unsigned int)cumulative_humidity / SAMPLE_COUNT);
#endif

    return rslt;
}

/*!
 *  @brief This internal API is used to get compensated pressure data.
 */
static int8_t get_pressure(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    double cumulative_pressure = 0;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifdef BME280_64BIT_ENABLE
            comp_data.pressure = comp_data.pressure / 100;
#endif

            cumulative_pressure = cumulative_pressure + comp_data.pressure;

            idx++;
        }
    }

#ifdef BME280_DOUBLE_ENABLE
    printf("Double Precision Pressure[%d]:  %lf Pa\n", idx, cumulative_pressure / SAMPLE_COUNT);
#else
    printf("Pressure[%d]:   %lu Pa\n", idx, (long unsigned int)cumulative_pressure / SAMPLE_COUNT);
#endif

    return rslt;
}


/*!
 *  @brief This internal API is used to get compensated temperature data.
 */
static int8_t get_temperature(uint32_t period, struct bme280_dev *dev)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    double cumulative_temperature = 0;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_TEMP, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
#endif

            cumulative_temperature = cumulative_temperature + comp_data.temperature;

            idx++;
        }
    }

#ifdef BME280_DOUBLE_ENABLE
    printf("Averaged Double Precision Temperature[%d]:   %lf deg C\n", idx, cumulative_temperature / SAMPLE_COUNT);
#else
    printf("Averaged Temperature[%d]:   %ld deg C\n", idx, (long int)cumulative_temperature / SAMPLE_COUNT);
#endif

    return rslt;
}


/*!
 *  @brief This internal API is used to get compensated temperature, pressure, and humidity data.
 */
static int8_t get_tph(uint32_t period, struct bme280_dev *dev, struct tph_data *data)
{
    int8_t rslt = BME280_E_NULL_PTR;
    int8_t idx = 0;
    uint8_t status_reg;
    struct bme280_data comp_data;

    data -> temperature = 0;
    data -> pressure = 0;
    data -> humidity = 0;

    while (idx < SAMPLE_COUNT)
    {
        rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        bme280_error_codes_print_result("bme280_get_regs", rslt);

        if (status_reg & BME280_STATUS_MEAS_DONE)
        {
            /* Measurement time delay given to read sample */
            dev->delay_us(period, dev->intf_ptr);

            /* Read compensated data */
            rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
            bme280_error_codes_print_result("bme280_get_sensor_data", rslt);

#ifndef BME280_DOUBLE_ENABLE
            comp_data.temperature = comp_data.temperature / 100;
            comp_data.pressure = comp_data.pressure / 100;
            comp_data.humidity = comp_data.humidity / 1000;
#endif

            data -> temperature = data -> temperature + comp_data.temperature;
            data -> pressure = data -> pressure + comp_data.pressure;
            data -> humidity = data -> humidity + comp_data.humidity;

            idx++;
        }
    }

    return rslt;
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
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
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static esp_mqtt_client_handle_t mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    return client;
}

/*!
 *  @brief This function configures the BME280 with desired configuration for this application
 */
int8_t configureBME280(uint32_t *period, struct bme280_dev *dev){
    int8_t rslt;
    struct bme280_settings settings;


    /* Interface selection is to be updated as parameter
     * For I2C :  BME280_I2C_INTF
     * For SPI :  BME280_SPI_INTF
     */
    rslt = bme280_interface_selection(dev, BME280_I2C_INTF);
    bme280_error_codes_print_result("bme280_interface_selection", rslt);

    rslt = bme280_init(dev);
    bme280_error_codes_print_result("bme280_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bme280_get_sensor_settings(&settings, dev);
    bme280_error_codes_print_result("bme280_get_sensor_settings", rslt);

    /* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, dev);
    bme280_error_codes_print_result("bme280_set_sensor_settings", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, dev);
    bme280_error_codes_print_result("bme280_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bme280_cal_meas_delay(period, &settings);
    bme280_error_codes_print_result("bme280_cal_meas_delay", rslt);

    printf("\nTemperature calculation (Data displayed are compensated values)\n");
    printf("Measurement time : %lu us\n\n", (long unsigned int)*period);

    return rslt;
}

/*!
 *  @brief This function configures SNTP client to talk to an NTP server and get current date/time
 */
int8_t configureSNTP(){
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_NTP_URI);
    esp_netif_sntp_init(&config);

    // See this for valid timezone codes https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
    // setenv("TZ", CONFIG_TIMEZONE_LOCAL_STRING, 1);
    // tzset();

    ESP_LOGI(TAG, "Trying to configure SNTP and talk to server at '%s'", CONFIG_NTP_URI);
    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to sync with '%s' and update system time within 10s timeout", CONFIG_NTP_URI);
        return 1;
    } else {
        // Get Current Time
        time_t now;
        char strftime_buf[64];
        struct tm timeinfo;
    
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c %Z", &timeinfo);
        ESP_LOGI(TAG, "SNTP Synced to '%s'\nThe current date/time in %s is: %s", CONFIG_NTP_URI, CONFIG_TIMEZONE_LOCATION, strftime_buf);
    }

    return 0;
}

void taskPublishEnvironmentalData(void *arg) {
    int8_t rslt;
    uint32_t period;
    struct bme280_dev dev;

    struct tph_data data;

    esp_mqtt_client_handle_t client = mqtt_app_start();

    configureBME280(&period, &dev);

    esp_err_t ret = ESP_OK;
    uint8_t base_mac_addr[6];
    ret = esp_efuse_mac_get_default(base_mac_addr);
    if(ret != ESP_OK){
            ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK0. (%s)", esp_err_to_name(ret));
            ESP_LOGE(TAG, "Aborting");
            abort();
        } 

    while (true)
    {
        rslt = get_tph(period, &dev, &data);
        bme280_error_codes_print_result("get_tph", rslt);

        char json_string[256];
        // char pressure_string[128];
        // char humidity_string[128];
        
        // Get Current Time
        time_t now;
        char strftime_buf[64];
        struct tm timeinfo;
    
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c %Z", &timeinfo);
        ESP_LOGI(TAG, "The current date/time in %s is: %s", CONFIG_TIMEZONE_LOCATION, strftime_buf);

        #ifdef BME280_DOUBLE_ENABLE
            sprintf(json_string, "{\"time\": \"%s\", \"MAC\": \"%02x:%02x:%02x:%02x:%02x:%02x\", \"temperature\": %lf, \"humidity\": %lf, \"pressure\": %lf}", \
                strftime_buf, \
                base_mac_addr[0], \
                base_mac_addr[1], \
                base_mac_addr[2], \
                base_mac_addr[3], \
                base_mac_addr[4], \
                base_mac_addr[5], \
                data.temperature / SAMPLE_COUNT, \
                data.humidity / SAMPLE_COUNT, \
                data.pressure / SAMPLE_COUNT);
        #else
            sprintf(json_string, "{\"time\": \"%s\", \"MAC\": \"%02x:%02x:%02x:%02x:%02x:%02x\",  \"temperature\": %ld, \"humidity\": %ld, \"pressure\": %ld}", \
                strftime_buf, \
                base_mac_addr[0], \
                base_mac_addr[1], \
                base_mac_addr[2], \
                base_mac_addr[3], \
                base_mac_addr[4], \
                base_mac_addr[5], \
                (long int) data.temperature / SAMPLE_COUNT, \
                (long unsigned int) data.humidity / SAMPLE_COUNT, \
                (long unsigned int) data.pressure / SAMPLE_COUNT);
        #endif

        #ifdef BME280_DOUBLE_ENABLE
            ESP_LOGI(TAG, "Double Precision: %s", json_string);
        #else
            ESP_LOGI(TAG, "%s", json_string);
        #endif

        int msg_id = esp_mqtt_client_publish(client, "env_sensor_data", json_string, 0, 1, 0);

        vTaskDelay(pdMS_TO_TICKS(CONFIG_MEASUREMENT_PERIOD));
    }

    bme280_esp32_deinit();
}

void app_main(void)
{
    wifi_init();

    while (configureSNTP() != 0) {
        ESP_LOGI(TAG, "Re-trying to configure SNTP");
    }

    xTaskCreate(taskPublishEnvironmentalData, "taskPublishEnvironmentalData", 4096, NULL, 10, NULL);
}

