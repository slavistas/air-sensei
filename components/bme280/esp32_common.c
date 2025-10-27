/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bme280.h"

/*
    ESP Header Includes
*/
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

/******************************************************************************/
/*!                               Macros                                      */

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS       1000

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to ESP32 platform
 */
BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    // return coines_read_i2c(COINES_I2C_BUS_0, dev_addr, reg_addr, reg_data, (uint16_t)length);
    return i2c_master_write_read_device(I2C_MASTER_NUM, BME280_I2C_ADDR_PRIM, &reg_addr, 1, reg_data, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*!
 * I2C write function map to ESP32 platform
 */
BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    uint8_t *write_buffer = (uint8_t*) malloc((length + 1) * sizeof(uint8_t));
    
    // First Value is register address
    write_buffer[0] = reg_addr;
    // Remaining values are write values
    memcpy(write_buffer + 1,  reg_data, length);


    int res = i2c_master_write_to_device(I2C_MASTER_NUM, BME280_I2C_ADDR_PRIM, write_buffer, length + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    free(write_buffer);

    return res;
}

/*!
 * Delay function map to ESP32 platform
 */
void bme280_delay_us(uint32_t period, void *intf_ptr)
{
    int64_t start_time = esp_timer_get_time();
    int64_t current_time;

    do
    {
        current_time = esp_timer_get_time();
    } while ((current_time - start_time) < (uint64_t) period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme280_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        printf("%s\t", api_name);

        switch (rslt)
        {
            case BME280_E_NULL_PTR:
                printf("Error [%d] : Null pointer error.", rslt);
                printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BME280_E_COMM_FAIL:
                printf("Error [%d] : Communication failure error.", rslt);
                printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BME280_E_DEV_NOT_FOUND:
                printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BME280_E_INVALID_LEN:
                printf("Error [%d] : Invalid length error. It occurs when write is done with invalid length\r\n", rslt);
                break;

            default:
                printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme280_interface_selection(struct bme280_dev *dev, uint8_t intf)
{
    int8_t rslt = BME280_OK;

    if (dev != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BME280_I2C_INTF)
        {
            printf("I2C Interface\n");

            dev_addr = BME280_I2C_ADDR_PRIM;
            dev->read = bme280_i2c_read;
            dev->write = bme280_i2c_write;
            dev->intf = BME280_I2C_INTF;
			
            /* set the sensor interface as I2C with 100kHz speed */
            ESP_ERROR_CHECK(i2c_master_init());
        }
        /* Bus configuration : SPI */
        else if (intf == BME280_SPI_INTF)
        {
            printf("SPI Interface\n");
            printf("SPI Interface for ESP32 not yet supported");

            ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
            rslt = BME280_E_DEV_NOT_FOUND;
        }

        /* Holds the I2C device addr or SPI chip selection */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bme280_delay_us;

    }
    else
    {
        rslt = BME280_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Function deinitializes esp32 i2c device.
 */
void bme280_esp32_deinit(void)
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI("BME280 Driver", "I2C de-initialized successfully");
}
