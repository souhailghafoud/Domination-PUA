/*******************************************************************************************************
**
** @brief     XstractiK Domination Project  -  Player-v0.3.0 Firmware
**
** @copyright Copyright © 2021 GHS. All rights reserved.
** 
** @file	  main.cpp
** @author    Souhail Ghafoud
** @date	  April 26, 2022
** @version	  0.3.0
**
*******************************************************************************************************/



/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP32 */
#include "driver/gpio.h"

/* NVS */
#include "nvs_flash.h"

/* Bluetooth */
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

/* LOG */
#include "esp_log.h"

/* IMU */
#include "MPU6050.h"

/* LoRa */
#include "lora.h"

/* Vibration motor */
#include "vibration.h"



/********************************************* Constants **********************************************/

#define FIRMWARE_VERSION                "0.3.0"                     // Firmware version

#define PRO_CORE                        0                           // ESP32 Core 0
#define APP_CORE                        1                           // ESP32 Core 1

#define PLAYER_STATUS_TASK_STACK        (1024 * 3)                  // Stack size in bytes
#define PLAYER_STATUS_TASK_PRIORITY     (tskIDLE_PRIORITY + 3)      // Priority level
#define PLAYER_STATUS_TASK_CORE         APP_CORE                    // CPU core ID

#define CP_STATUS_TASK_STACK            (1024 * 3)                  // Stack size in bytes
#define CP_STATUS_TASK_PRIORITY         (tskIDLE_PRIORITY + 2)      // Priority level
#define CP_STATUS_TASK_CORE             APP_CORE                    // CPU core ID

#define MAX_CENTRALS                    3
#define CENTRAL_QUEUE_LEN               MAX_CENTRALS                // Central devices Queue length

#define MAX_RSSI_COUNT                  10
#define RSSI_RADIUS                     -65                         // dBm

#define LORA_PACKET_LEN                 40
#define LORA_PACKET_ARG_LEN             3

#define HIGH			                1
#define LOW 			                0

/* I2C pins */
#define PIN_I2C_SCL                     GPIO_NUM_22
#define PIN_I2C_SDA                     GPIO_NUM_21

/* SPI pins */
#define PIN_SPI_MOSI                    GPIO_NUM_23
#define PIN_SPI_MISO                    GPIO_NUM_19
#define PIN_SPI_CLK                     GPIO_NUM_18

/* LoRa pins */
#define PIN_LORA_SS                     GPIO_NUM_5
#define PIN_LORA_DIO0                   GPIO_NUM_17
#define PIN_LORA_RESET                  GPIO_NUM_16

/* Vibration Motor pins */
#define PIN_VIBRATION_EN                GPIO_NUM_4

/* Battery Management pins */
#define PIN_FUELGAUGE_ALERT             GPIO_NUM_2
#define PIN_CHARGER_STAT                GPIO_NUM_15



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief LED colors.
 */
typedef enum {
    LED_OFF = -1,
    LED_GREEN,
    LED_RED
} led_color_t;


/*!
 * @brief Central proximity status.
 */
typedef enum {
    CENTRAL_OUT_OF_RANGE = 0,
    CENTRAL_IN_RANGE,
    CENTRAL_ABSENT
} central_proximity_t;


/*!
 * @brief Player status.
 */
typedef enum {
    PLAYER_ALIVE = 0,
    PLAYER_DEAD    
} player_status_t;


/*!
 * @brief CP id.
 */
typedef enum {
    CP_A = 1,
    CP_B,
    CP_C
} cp_id_t;


/*!
 * @brief CP status.
 */
typedef enum {
    CP_CAPTURED = 0,
    CP_CAPTURING    
} cp_status_t;


/*!
 * @brief Team id.
 */
typedef enum {
    TEAM_A = 0,
    TEAM_B,
    TEAM_NONE
} team_id_t;


/*!
 * @brief LoRa packet args.
 */
typedef enum {
    PACKET_ARG_CP_ID = 0,
    PACKET_ARG_CP_STATUS,
    PACKET_ARG_DOMINANT_TEAM
} lora_packet_args_t;



/*************************************** Structure Definitions ****************************************/

/*!
 * @brief Central device.
 */
typedef struct {
    esp_bd_addr_t address;
    central_proximity_t proximity;
    int rssi;
    uint8_t rssi_counter;
} central_dev_t;


/*!
 * @brief Player info.
 */
typedef struct {
    player_status_t status;
    uint32_t death_count;
} player_info_t;


/*!
 * @brief CP info.
 */
typedef struct {
    cp_id_t id;
    cp_status_t status;
    team_id_t dominant_team;
} cp_info_t;



/***************************************** Static Variables *******************************************/

static QueueHandle_t s_central_queue = NULL;            // Central devices Queue handle
static SemaphoreHandle_t s_lora_irq_semaphore = NULL;   // LoRa IRQ Semaphore handle


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};


static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static uint8_t raw_adv_data[20] = {
    /* Len:2, Type:1 (flags), Data:6 */
    0x02, 0x01, 0x06,
    /* Len:2, Type:a (tx power), Data:eb*/
    0x02, 0x0a, 0xeb,
    /* Len:3, Type:3 (Complete List of 16-bit Service Class UUIDs), Data: FF 00 */
    0x03, 0x03, 0xFF, 0x01,
    /* Len:5, Type:9 (Complete Local Name) */
    0x05, 0x09, 'P', '-', '0', '1'
};



/*********************************************** ISRs *************************************************/

/*!
 * @brief This interrupt service routine is used to .
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void lora_irq_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Higher priority task flag

    xSemaphoreGiveFromISR(s_lora_irq_semaphore, &xHigherPriorityTaskWoken);

    /* Wake up higher priority task immediately */
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



/***************************************** Private Functions ******************************************/

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    /* Store central device address and RSSI value */
                    central_dev_t central = {0};
                    memcpy(central.address, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                    central.rssi = scan_result->scan_rst.rssi;
                    /* Enqueue central device */
                    xQueueSend(s_central_queue, &central, 0);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            //scan start complete event to indicate scan start successfully or failed
            if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Scan start failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("Start scan successfully\n\n");
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            //scan stop complete event to indicate scan stop successfully or failed
            if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Scan stop failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("Stop scan successfully\n\n");
            }
            break;
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            break;
        }
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&ble_adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //adv start complete event to indicate adv start successfully or failed
            if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Adv start failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("start adv successfully\n\n");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            //adv stop complete event to indicate adv stop successfully or failed
            if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                printf("Adv stop failed: %s\n\n", esp_err_to_name(err));
            }
            else {
                printf("Stop adv successfully\n\n");
            }
            break;
        default:
            break;
    }
}



/***************************************** Public Functions *******************************************/

/*!
 * @brief This public function is used to delay a task for a period
 *        of time in milliseconds.
 */
void delay_ms(uint32_t period_ms)
{
    vTaskDelay((period_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}



/****************************************** App Core Tasks ********************************************/

/*!
 * @brief This internal task is used to..
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void player_status_task(void *arg)
{
    esp_bd_addr_t central_addr_list[MAX_CENTRALS] = {{0x9C, 0x9C, 0x1F, 0xC7, 0x33, 0x22},
                                                     {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                     {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
    central_dev_t central[MAX_CENTRALS] = {0};
    central_dev_t central_temp = {0};
    uint8_t central_index = 0;
    bool b_central_found = false;
    player_info_t player = {0};
    MPU6050_sensor_t mpu6050 = {0};
    vibration_motor_t vibration = {0};
    
    /* Create central devices queue */
    s_central_queue = xQueueCreate(CENTRAL_QUEUE_LEN, sizeof(central_dev_t));

    /* Initialize MPU6050 sensor */
    if (ESP_OK != MPU6050_init(&mpu6050)) {
        printf("MPU6050 I2C error\n");
    }

    /* Init vibration motor */
    vibration.enable_io_num = PIN_VIBRATION_EN;
    vibration.nb_times = 1;     // 1 time
    vibration.lenght_ms = 500;  // 0.5 second
    vibration_init(vibration);

    while (1) {
        /* Fetch new accelerometer data from MPU6050 */
        if (ESP_OK != MPU6050_read_accel(&mpu6050)) {
            printf("MPU6050 I2C error\n");
        }

        /* Check if player's arm is raised to indicate life status */
        if ((0.350f) <= mpu6050.accel_data[Y]) {
            /* Player is dead */
            esp_ble_gap_stop_advertising(); // Stop advertising
            player.status = PLAYER_DEAD;    // Set player status to dead
            player.death_count++;           // Increase player's death count

            /* Vibrate player's device to indicate death */
            vibration.nb_times = 1;     // 1 time
            vibration.lenght_ms = 1000; // 1 second
            vibration_enable(vibration);
        
            /* Start scanning for central devices (0 means scan permanently) */
            esp_ble_gap_start_scanning(0);

            /* Loop until player is near a central device to come back to life */
            while (PLAYER_DEAD == player.status) {
                /* Wait for central device then dequeue */
                xQueueReceive(s_central_queue, &central_temp, portMAX_DELAY);

                b_central_found = false; // Reset variable

                /* Search through all central devices */
                for (central_index = 0; b_central_found != true || central_index < MAX_CENTRALS; central_index++) {
                    /* Find the corresponding central within the address list */
                    if (0 == memcmp(central_addr_list[central_index], central_temp.address, ESP_BD_ADDR_LEN)) {
                        /* Sum RSSI value and increase counter */
                        central[central_index].rssi += central_temp.rssi;
                        central[central_index].rssi_counter++;

                        /* RSSI values ready to be averaged */
                        if (MAX_RSSI_COUNT == central[central_index].rssi_counter) {
                            /* Average RSSI values */
                            central[central_index].rssi = (float)central[central_index].rssi / (float)central[central_index].rssi_counter;

                            /* Determine if the central is in range or not */
                            if (RSSI_RADIUS <= central[central_index].rssi) {
                                /* Central in range.
                                    * Make sure beacon is not already in range so this is   
                                    * run only on a proximity status change 
                                    * */
                                if (CENTRAL_IN_RANGE != central[central_index].proximity) {
                                    /* Player is back to life (Save new status) */
                                    player.status = PLAYER_ALIVE;
                                    /* Vibrate player's device to indicate revival */
                                    vibration.nb_times = 2;     // 2 time
                                    vibration.lenght_ms = 1000; // 1 second
                                    vibration_enable(vibration);
                                    /* Stop scanning and start advertising */
                                    esp_ble_gap_stop_scanning();
                                    esp_ble_gap_start_advertising(&ble_adv_params);
                                }
                            }
                            /* Reset RSSI value and counter */
                            central[central_index].rssi = 0;
                            central[central_index].rssi_counter = 0;
                        }
                        b_central_found = true; // Central device found
                    }
                }
            }
        }

        /* DEBUG *//*
        for (uint8_t axis = 0; axis < 3; axis++) {
            printf("Accel[%d] = %.3f\n", axis, mpu6050.accel_data[axis]); 
        }
        printf("Player death count = %d\n", player.death_count); 
        printf(" \n");*/

        delay_ms(250);
    }
}


/*!
 * @brief This internal task is used to..
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
void cp_status_task(void *arg)
{
    cp_info_t cp = {0};
    uint8_t lora_packet[LORA_PACKET_LEN] = {0};
    uint8_t lora_packet_arg[LORA_PACKET_ARG_LEN] = {0};
    int lora_packet_len = 0;
    lora_packet_args_t packet_arg_index = 0;
    vibration_motor_t vibration = {0};

    /* Create binary semaphore for LoRa Rx events */
    s_lora_irq_semaphore = xSemaphoreCreateBinary();

    /* Set inpout mode for LoRa Rx Pins */
    gpio_set_direction(PIN_LORA_DIO0, GPIO_MODE_INPUT);
    
    /* Set rising edge interrupt type for LoRa Rx Pins */
    gpio_set_intr_type(PIN_LORA_DIO0, GPIO_INTR_POSEDGE);

    /* Setup ISR for LoRa Rx Pins */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(PIN_LORA_DIO0, lora_irq_isr, (void *)PIN_LORA_DIO0);

    /* Init LoRa module */
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();

    /* Init vibration motor struct variables */
    vibration.enable_io_num = PIN_VIBRATION_EN;
    vibration.nb_times = 2;     // 2 time
    vibration.lenght_ms = 1000; // 1 second

    while (1) {
        /* Set into receive mode */
        lora_receive(); 
        
        /* Wait for LoRa IRQ */
        xSemaphoreTake(s_lora_irq_semaphore, portMAX_DELAY);

        lora_packet_len = lora_receive_packet(lora_packet, sizeof(lora_packet));

        printf("Received packet : %s\n\n", lora_packet);

        /* Parse LoRa packet */
        for (uint8_t i = 0; i < lora_packet_len; i++) {
            /* Retreive LoRa packet args
             *  arg[0] : cp_id -> 1 or 2 or 3 (A, B, C)
             *  arg[1] : cp_status -> 0 or 1 (Captured, Capturing)
             *  arg[2] : dominant_team -> 0 or 1 (A, B)
             * */
            if (':' == lora_packet[i]) {
                lora_packet_arg[packet_arg_index] = lora_packet[i+1];
                packet_arg_index++;
            }
        }

        /* Save CP info received via LoRa comms */
        cp.id = lora_packet_arg[PACKET_ARG_CP_ID];
        cp.status = lora_packet_arg[PACKET_ARG_CP_STATUS];
        cp.dominant_team = lora_packet_arg[PACKET_ARG_DOMINANT_TEAM];

        /*!
         * @todo Send the CP info above to another task (display on LCD)
         * */

        /* Vibrate player's device to indicate a CP status change */
        vibration_enable(vibration);

        /* Reset LoRa arrays and variable used for comms */
        memset(lora_packet, 0, sizeof(lora_packet));
        memset(lora_packet_arg, 0, sizeof(lora_packet_arg));
        packet_arg_index = 0;
    }
}



/****************************************** Pro Core Tasks ********************************************/

void app_main(void)
{
    esp_err_t esp_status = ESP_OK;

    printf("\n\nXstractiK Domination Project  -  Player-v%s\n\n", FIRMWARE_VERSION);
    
    /* Init I2C Peripheral */
    if (ESP_OK != i2c_init(PIN_I2C_SDA, PIN_I2C_SCL)) {
        printf("I2C init error\n\n");
    }

    /* Init NVS */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Free unused BT Classic memory */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* BT controller configuration */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);            //Initialize BT controller
    esp_bt_controller_enable(ESP_BT_MODE_BLE);  // Enable BT controller
    
    esp_bluedroid_init();   // Init and alloc the resource for bluetooth
    esp_bluedroid_enable(); // Enable bluetooth

    /* Register the scan callback function to the gap module */
    esp_status = esp_ble_gap_register_callback(esp_gap_cb);
    if (esp_status != ESP_OK) {
        printf("Gap register error: %s\n\n", esp_err_to_name(esp_status));
    }
    
    /* Set advertising data */
    esp_status = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
    if (esp_status != ESP_OK){
        printf("Config adv data failed: %s\n\n", esp_err_to_name(esp_status));
    }
    
    /* Set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
        
    /* Create a task to receive CP status via LoRa comms */
    xTaskCreatePinnedToCore(&cp_status_task,
                            "CP status task",
                            CP_STATUS_TASK_STACK,
                            NULL,
                            CP_STATUS_TASK_PRIORITY,
                            NULL,
                            CP_STATUS_TASK_CORE);
    
    /* Create a task for player status monitoring */
    xTaskCreatePinnedToCore(&player_status_task,
                            "Player status task",
                            PLAYER_STATUS_TASK_STACK,
                            NULL,
                            PLAYER_STATUS_TASK_PRIORITY,
                            NULL,
                            PLAYER_STATUS_TASK_CORE);
}



/******************************************************************************************************/