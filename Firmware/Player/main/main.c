/*******************************************************************************************************
**
** @brief     XstractiK Domination Project  -  Player-v0.3.0 Firmware
**
** @copyright Copyright © 2021 GHS-Tech. All rights reserved.
** 
** @file	  main.cpp
** @author    Souhail Ghafoud
** @date	  April 29, 2022
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
#include <freertos/semphr.h>
#include "freertos/queue.h"

/* ESP32 drivers */
#include "driver/gpio.h"    // GPIO

/* NVS */
#include "nvs_flash.h"

/* BLE */
#include "ble.h"

/* I2C */
#include "i2c.h"

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

#define CP_NOTIF_TASK_STACK             (1024 * 3)                  // Stack size in bytes
#define CP_NOTIF_TASK_PRIORITY          (tskIDLE_PRIORITY + 2)      // Priority level
#define CP_NOTIF_TASK_CORE              APP_CORE                    // CPU core ID

#define CP_NOTIF_DATA_LEN               20
#define CP_NOTIF_ARG_LEN                3

#define CP_INFO_QUEUE_LEN               10

#define MAX_CP_DEVICES		            3

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
 * @brief Team id.
 */
typedef enum {
    TEAM_A = 0,
    TEAM_B,
    TEAM_NONE
} team_id_t;


/*!
 * @brief Player life state.
 */
typedef enum {
    PLAYER_INGAME = 0,
    PLAYER_ELIMINATED    
} player_life_state_t;


/*!
 * @brief CP id.
 */
typedef enum {
    CP_A = 0,
    CP_B,
    CP_C
} cp_id_t;


/*!
 * @brief CP capture state.
 */
typedef enum {
    CP_LOST = 0,
    CP_CAPTURED,
    CP_CAPTURING    
} cp_capture_state_t;


/*!
 * @brief CP notification args.
 */
typedef enum {
    CP_NOTIF_ARG_CP_ID = 0,
    CP_NOTIF_ARG_CP_STATUS,
    CP_NOTIF_ARG_DOMINANT_TEAM
} cp_notif_args_t;



/*************************************** Structure Definitions ****************************************/

/*!
 * @brief Player information.
 */
typedef struct {
    player_life_state_t life_state;
    uint32_t eliminated_count;
} player_info_t;


/*!
 * @brief CP information.
 */
typedef struct {
    cp_id_t id;
    cp_capture_state_t capture_state;
    team_id_t dominant_team;
} cp_info_t;



/***************************************** Static Variables *******************************************/

static QueueHandle_t s_cp_info_queue = NULL;                // CP notification Queue handle
static SemaphoreHandle_t s_cp_notif_irq_semaphore = NULL;   // CP notification IRQ Semaphore handle


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


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};


static esp_bd_addr_t cp_ble_addr_list[MAX_CP_DEVICES] = {{0x9C, 0x9C, 0x1F, 0xC7, 0x33, 0x22},
                                                         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
                                                         {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};



/*********************************************** ISRs *************************************************/

/*!
 * @brief This interrupt service routine is used to .
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void cp_notif_irq_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Higher priority task flag

    xSemaphoreGiveFromISR(s_cp_notif_irq_semaphore, &xHigherPriorityTaskWoken);

    /* Wake up higher priority task immediately */
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used to delay a task for a period
 *        of time in milliseconds.
 */
static void delay_ms(uint32_t period_ms)
{
    vTaskDelay((period_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}



/****************************************** App Core Tasks ********************************************/

/*!
 * @brief This internal task is used for player status management through
 *        arm position monitoring.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void player_status_task(void *arg)
{
    player_info_t player = {0};
    MPU6050_sensor_t mpu6050 = {0};
    vibration_motor_t vibration = {0};
    
    /* Initialize MPU6050 sensor */
    if (ESP_OK != MPU6050_init(&mpu6050)) {
        printf("MPU6050 I2C error\n");
    }
    
    /* Start BLE advertising */
    ble_start_advertising(ble_adv_params);

    /* Init vibration motor */
    vibration.enable_io_num = PIN_VIBRATION_EN;
    vibration.nb_time = 1;          // 1 time
    vibration.duration_ms = 150;    // 0.1 second
    vibration_init(vibration);      // Vibrate

    while (1) {
        /* Fetch new accelerometer data from MPU6050 */
        if (ESP_OK != MPU6050_read_accel(&mpu6050)) {
            printf("MPU6050 I2C error\n");
        }

        /* Check player's arm position to determine life status.
         *  - Arm raised  = accel_data[Y] < -0.36 g  (Eliminated)
         *  - Arm lowered = accel_data[Y] > -0.36 g  (In game)
         * */
        if (-0.36f > mpu6050.accel_data[Y])
        {            
            /*********************** Player eliminated ***********************/

            /* Stop BLE advertising */
            if (ESP_OK != ble_stop_advertising()) {
                printf("ERROR : BLE stop adv failed\n\n");
            }

            /* Update life state and increase eliminated count */
            player.life_state = PLAYER_ELIMINATED;
            player.eliminated_count++;

            /* Vibrate player's device to indicate eliminated */
            vibration.nb_time = 1;          // 1 time
            vibration.duration_ms = 200;    // 0.2 second
            vibration_enable(vibration);    // Vibrate

            /* Wait for player's arm to be lowered */
            while(-0.36f > mpu6050.accel_data[Y]) {            
                /* Fetch new accelerometer data from MPU6050 */
                if (ESP_OK != MPU6050_read_accel(&mpu6050)) {
                    printf("MPU6050 I2C error\n");
                }
                delay_ms(150);  // Delay before next data sampling
            }

            /********************** Player back in game **********************/
            
            /*!
            * @todo Confirmation from referee or player's headquater
            *       to get back in game.
            * */

            /* Start BLE advertising */
            if (ESP_OK != ble_start_advertising(ble_adv_params)) {
                printf("ERROR : BLE start adv failed\n\n");
            }

            player.life_state = PLAYER_INGAME;  // Update player life state

            /* Vibrate player's device to indicate backk in game */
            vibration.nb_time = 2;          // 1 time
            vibration.duration_ms = 200;    // 0.2 second
            vibration_enable(vibration);    // Vibrate
        }
                
        /* DEBUG *//* 
        for (uint8_t axis = 0; axis < 3; axis++) {
            printf("Accel[%d] = %.3f\n", axis, mpu6050.accel_data[axis]); 
        }
        printf("Player elimination count = %d\n\n", player.eliminated_count); */

        delay_ms(150);  // Delay before next data sampling
    }
}


/*!
 * @brief This internal task is used to receive notifications from all CPs
 *        when their status changes.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
void cp_notif_task(void *arg)
{
    uint8_t cp_notif_data[CP_NOTIF_DATA_LEN] = {0};
    uint8_t cp_notif_arg[CP_NOTIF_ARG_LEN] = {0};
    int cp_notif_data_len = 0;
    cp_notif_args_t cp_notif_arg_index = 0;
    char cp_notif_preamble[9] = "CP-v";
    cp_info_t cp = {0};
    vibration_motor_t vibration = {PIN_VIBRATION_EN, 2, 200};
    
    /* Create CP notif queue */
    s_cp_info_queue = xQueueCreate(CP_INFO_QUEUE_LEN, sizeof(cp_info_t));

    /* Create binary semaphore for CP notification events */
    s_cp_notif_irq_semaphore = xSemaphoreCreateBinary();

    /* Set LoRa DIO0 pin as interrupt pin for CP notification events */
    gpio_set_direction(PIN_LORA_DIO0, GPIO_MODE_INPUT);     // Input mode
    gpio_set_intr_type(PIN_LORA_DIO0, GPIO_INTR_POSEDGE);   // Rising edge interrupt

    /* Setup ISR on LoRa DIO0 pin for CP notification events */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(PIN_LORA_DIO0, cp_notif_irq_isr, (void *)PIN_LORA_DIO0);
    
    /* Preamble in CP notifications */
    strcat(cp_notif_preamble, FIRMWARE_VERSION);
    
    /* Init LoRa module */
    lora_init();
    lora_set_frequency(915e6);
    lora_enable_crc();

    while (1) {
        /* Wait for CP notification */
        lora_receive();
        xSemaphoreTake(s_cp_notif_irq_semaphore, portMAX_DELAY);

        /* Fetch CP notification data */
        cp_notif_data_len = lora_receive_packet(cp_notif_data, sizeof(cp_notif_data));

        /* Parse CP notification if LoRa packet's preamble is valid */
        if (NULL != strstr((const char *)cp_notif_data, cp_notif_preamble)) {
            for (uint8_t i = 0; i < cp_notif_data_len; i++) {
                /* Retrieve LoRa packet args next to a comma char.
                 *  Data frame :
                 *   - arg[0] = 0 or 1 or 2 (A, B, C) -> cp.id
                 *   - arg[1] = 0 or 1 (Captured, Capturing) -> cp.capture_state
                 *   - arg[2] = 0 or 1 (A, B) -> cp.dominant_team
                 * */
                if (',' == cp_notif_data[i]) {
                    cp_notif_arg[cp_notif_arg_index] = atoi((const char *)&cp_notif_data[i+1]);
                    cp_notif_arg_index++;
                }
            }

            /* Save CP information received via LoRa comms */
            cp.id = cp_notif_arg[CP_NOTIF_ARG_CP_ID];
            cp.capture_state = cp_notif_arg[CP_NOTIF_ARG_CP_STATUS];
            cp.dominant_team = cp_notif_arg[CP_NOTIF_ARG_DOMINANT_TEAM];

            /* Vibrate player's device to indicate a CP status change */
            vibration_enable(vibration);

            /*!
            * @todo Send the CP information above to another task (display on LCD)
            * */
            /* Enqueue CP information for lcd_display_task */
            xQueueSend(s_cp_info_queue, &cp, 0);

            /* DEBUG */
            printf("Received CP notif : Team-%c %s CP-%c\n\n",
                    cp.dominant_team == 0 ? 'A' : 'B',
                    cp.capture_state == 0 ? "captured" : "is capturing",
                    cp.id == 0 ? 'A' : 'B');
            printf("**********************************************\n\n");
        }

        /* Reset LoRa arrays and variable used for comms */
        memset(cp_notif_data, 0, sizeof(cp_notif_data));
        memset(cp_notif_arg, 0, sizeof(cp_notif_arg));
        cp_notif_arg_index = 0;
    }
}



/****************************************** Pro Core Tasks ********************************************/

void app_main(void)
{
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    printf("**********************************************\n");
    printf("XstractiK Domination Project  -  Player-v%s\n", FIRMWARE_VERSION);
    printf("**********************************************\n\n");
    
    /* Init I2C Peripheral */
    if (ESP_OK != i2c_init(PIN_I2C_SDA, PIN_I2C_SCL)) {
        printf("I2C init error\n\n");
    }

    /* Init NVS */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Init BLE radio */
    ble_init(ble_scan_params, raw_adv_data);
        
    /* Create a task to receive CP notifications via LoRa comms */
    xTaskCreatePinnedToCore(&cp_notif_task,
                            "CP notif",
                            CP_NOTIF_TASK_STACK,
                            NULL,
                            CP_NOTIF_TASK_PRIORITY,
                            NULL,
                            CP_NOTIF_TASK_CORE);
    
    /* Create a task for player status monitoring */
    xTaskCreatePinnedToCore(&player_status_task,
                            "Player status",
                            PLAYER_STATUS_TASK_STACK,
                            NULL,
                            PLAYER_STATUS_TASK_PRIORITY,
                            NULL,
                            PLAYER_STATUS_TASK_CORE);
}



/******************************************************************************************************/