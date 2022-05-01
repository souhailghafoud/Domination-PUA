/****************************************** Header includes *******************************************/

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP32 */
#include "driver/gpio.h"

#include "vibration.h"



/********************************************* Constants **********************************************/

#define APP_CORE                        1                           // ESP32 Core 1

#define VIBRATION_TASK_STACK           (1024 * 2)                   // Stack size in bytes
#define VIBRATION_TASK_PRIORITY        (tskIDLE_PRIORITY + 1)       // Priority level
#define VIBRATION_TASK_CORE            APP_CORE                     // CPU core ID

#define VIBRATION_QUEUE_LEN             10                          // Vibration data Queue length

#define HIGH			                1
#define LOW 			                0



/****************************************** Static Variables ******************************************/

static TaskHandle_t s_vibration_task_handle = NULL;       // Vibration task handle
static QueueHandle_t s_vibration_queue = NULL;            // Vibration data Queue handle



/****************************************** Private Functions *****************************************/

/*!
 * @brief This public function is used to delay a task for a period
 *        of time in milliseconds.
 */
static void delay_ms(uint32_t period_ms)
{
    vTaskDelay((period_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}



/****************************************** App Core Tasks ********************************************/

/*!
 * @brief The internal task is used to enable vibration for a
 *        given number of times and lenght of time.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void vibration_task(void *arg)
{
    vibration_motor_t vibration = {0};

    while (1) {        
        /* Wait for vibration data then dequeue */
        xQueueReceive(s_vibration_queue, &vibration, portMAX_DELAY);

        /* Vibrate during the given lenght of time 
         * for a given number of times.
         * */
        for (uint8_t i = 0; i < vibration.nb_time; i++) {
            gpio_set_level(vibration.enable_io_num, HIGH);
            delay_ms(vibration.duration_ms);
            gpio_set_level(vibration.enable_io_num, LOW);
            delay_ms(100);
        }

        delay_ms(1000);
    }
}



/****************************************** Public Functions ******************************************/

/*!
 * @brief This public function is used to initialize the vibration motor.
 */
void vibration_init(vibration_motor_t vibration)
{
    /* Set GPIO pin */
    gpio_set_direction(vibration.enable_io_num, GPIO_MODE_OUTPUT);
    gpio_set_level(vibration.enable_io_num, LOW);
    
    /* Create vibration data queue */
    s_vibration_queue = xQueueCreate(VIBRATION_QUEUE_LEN, sizeof(vibration_motor_t));
    
    /* Create a task for vibration */
    xTaskCreatePinnedToCore(&vibration_task,
                            "Vibration Task",
                            VIBRATION_TASK_STACK,
                            &vibration,
                            VIBRATION_TASK_PRIORITY,
                            &s_vibration_task_handle,
                            VIBRATION_TASK_CORE);

    /* Vibrate motor only if a duration was given */
    if (vibration.duration_ms > 0) {
        vibration_enable(vibration);
    }
}


/*!
 * @brief This public function is used to enable the vibration for a
 *        given number of times and lenght of time.
 */
void vibration_enable(vibration_motor_t vibration)
{
    /* Enqueue vibration data */
    xQueueSend(s_vibration_queue, &vibration, 0);
}



/******************************************************************************************************/