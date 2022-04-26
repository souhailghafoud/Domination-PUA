/****************************************** Header includes *******************************************/

#include "vibration.h"



/********************************************* Constants **********************************************/

#define APP_CORE                        1                           // ESP32 Core 1

#define VIBRATION_TASK_STACK           (1024 * 2)                   // Stack size in bytes
#define VIBRATION_TASK_PRIORITY        (tskIDLE_PRIORITY + 1)       // Priority level
#define VIBRATION_TASK_CORE            APP_CORE                     // CPU core ID

#define HIGH			                1
#define LOW 			                0


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
    vibration_motor_t vibration = *(vibration_motor_t *)arg;    // Store vibration struct

    while (1) {
        /* Vibrate during the given lenght of time 
         * for a given number of times.
         * */
        for (uint8_t i = 0; i < vibration.nb_times; i++) {
            gpio_set_level(vibration.enable_io_num, HIGH);
            delay_ms(vibration.lenght_ms);
            gpio_set_level(vibration.enable_io_num, LOW);
            delay_ms(500);
        }
        
        /* Delete this task */
        vTaskDelete(NULL);
    }
}



/****************************************** Public Functions ******************************************/

/*!
 * @brief This public function is used to initialize the vibration motor.
 */
void vibration_init(vibration_motor_t vibration)
{
    gpio_set_direction(vibration.enable_io_num, GPIO_MODE_OUTPUT);
    gpio_set_level(vibration.enable_io_num, LOW);

    vibration_enable(vibration);
}


/*!
 * @brief This public function is used to start a task for vibration.
 */
void vibration_enable(vibration_motor_t vibration)
{
    /* Create a task for vibration */
    xTaskCreatePinnedToCore(&vibration_task,
                            "Vibration Task",
                            VIBRATION_TASK_STACK,
                            &vibration,
                            VIBRATION_TASK_PRIORITY,
                            NULL,
                            VIBRATION_TASK_CORE);
}



/******************************************************************************************************/