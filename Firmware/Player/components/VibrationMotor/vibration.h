#ifndef VIBRATION_H      /* Include guard */
#define VIBRATION_H


/****************************************** Header includes *******************************************/

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP32 */
#include "driver/gpio.h"

#include "vibration.h"



/*************************************** Structure Definitions ****************************************/

/*!
 * @brief Vibration motor.
 */
typedef struct {
    gpio_num_t enable_io_num;
    uint8_t nb_times;
    uint32_t lenght_ms;
} vibration_motor_t;



/*********************************** Function Prototype Declarations **********************************/

/*!
 * @brief This public function is used to initialize the vibration motor.
 * 
 * @param[in] vibration  :Structure instance of vibration_motor_t.
 *
 * @return Nothing
 */
void vibration_init(vibration_motor_t vibration);


/*!
 * @brief This public function is used to enable vibration for a
 *        given lenght and number of time.
 * 
 * @param[in] vibration  :Structure instance of vibration_motor_t.
 *
 * @return Nothing
 */
void vibration_enable(vibration_motor_t vibration);



#endif      /* End of include guard */

/******************************************************************************************************/