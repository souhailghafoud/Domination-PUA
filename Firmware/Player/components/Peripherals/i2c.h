#ifndef _I2C_H_   /* Include guard */
#define _I2C_H_



/************************************* Header Includes *************************************/

#include <stdlib.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <driver/i2c.h>



/***************************** Function Prototype Declarations *****************************/

/*!
 * @brief This public function is used to initialize the I2C port
 *        as master.
 * 
 * @param[in] sda  :Pin for I2C data line
 * @param[in] scl  :Pin for I2C clock line
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t i2c_init(int8_t sda, int8_t scl);


/*!
 * @brief This public function is used to send data to a slave
 *        through the I2C port.
 *
 * @param[in] addr  :Slave address
 * @param[in] reg   :Register to write
 * @param[in] data  :Data to send
 * @param[in] len   :Data lenght
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t i2c_send_data(uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len);


/*!
 * @brief This public function is used to read data from a slave
 *        through the I2C port.
 *
 * @param[in] addr  :Slave address
 * @param[in] reg   :Register to read
 * @param[in] data  :Data read from reg
 * @param[in] len   :Data lenght
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t i2c_read_data(uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len);



#endif  /* End of include guard */

/*********************************************************************************************/