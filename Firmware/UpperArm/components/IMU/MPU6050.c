
/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>         // For uint8_t definition 
#include <stdbool.h>        // For true/false definition

/* I2C */
#include "i2c.h"

/* MPU6050 */
#include "MPU6050.h"



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used to write in MPU6050 registers.
 */
static esp_err_t MPU6050_write_reg(uint8_t reg, uint8_t *data, uint32_t len)
{
    return i2c_send_data(MPU6050_I2C_ADDR, &reg, data, len);
}


/*!
 * @brief This private function is used to read MPU6050 registers.
 */
static esp_err_t MPU6050_read_reg(uint8_t reg, uint8_t *data, uint32_t len)
{
    return i2c_read_data(MPU6050_I2C_ADDR, &reg, data, len);
}



/***************************************** Public Functions *******************************************/

/*!
 * @brief This public function is used to initialize the MPU6050 sensor.
 */
esp_err_t MPU6050_init(MPU6050_sensor_t *mpu6050)
{
    /* Check device ID */
    uint8_t who_am_i = 0;
    if (ESP_OK != MPU6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) {
        return ESP_FAIL;
    }
    if (MPU6050_I2C_ADDR != who_am_i) {
        return ESP_FAIL;
    }

    /* Disable temperature sensor and enable cycle mode */
	MPU6050_PWR_MGMT_1_t pwr_mgmt_1 = {0};
    pwr_mgmt_1.TEMP_DIS = 1;
    pwr_mgmt_1.CYCLE = 1;
    if (ESP_OK != MPU6050_write_reg(MPU6050_REG_PWR_MGMT_1, (uint8_t *)&pwr_mgmt_1, sizeof(MPU6050_PWR_MGMT_1_t))) {
        return ESP_FAIL;
    }

    /* Disable gyroscope and set wake-up frequency to 5 Hz */
	MPU6050_PWR_MGMT_2_t pwr_mgmt_2 = {0};
    pwr_mgmt_2.STBY_ZG = 1;
    pwr_mgmt_2.STBY_YG = 1;
    pwr_mgmt_2.STBY_XG = 1;
    pwr_mgmt_2.LP_WAKE_CTRL = 1;
    if (ESP_OK != MPU6050_write_reg(MPU6050_REG_PWR_MGMT_2, (uint8_t *)&pwr_mgmt_2, sizeof(MPU6050_PWR_MGMT_2_t))) {
        return ESP_FAIL;
    }
    
    mpu6050->accel_res = 16384.0f;

    return ESP_OK;
}


/*!
 * @brief This public function is used to fetch new accelerometer data
 *        from the MPU6050 sensor.
 */
esp_err_t MPU6050_read_accel(MPU6050_sensor_t *mpu6050)
{
    uint8_t raw_accel[6] = {0};
    int16_t accel_16b[3] = {0};

    /* Fetch raw 3-axis accelerometer data from MPU6050 sensor (High and low bits) */
    if (ESP_OK != MPU6050_read_reg(MPU6050_REG_ACCEL_XOUT_H, raw_accel, 6)) {
        return ESP_FAIL;
    }
        
    /* Convert high and low bits into full 16 bits data (x, y, z) */
    accel_16b[X] = (int16_t)(((int16_t)raw_accel[0] << 8) | raw_accel[1]);
    accel_16b[Y] = (int16_t)(((int16_t)raw_accel[2] << 8) | raw_accel[3]);
    accel_16b[Z] = (int16_t)(((int16_t)raw_accel[4] << 8) | raw_accel[5]);

    /* Scale 3-axis accelerometer data according to the current resolution (2g, 4g, 8g, 16g)*/
    for (uint8_t axis = 0; axis < 3; axis++) {
        mpu6050->accel_data[axis] = ((float)accel_16b[axis] / mpu6050->accel_res);
    }

    return ESP_OK;
}


/******************************************************************************************************/