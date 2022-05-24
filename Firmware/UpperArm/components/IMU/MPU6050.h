#ifndef _MPU6050_H_   /* Include guard */
#define _MPU6050_H_



/****************************************** Header includes *******************************************/

#include "esp_err.h"



/********************************************* Constants **********************************************/

#define MPU6050_I2C_ADDR                        0x68

#define MPU6050_REG_CONFIG                      0x19
#define MPU6050_REG_SMPRT_DIV                   0x1A
#define MPU6050_REG_ACCEL_CONFIG                0x1C
#define MPU6050_REG_INT_ENABLE                  0x38
#define MPU6050_REG_INT_STATUS                  0x3A
#define MPU6050_REG_ACCEL_XOUT_H                0x3B
#define MPU6050_REG_ACCEL_XOUT_L                0x3C
#define MPU6050_REG_ACCEL_YOUT_H                0x3D
#define MPU6050_REG_ACCEL_YOUT_L                0x3E
#define MPU6050_REG_ACCEL_ZOUT_H                0x3F
#define MPU6050_REG_ACCEL_ZOUT_L                0x40
#define MPU6050_REG_PWR_MGMT_1                  0x6B
#define MPU6050_REG_PWR_MGMT_2                  0x6C
#define MPU6050_REG_WHO_AM_I                    0x75



/*************************************** Structure Definitions ****************************************/

typedef struct {
    float accel_data[3];
    float accel_res;
} MPU6050_sensor_t;


typedef struct {
	uint8_t DLPF_CFG		        : 2;
	uint8_t EXT_SYNC_SET	        : 3;
	uint8_t reserved_0		        : 2;
} MPU6050_CONFIG_t;


typedef struct {
	uint8_t reserved_0		        : 3;
	uint8_t AFS_SEL			        : 2;
	uint8_t ZA_ST			        : 1;
	uint8_t YA_ST			        : 1;
	uint8_t XA_ST			        : 1;
} MPU6050_ACCEL_CONFIG_t;


typedef struct {
	uint8_t ACCEL_XOUT_H;
} MPU6050_ACCEL_XOUT_H_t;


typedef struct {
	uint8_t ACCEL_XOUT_L;
} MPU6050_ACCEL_XOUT_L_t;


typedef struct {
	uint8_t ACCEL_YOUT_H;
} MPU6050_ACCEL_YOUT_H_t;


typedef struct {
	uint8_t ACCEL_YOUT_L;
} MPU6050_ACCEL_YOUT_L_t;


typedef struct {
	uint8_t ACCEL_ZOUT_H;
} MPU6050_ACCEL_ZOUT_H_t;


typedef struct {
	uint8_t ACCEL_ZOUT_L;
} MPU6050_ACCEL_ZOUT_L_t;


typedef struct {
	uint8_t SIG_COND_RESET			: 1;
	uint8_t I2C_MST_RESET			: 1;
	uint8_t FIFO_RESET				: 1;
	uint8_t reserved_1 				: 1;
	uint8_t	I2C_IF_DIS				: 1;
	uint8_t I2C_MST_EN				: 1;
	uint8_t FIFO_EN 				: 1;
	uint8_t reserved_0 				: 1;
} MPU6050_USER_CTRL_t;


typedef struct {
	uint8_t CLKSEL					: 3;
	uint8_t TEMP_DIS				: 1;
	uint8_t reserved_0 				: 1;
	uint8_t CYCLE					: 1;
	uint8_t SLEEP					: 1;
	uint8_t DEVICE_RESET			: 1;
} MPU6050_PWR_MGMT_1_t;


typedef struct{
	uint8_t STBY_ZG				    : 1;
	uint8_t STBY_YG				    : 1;
	uint8_t STBY_XG				    : 1;
	uint8_t STBY_ZA				    : 1;
	uint8_t STBY_YA				    : 1;
	uint8_t STBY_XA				    : 1;
	uint8_t LP_WAKE_CTRL			: 2;
} MPU6050_PWR_MGMT_2_t;


typedef struct {
    uint8_t WHO_AM_I;
}  MPU6050_WHO_AM_I_t;



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief 3-axis index.
 */
typedef enum {
    X = 0,      // x-axis
    Y,          // y-axis
    Z           // z-axis
} three_axis_index_t;



/*********************************** Function Prototype Declarations **********************************/

/*!
 * @brief This public function is used to initialize the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of MPU6050_sensor_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t MPU6050_init(MPU6050_sensor_t *mpu6050);


/*!
 * @brief This public function is used to fetch new accelerometer data
 *        from the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of MPU6050_sensor_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t MPU6050_read_accel(MPU6050_sensor_t *mpu6050);



#endif  /* End of include guard */

/******************************************************************************************************/

