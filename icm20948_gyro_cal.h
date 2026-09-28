#ifndef ICM20948_GYRO_CAL_H
#define ICM20948_GYRO_CAL_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    uint8_t who_am_i;
} ICM20948_GYRO_t;

typedef struct
{
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;

    float x_dps;
    float y_dps;
    float z_dps;

    float x_rad_s;
    float y_rad_s;
    float z_rad_s;
} ICM20948_GYRO_Data_t;

typedef struct
{
    float bias_x_raw;
    float bias_y_raw;
    float bias_z_raw;

    uint32_t sample_count;
    uint8_t valid;
} ICM20948_GYRO_Calibration_t;

HAL_StatusTypeDef ICM20948_GYRO_Init(
        ICM20948_GYRO_t *dev,
        SPI_HandleTypeDef *hspi,
        GPIO_TypeDef *cs_port,
        uint16_t cs_pin);

HAL_StatusTypeDef ICM20948_GYRO_ReadRaw(
        ICM20948_GYRO_t *dev,
        int16_t *gx,
        int16_t *gy,
        int16_t *gz);

HAL_StatusTypeDef ICM20948_GYRO_CalibrateStationary(
        ICM20948_GYRO_t *dev,
        ICM20948_GYRO_Calibration_t *cal,
        uint16_t sample_count,
        uint32_t sample_delay_ms);

HAL_StatusTypeDef ICM20948_GYRO_ReadCalibrated(
        ICM20948_GYRO_t *dev,
        const ICM20948_GYRO_Calibration_t *cal,
        ICM20948_GYRO_Data_t *data);

#endif
