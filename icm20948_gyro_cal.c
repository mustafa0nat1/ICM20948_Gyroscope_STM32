#include "icm20948_gyro_cal.h"

/* ---------------- USER BANK 0 ---------------- */

#define ICM20948_REG_WHO_AM_I          0x00
#define ICM20948_REG_USER_CTRL         0x03
#define ICM20948_REG_PWR_MGMT_1        0x06
#define ICM20948_REG_PWR_MGMT_2        0x07
#define ICM20948_REG_GYRO_XOUT_H       0x33
#define ICM20948_REG_BANK_SEL          0x7F

#define ICM20948_WHO_AM_I_VALUE        0xEA

/* ---------------- USER BANK 2 ---------------- */

#define ICM20948_B2_GYRO_SMPLRT_DIV    0x00
#define ICM20948_B2_GYRO_CONFIG_1      0x01

#define ICM20948_BANK_0                0
#define ICM20948_BANK_2                2

/*
 * GYRO_CONFIG_1 = 0x11:
 *
 * GYRO_FS_SEL = 0  -> +/-250 dps
 * GYRO_DLPFCFG = 2
 * GYRO_FCHOICE = 1
 *
 * Sensitivity at +/-250 dps:
 * 131 LSB/(deg/s)
 */
#define ICM20948_GYRO_LSB_PER_DPS      131.0f

#define DEG_TO_RAD                     0.01745329251994329577f

/* ------------------------------------------------------- */

static void CS_Low(ICM20948_GYRO_t *dev)
{
    HAL_GPIO_WritePin(
            dev->cs_port,
            dev->cs_pin,
            GPIO_PIN_RESET);
}

static void CS_High(ICM20948_GYRO_t *dev)
{
    HAL_GPIO_WritePin(
            dev->cs_port,
            dev->cs_pin,
            GPIO_PIN_SET);
}

static HAL_StatusTypeDef WriteRegister(
        ICM20948_GYRO_t *dev,
        uint8_t address,
        uint8_t value)
{
    uint8_t tx[2];

    tx[0] = address & 0x7F;
    tx[1] = value;

    CS_Low(dev);

    HAL_StatusTypeDef status =
            HAL_SPI_Transmit(
                    dev->hspi,
                    tx,
                    2,
                    1000);

    CS_High(dev);

    return status;
}

static HAL_StatusTypeDef ReadRegister(
        ICM20948_GYRO_t *dev,
        uint8_t address,
        uint8_t *value)
{
    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};

    tx[0] = address | 0x80;

    CS_Low(dev);

    HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive(
                    dev->hspi,
                    tx,
                    rx,
                    2,
                    1000);

    CS_High(dev);

    if (status == HAL_OK)
    {
        *value = rx[1];
    }

    return status;
}

static HAL_StatusTypeDef ReadRegisters(
        ICM20948_GYRO_t *dev,
        uint8_t start_address,
        uint8_t *data,
        uint8_t length)
{
    if ((data == NULL) || (length == 0) || (length > 16))
    {
        return HAL_ERROR;
    }

    uint8_t tx[17] = {0};
    uint8_t rx[17] = {0};

    tx[0] = start_address | 0x80;

    CS_Low(dev);

    HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive(
                    dev->hspi,
                    tx,
                    rx,
                    length + 1,
                    1000);

    CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = rx[i + 1];
    }

    return HAL_OK;
}

static HAL_StatusTypeDef SelectBank(
        ICM20948_GYRO_t *dev,
        uint8_t bank)
{
    return WriteRegister(
            dev,
            ICM20948_REG_BANK_SEL,
            (uint8_t)((bank & 0x03) << 4));
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_GYRO_Init(
        ICM20948_GYRO_t *dev,
        SPI_HandleTypeDef *hspi,
        GPIO_TypeDef *cs_port,
        uint16_t cs_pin)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) ||
        (hspi == NULL) ||
        (cs_port == NULL))
    {
        return HAL_ERROR;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->who_am_i = 0;

    CS_High(dev);

    HAL_Delay(200);

    status = SelectBank(dev, ICM20948_BANK_0);
    if (status != HAL_OK) return status;

    status = ReadRegister(
            dev,
            ICM20948_REG_WHO_AM_I,
            &dev->who_am_i);

    if (status != HAL_OK)
    {
        return status;
    }

    if (dev->who_am_i != ICM20948_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    /*
     * Disable primary I2C slave interface because host uses SPI.
     */
    status = WriteRegister(
            dev,
            ICM20948_REG_USER_CTRL,
            0x10);

    if (status != HAL_OK) return status;

    /*
     * Wake device, CLKSEL = 1.
     */
    status = WriteRegister(
            dev,
            ICM20948_REG_PWR_MGMT_1,
            0x01);

    if (status != HAL_OK) return status;

    HAL_Delay(50);

    /*
     * Gyro-only project:
     *
     * DISABLE_ACCEL = 111
     * DISABLE_GYRO  = 000
     *
     * 0b00111000 = 0x38
     */
    status = WriteRegister(
            dev,
            ICM20948_REG_PWR_MGMT_2,
            0x38);

    if (status != HAL_OK) return status;

    HAL_Delay(10);

    status = SelectBank(dev, ICM20948_BANK_2);
    if (status != HAL_OK) return status;

    /*
     * Gyro sample rate divider = 10
     * Approximate output rate:
     * 1.1 kHz / (1 + 10) ~= 100 Hz
     */
    status = WriteRegister(
            dev,
            ICM20948_B2_GYRO_SMPLRT_DIV,
            10);

    if (status != HAL_OK) return status;

    /*
     * GYRO_CONFIG_1 = 0x11
     * +/-250 dps, DLPF enabled.
     */
    status = WriteRegister(
            dev,
            ICM20948_B2_GYRO_CONFIG_1,
            0x11);

    if (status != HAL_OK) return status;

    HAL_Delay(20);

    return SelectBank(dev, ICM20948_BANK_0);
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_GYRO_ReadRaw(
        ICM20948_GYRO_t *dev,
        int16_t *gx,
        int16_t *gy,
        int16_t *gz)
{
    uint8_t data[6];

    if ((dev == NULL) ||
        (gx == NULL) ||
        (gy == NULL) ||
        (gz == NULL))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
            SelectBank(dev, ICM20948_BANK_0);

    if (status != HAL_OK)
    {
        return status;
    }

    status = ReadRegisters(
            dev,
            ICM20948_REG_GYRO_XOUT_H,
            data,
            6);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * ICM-20948 accel/gyro registers are big-endian:
     * high byte first, then low byte.
     */
    *gx = (int16_t)(
            ((uint16_t)data[0] << 8) |
            data[1]);

    *gy = (int16_t)(
            ((uint16_t)data[2] << 8) |
            data[3]);

    *gz = (int16_t)(
            ((uint16_t)data[4] << 8) |
            data[5]);

    return HAL_OK;
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_GYRO_CalibrateStationary(
        ICM20948_GYRO_t *dev,
        ICM20948_GYRO_Calibration_t *cal,
        uint16_t sample_count,
        uint32_t sample_delay_ms)
{
    if ((dev == NULL) ||
        (cal == NULL) ||
        (sample_count == 0))
    {
        return HAL_ERROR;
    }

    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    uint16_t valid_samples = 0;

    cal->bias_x_raw = 0.0f;
    cal->bias_y_raw = 0.0f;
    cal->bias_z_raw = 0.0f;
    cal->sample_count = 0;
    cal->valid = 0;

    while (valid_samples < sample_count)
    {
        int16_t gx;
        int16_t gy;
        int16_t gz;

        if (ICM20948_GYRO_ReadRaw(
                dev,
                &gx,
                &gy,
                &gz) == HAL_OK)
        {
            sum_x += gx;
            sum_y += gy;
            sum_z += gz;

            valid_samples++;
        }

        HAL_Delay(sample_delay_ms);
    }

    cal->bias_x_raw =
            (float)sum_x / (float)valid_samples;

    cal->bias_y_raw =
            (float)sum_y / (float)valid_samples;

    cal->bias_z_raw =
            (float)sum_z / (float)valid_samples;

    cal->sample_count = valid_samples;
    cal->valid = 1;

    return HAL_OK;
}

/* ------------------------------------------------------- */

HAL_StatusTypeDef ICM20948_GYRO_ReadCalibrated(
        ICM20948_GYRO_t *dev,
        const ICM20948_GYRO_Calibration_t *cal,
        ICM20948_GYRO_Data_t *data)
{
    if ((dev == NULL) ||
        (cal == NULL) ||
        (data == NULL) ||
        (!cal->valid))
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
            ICM20948_GYRO_ReadRaw(
                    dev,
                    &data->x_raw,
                    &data->y_raw,
                    &data->z_raw);

    if (status != HAL_OK)
    {
        return status;
    }

    float x_corrected_raw =
            (float)data->x_raw -
            cal->bias_x_raw;

    float y_corrected_raw =
            (float)data->y_raw -
            cal->bias_y_raw;

    float z_corrected_raw =
            (float)data->z_raw -
            cal->bias_z_raw;

    data->x_dps =
            x_corrected_raw /
            ICM20948_GYRO_LSB_PER_DPS;

    data->y_dps =
            y_corrected_raw /
            ICM20948_GYRO_LSB_PER_DPS;

    data->z_dps =
            z_corrected_raw /
            ICM20948_GYRO_LSB_PER_DPS;

    data->x_rad_s =
            data->x_dps * DEG_TO_RAD;

    data->y_rad_s =
            data->y_dps * DEG_TO_RAD;

    data->z_rad_s =
            data->z_dps * DEG_TO_RAD;

    return HAL_OK;
}
