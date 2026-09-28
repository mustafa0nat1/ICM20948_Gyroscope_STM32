/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ICM-20948 Gyroscope Calibration - Termite
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "icm20948_gyro_cal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

ICM20948_GYRO_t gyro;
ICM20948_GYRO_Calibration_t gyroCal;
ICM20948_GYRO_Data_t gyroData;

char uartBuffer[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void UART_Send(const char *text);

static void FormatFloat3(
        char *buffer,
        size_t buffer_size,
        float value);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(
            &huart2,
            (uint8_t *)text,
            strlen(text),
            HAL_MAX_DELAY);
}

static void FormatFloat3(
        char *buffer,
        size_t buffer_size,
        float value)
{
    int32_t scaled;

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * 1000.0f + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * 1000.0f - 0.5f);
    }

    int32_t whole =
            scaled / 1000;

    int32_t fraction =
            scaled % 1000;

    if (fraction < 0)
    {
        fraction = -fraction;
    }

    if ((scaled < 0) && (whole == 0))
    {
        snprintf(
                buffer,
                buffer_size,
                "-0.%03ld",
                (long)fraction);
    }
    else
    {
        snprintf(
                buffer,
                buffer_size,
                "%ld.%03ld",
                (long)whole,
                (long)fraction);
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();

    /* USER CODE BEGIN 2 */

    UART_Send("\r\n");
    UART_Send("============================================\r\n");
    UART_Send("ICM-20948 GYROSCOPE CALIBRATION\r\n");
    UART_Send("============================================\r\n");

    if (ICM20948_GYRO_Init(
            &gyro,
            &hspi1,
            GPIOA,
            GPIO_PIN_4) != HAL_OK)
    {
        snprintf(
                uartBuffer,
                sizeof(uartBuffer),
                "GYRO INIT ERROR | WHO_AM_I:0x%02X\r\n",
                gyro.who_am_i);

        UART_Send(uartBuffer);

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    snprintf(
            uartBuffer,
            sizeof(uartBuffer),
            "ICM-20948 FOUND : 0x%02X\r\n\r\n",
            gyro.who_am_i);

    UART_Send(uartBuffer);

    UART_Send("KEEP THE SENSOR COMPLETELY STILL\r\n");
    UART_Send("GYRO CALIBRATION STARTS IN 5 SECONDS\r\n\r\n");

    for (int i = 5; i >= 1; i--)
    {
        snprintf(
                uartBuffer,
                sizeof(uartBuffer),
                "Starting in %d...\r\n",
                i);

        UART_Send(uartBuffer);

        HAL_Delay(1000);
    }

    UART_Send("\r\n");
    UART_Send("========== GYRO CALIBRATION START ==========\r\n");
    UART_Send("DO NOT MOVE THE SENSOR\r\n");

    /*
     * 500 samples x 10 ms ~= 5 seconds.
     * Gyro output rate is configured around 100 Hz.
     */
    if (ICM20948_GYRO_CalibrateStationary(
            &gyro,
            &gyroCal,
            500,
            10) != HAL_OK)
    {
        UART_Send("GYRO CALIBRATION ERROR\r\n");

        while (1)
        {
            HAL_Delay(1000);
        }
    }

    UART_Send("=========== GYRO CALIBRATION END ===========\r\n\r\n");

    char bx[24], by[24], bz[24];
    char bxdps[24], bydps[24], bzdps[24];

    FormatFloat3(
            bx,
            sizeof(bx),
            gyroCal.bias_x_raw);

    FormatFloat3(
            by,
            sizeof(by),
            gyroCal.bias_y_raw);

    FormatFloat3(
            bz,
            sizeof(bz),
            gyroCal.bias_z_raw);

    FormatFloat3(
            bxdps,
            sizeof(bxdps),
            gyroCal.bias_x_raw / 131.0f);

    FormatFloat3(
            bydps,
            sizeof(bydps),
            gyroCal.bias_y_raw / 131.0f);

    FormatFloat3(
            bzdps,
            sizeof(bzdps),
            gyroCal.bias_z_raw / 131.0f);

    UART_Send("GYRO CALIBRATION SUCCESS\r\n\r\n");

    snprintf(
            uartBuffer,
            sizeof(uartBuffer),
            "BIAS RAW [LSB]\r\n"
            "X:%s Y:%s Z:%s\r\n\r\n"
            "BIAS [deg/s]\r\n"
            "X:%s Y:%s Z:%s\r\n\r\n"
            "SAMPLES:%lu\r\n\r\n",
            bx,
            by,
            bz,
            bxdps,
            bydps,
            bzdps,
            (unsigned long)gyroCal.sample_count);

    UART_Send(uartBuffer);

    UART_Send("NORMAL MODE STARTED\r\n");
    UART_Send("Keep still first, then rotate around X/Y/Z axes.\r\n\r\n");

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (ICM20948_GYRO_ReadCalibrated(
                &gyro,
                &gyroCal,
                &gyroData) == HAL_OK)
        {
            char xdps[24], ydps[24], zdps[24];
            char xrad[24], yrad[24], zrad[24];

            FormatFloat3(
                    xdps,
                    sizeof(xdps),
                    gyroData.x_dps);

            FormatFloat3(
                    ydps,
                    sizeof(ydps),
                    gyroData.y_dps);

            FormatFloat3(
                    zdps,
                    sizeof(zdps),
                    gyroData.z_dps);

            FormatFloat3(
                    xrad,
                    sizeof(xrad),
                    gyroData.x_rad_s);

            FormatFloat3(
                    yrad,
                    sizeof(yrad),
                    gyroData.y_rad_s);

            FormatFloat3(
                    zrad,
                    sizeof(zrad),
                    gyroData.z_rad_s);

            snprintf(
                    uartBuffer,
                    sizeof(uartBuffer),

                    "RAW X:%6d Y:%6d Z:%6d | "
                    "DPS X:%s Y:%s Z:%s | "
                    "RAD/S X:%s Y:%s Z:%s\r\n",

                    gyroData.x_raw,
                    gyroData.y_raw,
                    gyroData.z_raw,

                    xdps,
                    ydps,
                    zdps,

                    xrad,
                    yrad,
                    zrad);

            UART_Send(uartBuffer);
        }
        else
        {
            UART_Send("GYRO READ ERROR\r\n");
        }

        HAL_Delay(100);
    }

    /* USER CODE END WHILE */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
            RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
            RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
            RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
            RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
            RCC_PLLSOURCE_HSI;

    RCC_OscInitStruct.PLL.PLLMUL =
            RCC_PLL_MUL4;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
            RCC_CLOCKTYPE_HCLK |
            RCC_CLOCKTYPE_SYSCLK |
            RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
            RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
            RCC_SYSCLK_DIV2;

    RCC_ClkInitStruct.APB1CLKDivider =
            RCC_HCLK_DIV2;

    RCC_ClkInitStruct.APB2CLKDivider =
            RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief SPI1 Initialization Function
  * @retval None
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode =
            SPI_MODE_MASTER;

    hspi1.Init.Direction =
            SPI_DIRECTION_2LINES;

    hspi1.Init.DataSize =
            SPI_DATASIZE_8BIT;

    hspi1.Init.CLKPolarity =
            SPI_POLARITY_LOW;

    hspi1.Init.CLKPhase =
            SPI_PHASE_1EDGE;

    hspi1.Init.NSS =
            SPI_NSS_SOFT;

    hspi1.Init.BaudRatePrescaler =
            SPI_BAUDRATEPRESCALER_8;

    hspi1.Init.FirstBit =
            SPI_FIRSTBIT_MSB;

    hspi1.Init.TIMode =
            SPI_TIMODE_DISABLE;

    hspi1.Init.CRCCalculation =
            SPI_CRCCALCULATION_DISABLE;

    hspi1.Init.CRCPolynomial = 7;

    hspi1.Init.CRCLength =
            SPI_CRC_LENGTH_DATASIZE;

    hspi1.Init.NSSPMode =
            SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate =
            115200;

    huart2.Init.WordLength =
            UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
            UART_STOPBITS_1;

    huart2.Init.Parity =
            UART_PARITY_NONE;

    huart2.Init.Mode =
            UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
            UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
            UART_OVERSAMPLING_16;

    huart2.Init.OneBitSampling =
            UART_ONE_BIT_SAMPLE_DISABLE;

    huart2.AdvancedInit.AdvFeatureInit =
            UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}


/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_WritePin(
            GPIOA,
            GPIO_PIN_4,
            GPIO_PIN_SET);

    GPIO_InitStruct.Pin =
            GPIO_PIN_4;

    GPIO_InitStruct.Mode =
            GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
            GPIO_NOPULL;

    GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct);
}


void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT
void assert_failed(
        uint8_t *file,
        uint32_t line)
{
}
#endif
