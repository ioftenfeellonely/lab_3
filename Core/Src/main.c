/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (MPU-6500, I2C2, Variant #9)
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6500_ADDR_LOW    (0x68u << 1)   // AD0 = GND
#define MPU6500_ADDR_HIGH   (0x69u << 1)   // AD0 = VDDIO

// Реєстри MPU-6500
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_WHO_AM_I        0x75
#define REG_PWR_MGMT_1      0x6B
#define REG_GYRO_XOUT_H     0x43  // XH,XL,YH,YL,ZH,ZL

// Масштаб гіроскопа для ±500 dps
#define GYRO_SENS_500DPS    (65.5f)        // LSB/(°/s)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
static uint16_t i2c_timeout = 100;  // мс
static uint8_t  mpu_addr = 0;       // поточна I2C-адреса сенсора

volatile uint8_t  who_am_i = 0;

volatile int16_t  gyro_x = 0, gyro_y = 0, gyro_z = 0;                // LSB
volatile float    gyro_dps_x = 0.0f, gyro_dps_y = 0.0f, gyro_dps_z = 0.0f;  // град/с
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef mpu6500_write(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef mpu6500_read(uint8_t reg, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef mpu6500_detect(void);
static HAL_StatusTypeDef mpu6500_init_variant9(void);

// SWO/ITM вивід (Stimulus Port 0)
static inline int  ITM_SendChar_(int ch);
static inline void swo_print(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Низькорівневі I2C доступи
static HAL_StatusTypeDef mpu6500_write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c2, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, i2c_timeout);
}

static HAL_StatusTypeDef mpu6500_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c2, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT,
                          buf, len, i2c_timeout);
}

/**
 * Підключення та WHO_AM_I:
 * пробує 0x68 (AD0=0), якщо ні - 0x69 (AD0=1)
 * читає REG_WHO_AM_I і зберігає у who_am_i
 */
static HAL_StatusTypeDef mpu6500_detect(void)
{
  uint8_t id = 0;

  mpu_addr = MPU6500_ADDR_LOW; // 0x68<<1
  if (HAL_I2C_IsDeviceReady(&hi2c2, mpu_addr, 2, i2c_timeout) == HAL_OK)
  {
    if (mpu6500_read(REG_WHO_AM_I, &id, 1) == HAL_OK)
    {
      who_am_i = id;
      return HAL_OK;
    }
  }

  mpu_addr = MPU6500_ADDR_HIGH; // 0x69<<1
  if (HAL_I2C_IsDeviceReady(&hi2c2, mpu_addr, 2, i2c_timeout) == HAL_OK)
  {
    if (mpu6500_read(REG_WHO_AM_I, &id, 1) == HAL_OK)
    {
      who_am_i = id;
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}

/**
 * За варіантом №9
 * Розбудити: PWR_MGMT_1 = 0x01 (CLKSEL=PLL, SLEEP=0)
 * Гіроскоп ±500 dps, ODR 416 Гц
 */
static HAL_StatusTypeDef mpu6500_init_variant9(void)
{
  HAL_StatusTypeDef st;
  uint8_t v;

  // Вибрати PLL як тактове, sleep=0
  st = mpu6500_write(REG_PWR_MGMT_1, 0x01);
  if (st != HAL_OK) return st;
  HAL_Delay(10);

  // CONFIG: DLPF_CFG=0
  st = mpu6500_write(REG_CONFIG, 0x00);
  if (st != HAL_OK) return st;

  // GYRO_CONFIG: FS_SEL=1 (±500 dps), FCHOICE_B=3 (байпас DLPF)
  if ((st = mpu6500_read(REG_GYRO_CONFIG, &v, 1)) != HAL_OK) return st;
  v &= (uint8_t)~0x18u;
  v |= (uint8_t)(1u << 3);
  v &= (uint8_t)~0x03u;
  v |= (uint8_t)0x03u;
  if ((st = mpu6500_write(REG_GYRO_CONFIG, v)) != HAL_OK) return st;

  // 8 кГц в 421 Гц
  st = mpu6500_write(REG_SMPLRT_DIV, 18);
  if (st != HAL_OK) return st;

  return HAL_OK;
}


static inline int ITM_SendChar_(int ch)
{
  volatile uint32_t *ITM_STIM0 = (uint32_t*)0xE0000000;
  volatile uint32_t *ITM_TCR   = (uint32_t*)0xE0000E80;
  volatile uint32_t *ITM_ENA   = (uint32_t*)0xE0000E00;
  if ((*ITM_TCR & 1) == 0) return -1;
  if ((*ITM_ENA & 1) == 0) return -1;
  while ((*ITM_STIM0 & 1) == 0) { __NOP(); }
  *(volatile char*)ITM_STIM0 = (char)ch;
  return ch;
}

static inline void swo_print(const char *s)
{
  while (*s) ITM_SendChar_(*s++);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  if (mpu6500_detect() != HAL_OK)
  {
    swo_print("MPU6500 not found!\n");
    Error_Handler();
  }
  else
  {
    char line[48];
    snprintf(line, sizeof(line), "WHO_AM_I=0x%02X\n", (unsigned int)who_am_i);
    swo_print(line);
  }

  if (mpu6500_init_variant9() != HAL_OK)
  {
    swo_print("MPU6500 init failed!\n");
    Error_Handler();
  }
  else
  {
    swo_print("MPU6500 init OK (±500 dps, ~421 Hz)\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Читання гіроскопа і масштабування
    uint8_t data[6];
    if (mpu6500_read(REG_GYRO_XOUT_H, data, sizeof(data)) == HAL_OK)
    {
      gyro_x = (int16_t)((data[0] << 8) | data[1]);
      gyro_y = (int16_t)((data[2] << 8) | data[3]);
      gyro_z = (int16_t)((data[4] << 8) | data[5]);

      gyro_dps_x = (float)gyro_x / GYRO_SENS_500DPS;
      gyro_dps_y = (float)gyro_y / GYRO_SENS_500DPS;
      gyro_dps_z = (float)gyro_z / GYRO_SENS_500DPS;
    }

    static uint32_t t0 = 0;
    if (HAL_GetTick() - t0 > 200)
    {
      t0 = HAL_GetTick();
      char buf[120];
      int n = snprintf(buf, sizeof(buf),
                       "gx=%6d gy=%6d gz=%6d  dps=[%7.2f %7.2f %7.2f]\n",
                       gyro_x, gyro_y, gyro_z,
                       (double)gyro_dps_x, (double)gyro_dps_y, (double)gyro_dps_z);
      if (n > 0) swo_print(buf);
    }

    HAL_Delay(10);
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;  // <-- виправлено (було HSI_ON)
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
