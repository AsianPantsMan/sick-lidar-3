/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "pid.h"
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_RESOLUTION 5264

#define MCU_VOLTAGE 3.3f    // System Voltage
#define DAC_MAX_VAL 4095.0f // 12-bit resolution

// RoboClaw Logic Levels
#define RC_VOLT_MIN 0.0f
#define RC_VOLT_STOP 1.0f
#define RC_VOLT_MAX 2.0f

#define DAC_VAL_STOP (uint32_t)((RC_VOLT_STOP / MCU_VOLTAGE) * DAC_MAX_VAL)
#define DAC_VAL_MAX (uint32_t)((RC_VOLT_MAX / MCU_VOLTAGE) * DAC_MAX_VAL)
#define DAC_VAL_MIN 0

#define VARYING_LIMIT 1241

// BMI270 I2C Address
#define BMI270_ADDR (0x68 << 1)

// BMI270 Registers
#define BMI2_CHIP_ID        0x00
#define BMI2_ACC_X_LSB      0x0C
#define BMI2_GYR_X_LSB      0x12
#define BMI2_ACC_CONF       0x40
#define BMI2_GYR_CONF       0x42
#define BMI2_PWR_CONF       0x7C
#define BMI2_PWR_CTRL       0x7D
#define BMI2_CMD            0x7E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_Config pid_config;
PID_State pid_state;
uint32_t last_time = 0;
int32_t last_encoder_count = 0;
float target_speed = 1000.0f;

// DAC switching variables
static uint32_t last_switch_time = 0;
static uint8_t current_state = 0;
#define SWITCH_INTERVAL_MS 10000

#define DAC_1000_RPM_CW (DAC_VAL_STOP - 158)
#define DAC_2000_RPM_CCW (DAC_VAL_STOP + 101)

char *direction = "N/A";

// UART Receive from Raspberry Pi
#define UART_RX_BUFFER_SIZE 100
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
char received_message[UART_RX_BUFFER_SIZE];
volatile uint8_t message_received = 0;
char message_to_repeat[UART_RX_BUFFER_SIZE] = "No message received yet.\r\n";

uint8_t rx_byte;
uint16_t rx_index = 0;

// BMI270 sensor data
int16_t gyro_raw[3];
int16_t accel_raw[3];
float gyro_deg[3];   // deg/s
float accel_ms2[3];  // m/s²

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// BMI270 functions
void bmi270_write_reg(uint8_t reg, uint8_t value);
uint8_t bmi270_read_reg(uint8_t reg);
void bmi270_init(void);
void bmi270_read_gyro(void);
void bmi270_read_accel(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Printf redirect to UART
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
  return len;
}

HAL_StatusTypeDef UART_LoopbackTest(UART_HandleTypeDef *huart)
{
    uint8_t txByte = 'A';
    uint8_t rxByte = 0;

    HAL_UART_Transmit(huart, &txByte, 1, 100);
    HAL_Delay(10);

    if (HAL_UART_Receive(huart, &rxByte, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (rxByte == txByte)
    {
        return HAL_OK;
    }

    return HAL_ERROR;
}

/**
  * @brief  Write single byte to BMI270 register
  */
void bmi270_write_reg(uint8_t reg, uint8_t value)
{
  HAL_I2C_Mem_Write(&hi2c1, BMI270_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
}

/**
  * @brief  Read single byte from BMI270 register
  */
uint8_t bmi270_read_reg(uint8_t reg)
{
  uint8_t data;
  HAL_I2C_Mem_Read(&hi2c1, BMI270_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
  return data;
}

/**
  * @brief  Initialize BMI270 sensor
  */
void bmi270_init(void)
{
  // Soft reset
  bmi270_write_reg(BMI2_CMD, 0xB6);
  HAL_Delay(200);

  // Disable advanced power save mode
  bmi270_write_reg(BMI2_PWR_CONF, 0x00);
  HAL_Delay(1);

  // Enable accelerometer and gyroscope
  bmi270_write_reg(BMI2_PWR_CTRL, 0x0E);
  HAL_Delay(50);

  // Configure accelerometer: ODR=100Hz, Normal mode
  bmi270_write_reg(BMI2_ACC_CONF, 0xA8);

  // Configure gyroscope: ODR=200Hz, Normal mode
  bmi270_write_reg(BMI2_GYR_CONF, 0xA9);

  HAL_Delay(10);
}

/**
  * @brief  Read gyroscope data
  */
void bmi270_read_gyro(void)
{
  uint8_t data[6];

  // Read 6 bytes starting from gyro X LSB register
  HAL_I2C_Mem_Read(&hi2c1, BMI270_ADDR, BMI2_GYR_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 1000);

  // Combine bytes (little-endian)
  gyro_raw[0] = (int16_t)(data[0] | (data[1] << 8));  // X
  gyro_raw[1] = (int16_t)(data[2] | (data[3] << 8));  // Y
  gyro_raw[2] = (int16_t)(data[4] | (data[5] << 8));  // Z
}

/**
  * @brief  Read accelerometer data
  */
void bmi270_read_accel(void)
{
  uint8_t data[6];

  // Read 6 bytes starting from accel X LSB register
  HAL_I2C_Mem_Read(&hi2c1, BMI270_ADDR, BMI2_ACC_X_LSB, I2C_MEMADD_SIZE_8BIT, data, 6, 1000);

  // Combine bytes (little-endian)
  accel_raw[0] = (int16_t)(data[0] | (data[1] << 8));  // X
  accel_raw[1] = (int16_t)(data[2] | (data[3] << 8));  // Y
  accel_raw[2] = (int16_t)(data[4] | (data[5] << 8));  // Z
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
  MX_I2C1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(100);  // Wait for sensor power-up

  // Check BMI270 chip ID
  uint8_t chip_id = bmi270_read_reg(BMI2_CHIP_ID);

  if(chip_id == 0x24)
  {
    // BMI270 detected
    printf("BMI270 detected (Chip ID: 0x%02X)\r\n", chip_id);
    bmi270_init();
    HAL_Delay(100);
    printf("BMI270 initialized successfully\r\n");
  }
  else
  {
    // Sensor not found
    printf("ERROR: BMI270 not found! Chip ID: 0x%02X (expected 0x24)\r\n", chip_id);
    Error_Handler();
  }

  // Motor DAC initialization (currently commented out)
  // HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  // HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_VAL_STOP);

  // Motor encoder setup (currently commented out)
  // HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // PID Initialization (currently commented out)
  // ... your PID code ...

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // UART Loopback Test
    if (UART_LoopbackTest(&huart1) == HAL_OK)
    {
      // SUCCESS - toggle green LED
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }
    else
    {
      // FAILED - blink error LED
      while(1)
      {
        printf("UART Loopback Error\r\n");
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(50);
      }
    }

    // Read BMI270 sensors
    bmi270_read_gyro();
    bmi270_read_accel();

    // Convert gyro to deg/s (±2000 deg/s range, 16.4 LSB/deg/s)
    gyro_deg[0] = gyro_raw[0] / 16.4f;
    gyro_deg[1] = gyro_raw[1] / 16.4f;
    gyro_deg[2] = gyro_raw[2] / 16.4f;

    // Convert accel to m/s² (±4g range, 8192 LSB/g)
    accel_ms2[0] = (accel_raw[0] / 8192.0f) * 9.81f;
    accel_ms2[1] = (accel_raw[1] / 8192.0f) * 9.81f;
    accel_ms2[2] = (accel_raw[2] / 8192.0f) * 9.81f;

    // Print BMI270 data
    printf("=== BMI270 Data ===\r\n");
    printf("Gyro (deg/s): X:%.2f Y:%.2f Z:%.2f\r\n",
           gyro_deg[0], gyro_deg[1], gyro_deg[2]);
    printf("Accel (m/s²): X:%.2f Y:%.2f Z:%.2f\r\n",
           accel_ms2[0], accel_ms2[1], accel_ms2[2]);
    printf("Raw Gyro: X:%d Y:%d Z:%d\r\n",
           gyro_raw[0], gyro_raw[1], gyro_raw[2]);
    printf("Raw Accel: X:%d Y:%d Z:%d\r\n",
           accel_raw[0], accel_raw[1], accel_raw[2]);
    printf("\r\n");

    // Your existing motor/PID code (commented out)
    // ... motor control code here ...

    // Raspberry Pi message handling
    if (message_received) {
      strncpy(message_to_repeat, received_message, UART_RX_BUFFER_SIZE);
      message_to_repeat[UART_RX_BUFFER_SIZE - 1] = '\0';
      message_received = 0;
    }

    // printf("=== LAST RECEIVED MESSAGE ===\r\n");
    // printf("Repeated: %s\r\n", message_to_repeat);
    // printf("=============================\r\n");

    HAL_Delay(1000);  // 1 Hz update rate
  }
  /* USER CODE END 3 */
}

/* ... Rest of your existing code (SystemClock_Config, HAL_UART_RxCpltCallback, Error_Handler, etc.) ... */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    if (rx_index >= UART_RX_BUFFER_SIZE - 1) {
      rx_index = 0;
    }

    if (rx_byte == '\n' || rx_byte == '\r') {
      uart_rx_buffer[rx_index] = '\0';
      strcpy(received_message, (char *)uart_rx_buffer);
      message_received = 1;
      rx_index = 0;
    } else {
      uart_rx_buffer[rx_index++] = rx_byte;
    }

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("!!! HAL Error Occurred !!!\r\n");
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
