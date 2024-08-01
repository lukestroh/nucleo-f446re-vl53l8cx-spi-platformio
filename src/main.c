/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l8cx_api.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t status;
uint8_t resolution_, is_alive;
volatile int int_count;
uint8_t p_data_ready;

VL53L8CX_Configuration dev_;
VL53L8CX_ResultsData results_;

uint16_t idx;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==INT_C_Pin)
	{
		++int_count;
	}
}

void get_data_by_polling(VL53L8CX_Configuration *p_dev){
  status = vl53l8cx_check_data_ready(&dev_, &p_data_ready);
  if(p_data_ready){
    status = vl53l8cx_get_resolution(p_dev, &resolution_);
    status = vl53l8cx_get_ranging_data(p_dev, &results_);

    for(int i = 0; i < resolution_; ++i){
    	/* Print per zone results */
    	// printf("Zone : %2d, Nb targets : %2u, Ambient : %4lu Kcps/spads, ",
    	// 		i,
    	// 		results_.nb_target_detected[i],
    	// 		results_.ambient_per_spad[i]);

    	/* Print per target results */
    	if(results_.nb_target_detected[i] > 0){
    		// printf("Target status : %3u, Distance : %4d mm\n",
    				// results_.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
    				// results_.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
    	} else {
    		// printf("Target status : 255, Distance : No target\n");
    	}
    }
    printf("\n");
  }
  else{
    HAL_Delay(5);
  }
}

void get_data_by_interrupt(VL53L8CX_Configuration *p_dev) {
  __WFI(); // wait for interrupt
  if (int_count != 0) {
    int_count = 0;
    status = vl53l8cx_get_resolution(p_dev, &resolution_);
    status = vl53l8cx_get_ranging_data(p_dev, &results_);
    
    for (uint8_t i=0; i<resolution_; ++i) {
      // print per-zone results
      printf("Zone: %2d, Nb targets: %2u, Ambient: %4lu Kcps/spads, ",
              i,
              results_.nb_target_detected[i],
              0); // results_.ambient_per_spad[i]);
    
      // print per-target results
      if (results_.nb_target_detected[i] > 0) {
        printf("Target status: %3u, Distance: %4d mm\n",
                results_.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
                results_.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
      }
      else {
        printf("Target status : 255, Distance : No target\n");
      }
    }
    printf("\n");
  }
}

void wait_for_vl53l8cx(VL53L8CX_Configuration* p_dev, uint8_t* p_is_alive, uint8_t* send_msg_buf) {
    // Check if sensor attached TODO: move to function.
  while (1) {
    status = vl53l8cx_is_alive(p_dev, p_is_alive);
    if (!(*p_is_alive)) {
      memset(send_msg_buf, 0, sizeof(send_msg_buf));
      sprintf(send_msg_buf, "is_alive: %d\r\n", *p_is_alive);
      HAL_UART_Transmit(&huart2, send_msg_buf, sizeof(send_msg_buf), 100);
      HAL_Delay(1000);
    }
    else {
      memset(send_msg_buf, 0, sizeof(send_msg_buf));
      sprintf(send_msg_buf, "is_alive: %d\r\n", *p_is_alive);
      HAL_UART_Transmit(&huart2, send_msg_buf, sizeof(send_msg_buf), 100);
      break;
    } 
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  // Reset vl53l8cx
  VL53L8CX_Reset_Sensor(&(dev_.platform));

  // UART string buffers
  uint8_t setup_msg[48] = {'\0'};
  uint8_t data_msg[128] = {'\0'};

  sprintf(setup_msg, "Program begin.\r\n");
  HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);

  // Check if sensor attached TODO: move to function.
  // wait_for_vl53l8cx(&dev_, &is_alive, setup_msg);
  while (1) {
    status = vl53l8cx_is_alive(&dev_, &is_alive);
    if (!is_alive) {
      HAL_Delay(1000);
    }
    else {
      break;
    } 
  }
  memset(setup_msg, 0, sizeof(setup_msg));
  sprintf(setup_msg, "is_alive: %d\r\n", is_alive);
  HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);

  // Initialize the sensor. The function copies the firmware (~84 Kbytes) to the module. This is done by loading the code over the I²C/SPI interface, and performing a boot routine to complete the initialization
  status = vl53l8cx_init(&dev_);
  if (status != VL53L8CX_STATUS_OK) {
    memset(setup_msg, 0, sizeof(setup_msg));
    sprintf(setup_msg, "init failed with status %d\r\n", status);
    HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);
    return 255;
  }

  status = vl53l8cx_set_ranging_mode(&dev_, VL53L8CX_RANGING_MODE_CONTINUOUS);
   if (status != VL53L8CX_STATUS_OK) {
    memset(setup_msg, 0, sizeof(setup_msg));
    sprintf(setup_msg, "set_ranging_mode failed with status %d\r\n", status);
    HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);
    return 255;
  }
  status = vl53l8cx_set_resolution(&dev_, VL53L8CX_RESOLUTION_4X4); // Set the zone resolution of the sensor.
   if (status != VL53L8CX_STATUS_OK) {
    memset(setup_msg, 0, sizeof(setup_msg));
    sprintf(setup_msg, "set_resolution failed with status %d\r\n", status);
    HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);
    return 255;
  }
  status = vl53l8cx_set_ranging_frequency_hz(&dev_, 1);
   if (status != VL53L8CX_STATUS_OK) {
    memset(setup_msg, 0, sizeof(setup_msg));
    sprintf(setup_msg, "set_ranging_frequency failed with status %d\r\n", status);
    HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);
    return 255;
  }
  
  status = vl53l8cx_set_power_mode(&dev_, VL53L8CX_POWER_MODE_WAKEUP);
  if (status != VL53L8CX_STATUS_OK) {
    memset(setup_msg, 0, sizeof(setup_msg));
    sprintf(setup_msg, "set_power_mode failed with status %d", status);
    HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);
    return 255;
  }

  // Start ranging
  status = vl53l8cx_start_ranging(&dev_);
  if (status != VL53L8CX_STATUS_OK) {
    memset(setup_msg, 0, sizeof(setup_msg));
    sprintf(setup_msg, "start_ranging failed with status %d, data_size: %d\r\n", status, (int)dev_.data_read_size);
    HAL_UART_Transmit(&huart2, setup_msg, sizeof(setup_msg), 100);
    return 255;
  }

  // Main loop parameters
  uint32_t tickstart = HAL_GetTick();
  uint32_t led_delay = 1000;
  // Main loop
  while (1) {
    get_data_by_polling(&dev_);

    // LED Control
    if (HAL_GetTick() - tickstart > led_delay) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      tickstart = HAL_GetTick();

      for (uint8_t i=0; i<16; ++i) {
        memset(data_msg, 0, sizeof(data_msg));
        sprintf(data_msg, "Zone : %2d, Nb targets : %2u, Ambient : %4lu Kcps/spads,\r\n",
                          i,
                          results_.nb_target_detected[i],
                          0);// results_.ambient_per_spad[i]);
        HAL_UART_Transmit(&huart2, data_msg, sizeof(data_msg), 100);
      }
    }
    HAL_Delay(100);
  }

}

/**
  * @brief System Clock Configuration
  * @param None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  /* Change SPI baudrate using prescaler:
	 * Default clock is 84MHz
	 */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PWR_EN_C_Pin|LPn_C_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NCS_C_GPIO_Port, NCS_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_C_Pin */
  GPIO_InitStruct.Pin = INT_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_C_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_EN_C_Pin LPn_C_Pin */
  GPIO_InitStruct.Pin = PWR_EN_C_Pin|LPn_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NCS_C_Pin */
  GPIO_InitStruct.Pin = NCS_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NCS_C_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); // This one is from the demo, not needed??
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
