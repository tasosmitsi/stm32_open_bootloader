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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef void (*pFunction)(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define FLASH_APP_ADDR     0x8008000

#define BOOT_SIGNATURE      0xB00720AD  // "BOOTLOAD" signature in RAM
#define APP_SIGNATURE      0xB00720AA  // "APP" signature in RAM
#define RAM_SIGNATURE_ADDR  ((volatile uint32_t*)0x2001FFF0)  // Adjust to your system's RAM base address
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void go2APP(void);
void* Boot_GetApplicationAddress(void);
int is_application_valid(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void* Boot_GetApplicationAddress(void) {
    // Check for bootloader signature in non-initialized RAM
    if (*RAM_SIGNATURE_ADDR != APP_SIGNATURE) {
        // Do not clear the signature here, as it might need to restart the bootloader
        return NULL;  // Bootloader should start
    }

    // Check application integrity (simple check for now, you can expand this)
    if (is_application_valid()) {
        return (void*)FLASH_APP_ADDR;  // Return application base address if valid
    }

    return NULL;  // Application is not valid, start bootloader
}

int is_application_valid(void) {
    // You can implement an actual CRC or checksum validation here
    // For simplicity, assume application is valid if vector table's first entry is a valid stack pointer
    uint32_t sp = *(volatile uint32_t*)FLASH_APP_ADDR;
    return 1;
    return (sp >= 0x20000000 && sp <= 0x20010000);  // Check if SP is within valid RAM range
    //  if(((*(uint32_t*) FLASH_APP_ADDR) & 0x2FFE0000) == 0x20020000)
    //  {
    //	  printf("APP starts...\r\n");
    //  } else
    //  {
    //    printf("No APP founddddd.\r\n");
    //  }
}

void go2APP(void)
{
	uint32_t JumpAddress;
	pFunction Jump_To_Application;

	//jump to the application
	JumpAddress = *(uint32_t *) (FLASH_APP_ADDR + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	//init application's stack pointer
	__set_MSP(*(uint32_t *)FLASH_APP_ADDR);

	__DSB();  // Data Synchronization Barrier
	__ISB();  // Instruction Synchronization Barrier

	Jump_To_Application();
}

int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx=0; DataIdx < len; DataIdx++)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *) ptr++, 1, 100);
	}
	return len;
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("Bootloader start\r\n");

  printf("Reseting... \r\n");
  HAL_Delay(5000);
  *RAM_SIGNATURE_ADDR = APP_SIGNATURE;  // Clear the bootloader signature in RAM

  NVIC_SystemReset();       // Trigger a system reset to start the application
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
