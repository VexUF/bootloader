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
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILENAME "vexuf_fw.bin"
#define APPLICATION_ADDRESS 0x08008000 // Address after 32KB (0x8000)
#define BUFFER_SIZE 2048 // 2KB buffer
#define START_SECTOR FLASH_SECTOR_2 // use FLASH_SECTOR_2 for release and FLASH_SECTOR_3 for debug
#define NR_OF_SECTORS 4 // End sector is 5. This is the number of sectors to erase
#define MAX_FILE_SIZE 229376 //224KBs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Enum for pin types
typedef enum {
    PIN_ERROR,
    PIN_WARN,
    PIN_INFO
} PinType;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int isBoot1High(void);
int flashFirmwareInChunks(FIL *file, uint32_t fileSize);
void jumpToApplication(void);
void toggleIndicator(PinType pinType, int delay, int times);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int isBoot1High(void) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET;
}

void toggleIndicator(PinType pinType, int delay, int times) {
    uint16_t pin;
    GPIO_TypeDef *port = GPIOC;

    // Determine which pin to toggle based on pinType
    switch (pinType) {
        case PIN_ERROR:
            pin = Error_Pin;
            break;
        case PIN_WARN:
            pin = Warn_Pin;
            break;
        case PIN_INFO:
            pin = Info_Pin;
            break;
        default:
            return; // Invalid pinType, exit function
    }

    for (int i = 0; i < times * 2; i++) {
        HAL_GPIO_TogglePin(port, pin);
        HAL_Delay(delay);
    }
    HAL_Delay(delay * 4);

}



int flashFirmwareInChunks(FIL *file, uint32_t fileSize) {
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;
	uint8_t buffer[BUFFER_SIZE];
	uint32_t bytesRead = 0;
	UINT readBytes;
	uint32_t address = APPLICATION_ADDRESS;

	toggleIndicator(PIN_WARN, 30, 3);
	HAL_FLASH_Unlock();

	//Erase the application sectors in the flash memory
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = START_SECTOR; //Specify start sector number
	EraseInitStruct.NbSectors = NR_OF_SECTORS; //Specify num of sectors
	int res = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

	if (res != HAL_OK) {
		HAL_FLASH_Lock();
		return HAL_ERROR;
	}

	// Write the new firmware in chunks
	while (bytesRead < fileSize) {
		FRESULT res = f_read(file, buffer, BUFFER_SIZE, &readBytes);
		if (res != FR_OK || readBytes == 0) {
			HAL_FLASH_Lock();
			return HAL_ERROR;
		}

		for (uint32_t i = 0; i < readBytes; i += 4) {
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, *(uint32_t *)(buffer + i)) != HAL_OK) {
				HAL_FLASH_Lock();
				return HAL_ERROR;
			}
		}

		address += readBytes;
		bytesRead += readBytes;
		toggleIndicator(PIN_WARN, 30, 3);
	}

	HAL_FLASH_Lock();
	return HAL_OK;
}

void jumpToApplication(void) {
	uint32_t appJumpAddress = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
	void (*appResetHandler)(void) = (void (*)(void))appJumpAddress;

    // Deinitialize peripherals and system to reset state
	HAL_DeInit();
	 // Set the vector table to the application location
	SCB->VTOR = APPLICATION_ADDRESS;
	// Set the stack pointer
	__set_MSP(*(__IO uint32_t *)APPLICATION_ADDRESS);
	// Jump to application reset handler
	appResetHandler();
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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	if (!isBoot1High()) {
		jumpToApplication();
	}
	// Turn on the SD Card Led before accessing the SD Card.
	HAL_GPIO_WritePin(sd_led_GPIO_Port, sd_led_Pin, GPIO_PIN_SET);

	// Show the Start LED sequence.
	toggleIndicator(PIN_ERROR, 50, 1);
	toggleIndicator(PIN_WARN, 50, 1);
	toggleIndicator(PIN_INFO, 50, 1);

	FIL file;
	FRESULT res;
	uint32_t fileSize;
	FATFS FatFs;

	res = f_mount(&FatFs, SDPath, 1);

	if (res == FR_OK) {
		res = f_open(&file, FILENAME, FA_READ);
		if (res == FR_OK) {
			fileSize = f_size(&file);
			if (fileSize <= MAX_FILE_SIZE) {
				toggleIndicator(PIN_INFO, 150, 5);
				if (flashFirmwareInChunks(&file, fileSize) == HAL_OK) {
					f_close(&file);

					// Delete the firmware file from SD Card.
					f_unlink(FILENAME);

					// Show the Success LED Sequence.
					toggleIndicator(PIN_INFO, 150, 5);
					HAL_Delay(1000);


					// Show the End LED sequence.
					toggleIndicator(PIN_INFO, 50, 1);
					toggleIndicator(PIN_WARN, 50, 1);
					toggleIndicator(PIN_ERROR, 50, 1);

					// Turn the SD Card LED off.
					HAL_GPIO_WritePin(sd_led_GPIO_Port, sd_led_Pin, GPIO_PIN_RESET);


					jumpToApplication();
				}
	    	}
			f_close(&file);
	    }

	}

	// Reaching this code means there was an error.
	toggleIndicator(PIN_ERROR, 150, 5);
	// Turn the SD Card LED off.
	HAL_GPIO_WritePin(sd_led_GPIO_Port, sd_led_Pin, GPIO_PIN_RESET);
	jumpToApplication();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
