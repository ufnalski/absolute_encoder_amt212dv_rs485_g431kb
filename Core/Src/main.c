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
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	AMT21_DATA_OK = 0x00U, AMT21_DATA_ERROR = 0x01U
} AMT21Data_StatusTypeDef;

typedef union
{
	uint16_t uint16;
	int16_t int16;
} Uint16toInt16DecoderTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AMT21_ADDR 0x54 // can be changed using https://www.sameskydevices.com/product/motion-and-control/rotary-encoders/encoder-accessories/amt-cables/amt-pgrm-06c
#define AMT21_UART_HANDLE &huart1 // https://www.waveshare.com/wiki/RS485_Board_(3.3V)
#define AMT21_PERIOD 500
#define AMT21_TIMEOUT 1 // us

#define TOGGLE_ON_SUCCESS
//#define TOGGLE_ON_FAILURE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
const uint8_t ask_for_position[] =
{ AMT21_ADDR };
const uint8_t ask_for_turns[] =
{ AMT21_ADDR + 0x01 };

HAL_StatusTypeDef r485_receive_status;

uint8_t amt21_position_data[2];
AMT21Data_StatusTypeDef amt21_position_data_ok = AMT21_DATA_ERROR;
uint16_t amt21_position;

uint8_t amt21_turns_data[2];
AMT21Data_StatusTypeDef amt21_turns_data_ok = AMT21_DATA_ERROR;
int16_t amt21_turns;

Uint16toInt16DecoderTypeDef uint16_to_int16;

uint32_t AmtSoftTimer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
AMT21Data_StatusTypeDef IsAmtDataValid(uint8_t *_amt21_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Initialize leds */
	BSP_LED_Init(LED_GREEN);

	/* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
	BspCOMInit.BaudRate = 115200;
	BspCOMInit.WordLength = COM_WORDLENGTH_8B;
	BspCOMInit.StopBits = COM_STOPBITS_1;
	BspCOMInit.Parity = COM_PARITY_NONE;
	BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
	if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN BSP */

	/* -- Sample board code to send message over COM1 port ---- */
	printf("Welcome to Same Sky absolute encoder world!\r\n\r\n");

	/* -- Sample board code to switch on leds ---- */

	/* USER CODE END BSP */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	AmtSoftTimer = HAL_GetTick();

	while (1)
	{
		if (HAL_GetTick() - AmtSoftTimer > AMT21_PERIOD)
		{
			AmtSoftTimer = HAL_GetTick();

			HAL_UART_Transmit(AMT21_UART_HANDLE, ask_for_position,
					sizeof(ask_for_position), AMT21_TIMEOUT);
			memset(amt21_position_data, 0x0000, 2);
			r485_receive_status = HAL_UART_Receive(AMT21_UART_HANDLE,
					amt21_position_data, 2, AMT21_TIMEOUT);

			if ((HAL_OK == r485_receive_status)
					&& (AMT21_DATA_OK == IsAmtDataValid(amt21_position_data)))
			{
				amt21_position_data_ok = AMT21_DATA_OK;
#ifdef TOGGLE_ON_SUCCESS
				HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#endif
				amt21_position = ((uint16_t) (amt21_position_data[0])
						| (((uint16_t) (amt21_position_data[1])) << 8))
						& 0x3FFF;
				printf("Position: %d (%.1f degrees)\r\n", amt21_position,
						((float) amt21_position) * 360 / pow(2, 14));
			}
			else
			{
				amt21_position_data_ok = AMT21_DATA_ERROR;
#ifdef TOGGLE_ON_FAILURE
			HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
#endif
			}

			HAL_UART_Transmit(AMT21_UART_HANDLE, ask_for_turns,
					sizeof(ask_for_turns), AMT21_TIMEOUT);
			memset(amt21_turns_data, 0x0000, 2);
			r485_receive_status = HAL_UART_Receive(AMT21_UART_HANDLE,
					amt21_turns_data, 2, AMT21_TIMEOUT);

			if ((HAL_OK == r485_receive_status)
					&& (AMT21_DATA_OK == IsAmtDataValid(amt21_turns_data)))
			{
				amt21_turns_data_ok = AMT21_DATA_OK;
#ifdef TOGGLE_ON_SUCCESS
				HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
#endif
				uint16_to_int16.uint16 = ((uint16_t) (amt21_turns_data[0])
						| (((uint16_t) (amt21_turns_data[1])) << 8)) << 2;
				amt21_turns = uint16_to_int16.int16 / 4;
				printf("Number of turns: %d\r\n\r\n", amt21_turns);
			}
			else
			{
				amt21_turns_data_ok = AMT21_DATA_ERROR;
#ifdef TOGGLE_ON_FAILURE
			HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
#endif
			}

		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
AMT21Data_StatusTypeDef IsAmtDataValid(uint8_t *_amt21_data)
{
	uint16_t data = (uint16_t) (_amt21_data[0])
			| (((uint16_t) (_amt21_data[1])) << 8);
	uint8_t csum0 = 0;
	uint8_t csum1 = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		csum0 ^= (data >> i * 2) & 1;
		csum1 ^= (data >> (i * 2 + 1)) & 1;
	}
	if (csum0 && csum1)
	{
		return AMT21_DATA_OK;
	}
	else
	{
		return AMT21_DATA_ERROR;
	}
}
/* USER CODE END 4 */

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

/**
 * @}
 */

/**
 * @}
 */

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
