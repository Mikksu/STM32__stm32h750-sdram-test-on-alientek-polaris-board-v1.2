/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "dwt_impl.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint32_t testsram[250000] __attribute__((section(".ARM.__at_0XC0000000")));
static uint32_t * const testsram = (uint32_t *) (Bank5_SDRAM_ADDR);


#define SDRAM_1M            (1024 * 1024)
#define SDRAM_M_UNIT        (4 * SDRAM_1M)
#define SDRAM_D_WIDTH				(2 * 16)        // since we have 2 SDRAM chips on the board
#define SDRAM_BANK          (4)
#define SDRAM_TOTAL_BITS    (SDRAM_M_UNIT * SDRAM_D_WIDTH * SDRAM_BANK)
#define SDRAM_TOTAL_UNIT		(SDRAM_TOTAL_BITS / SDRAM_D_WIDTH)
//#define SDRAM_TOTAL_UNIT		(2 ^ 13)      // The total storage unit per table.

#define SRAM_64K           (64 * 1024)
static uint32_t sramBuff[SRAM_64K];
static void dmaTest(void)
{
  int i = 0;
  HAL_StatusTypeDef status;

  for(i = 0; i < SRAM_64K; i++)
    sramBuff[i] = i;

  status = HAL_DMA_Start(&hdma_memtomem_dma1_stream0, (uint32_t)&sramBuff, Bank5_SDRAM_ADDR, SRAM_64K);
  if(status == HAL_OK)
    printf("DMA Transfer to SDRAM finished!\r\n");
  else
    printf("DMA Transfer to SDRAM failed!\r\n");

  // validate the data in the SDRAM
  if(status == HAL_OK)
  {
    uint32_t *p = (uint32_t*)Bank5_SDRAM_ADDR;
    uint32_t temp;

    for(i = 0; i < SRAM_64K; i++)
    {
      temp = *p++;
      if(temp != i)
        break;
    }

    if(i != SRAM_64K - 1)
      printf("Data error in position %d\r\n", i);
    else
      printf("Data validation passed\r\n");

    HAL_DMA_Abort(&hdma_memtomem_dma1_stream0);
  }
}

static void WriteSpeedTest(void)
{
	uint32_t i, j;
	uint32_t *pBuf;


	/* ���ó�ʼ��ֵ�����¿�ʼʱ�� */
	j = 0;
	pBuf = (uint32_t *)Bank5_SDRAM_ADDR;

	
	/* �Ե����ķ�ʽд���ݵ�SDRAM���пռ� */
	for (i = 1024*1024/4; i >0 ; i--)
	{
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;	

		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;	

		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;	

		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;
		*pBuf++ = j++;	
	}
	

	j = 0;
	pBuf = (uint32_t *)Bank5_SDRAM_ADDR;
	for (i = 0; i < 1024*1024*8; i++)
	{
		if(*pBuf++ != j++)
		{
			printf("Failed to write data to the SDRAM j=%d\r\n", j);
			break;
		}
	}
}
  
void fmc_sdram_test(void)
{  
  float wr_cost = 0, rd_cost = 0;

	uint32_t i = 0;  	  
	uint32_t temp = 0;	   
	uint32_t sval = 0;		
	

  /*************** WRITE **********************/
	htim2.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim2);

	uint32_t* pram = (uint32_t*)(Bank5_SDRAM_ADDR);
	for(i = 0; i < SDRAM_TOTAL_UNIT; i++)
	{
		*pram++ = temp++;
	}
	
  htim2.Instance->CR1 &= 0;
  HAL_TIM_Base_Stop(&htim2);
  wr_cost = (float)htim2.Instance->CNT;

  /*************** WRITE END **********************/


  /*************** READ **********************/
  htim2.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim2);

	pram = (uint32_t*)(Bank5_SDRAM_ADDR);
 	for(i = 0; i < SDRAM_TOTAL_UNIT; i++)
	{	
		temp = *pram++;
		if(i==0)
			sval = temp;
 		else if(temp <= sval)
			break;
	}
	
  htim2.Instance->CR1 &= 0;
  HAL_TIM_Base_Stop(&htim2);
  rd_cost = (float)htim2.Instance->CNT;
  

  /*************** READ END **********************/

	float totalWords = (uint32_t)(temp - sval + 1);
	float totalBytes = totalWords * sizeof(uint32_t);
	
  printf("Total bytes written: %.0f, total words written: %.0f\r\n", totalBytes, totalWords);
	printf("SDRAM Capacity:%.3fMB\r\n",(totalBytes / SDRAM_1M));

	float sysClk = HAL_RCC_GetSysClockFreq();
  float wr_t_s =  wr_cost * (1 / sysClk * 2);
  float rd_t_s =  rd_cost * (1 / sysClk * 2);
  printf("WRITE cost: %.3fms; speed: %.1f MB/s\r\n", wr_t_s * 1000, (totalBytes / SDRAM_1M) / wr_t_s);
  printf("READ cost: %.3fms; speed: %.1f MB/s\r\n", rd_t_s * 1000, (totalBytes / SDRAM_1M) / rd_t_s);
 				 
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float cost_ms;
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  uint32_t sysClk =  HAL_RCC_GetSysClockFreq();
	printf("STM32H750 .....\r\n");
  printf("CPU Clock Freq.: %dMHz\r\n", sysClk / 1000000);
	
	DWT_ENABLE();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		fmc_sdram_test();
		//WriteSpeedTest();
		//dmaTest();

		
		//HAL_Delay(1000);
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
