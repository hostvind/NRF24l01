/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE BEGIN Includes */
#include "nrf24l01.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* FOR USE */
#define PACKET_SIZE 5   //NRFx.payload_length
nrf24l01_dev nrf1;
nrf24l01_dev nrf2;
uint8_t i;
uint8_t push[32] = "heyoo";
uint8_t pull[32] = "yoohe";
uint8_t ADDR_1[5] = {'H','V','I','N','D'};
uint8_t ADDR_2[5] = {'H','J','E','L','L'};
uint8_t PIPE_1[5] = {'P','I','P','E','1'};
uint8_t uart_str[32] = "";
/* FOR DEBUG */
HAL_SPI_StateTypeDef spi_state;
static NRF_RESULT res;
volatile uint8_t IRQ1_counter, IRQ2_counter;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    IRQ1_counter = 0;
    IRQ2_counter = 0;
    nrf1.DATA_RATE = NRF_DATA_RATE_250KBPS;
    nrf1.TX_POWER = NRF_TX_PWR_0dBm;
    nrf1.CRC_WIDTH = NRF_CRC_WIDTH_1B;
    nrf1.ADDR_WIDTH = NRF_ADDR_WIDTH_5;
    nrf1.PayloadLength = PACKET_SIZE; // maximum is 32 Bytes
    nrf1.RetransmitCount = 15; // maximum is 15 times
    nrf1.RetransmitDelay = 0x0F; // 4000us, LSB:250us
    
    nrf1.RF_CHANNEL = 0;
    nrf1.RX_ADDRESS = ADDR_1;   /*Pipe0 must be the same as target addr - for ACK packets*/
    nrf1.TX_ADDRESS = ADDR_1;   /*Target addr*/
    
    nrf1.NRF_IRQn = EXTI2_IRQn;
    nrf1.spi=&hspi1;
	
	nrf1.NRF_CSN_GPIOx = GPIOA;
    nrf1.NRF_CSN_GPIO_PIN = GPIO_PIN_4;
    nrf1.NRF_CE_GPIOx = GPIOA;
    nrf1.NRF_CE_GPIO_PIN = GPIO_PIN_3;
    nrf1.NRF_IRQ_GPIOx = GPIOA;
    nrf1.NRF_IRQ_GPIO_PIN = GPIO_PIN_2;
	
	nrf2.DATA_RATE = NRF_DATA_RATE_250KBPS;
    nrf2.TX_POWER = NRF_TX_PWR_0dBm;
    nrf2.CRC_WIDTH = NRF_CRC_WIDTH_1B;
    nrf2.ADDR_WIDTH = NRF_ADDR_WIDTH_5;
    nrf2.PayloadLength = PACKET_SIZE; // maximum is 32 Bytes
    nrf2.RetransmitCount = 10; // maximum is 15 times
    nrf2.RetransmitDelay = 0x0F; // 4000us, LSB:250us
    
    nrf2.RF_CHANNEL = 0;
    nrf2.RX_ADDRESS = ADDR_1;   /*This is the target*/
    nrf2.TX_ADDRESS = ADDR_2;   /*As long as we are in PRx mode, we don't care*/
    
    nrf2.NRF_IRQn = EXTI15_10_IRQn;
    nrf2.spi=&hspi2;
	
    nrf2.NRF_CSN_GPIOx = GPIOB;
    nrf2.NRF_CSN_GPIO_PIN = GPIO_PIN_12;
    nrf2.NRF_CE_GPIOx = GPIOB;
    nrf2.NRF_CE_GPIO_PIN = GPIO_PIN_11;
    nrf2.NRF_IRQ_GPIOx = GPIOB;
    nrf2.NRF_IRQ_GPIO_PIN = GPIO_PIN_10;	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */ 
//HAL_Delay(2000);  
  sprintf(uart_str, "\nAssimilation successful. %u\n",SystemCoreClock);
  HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);
/*DEBUG LINE*/  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
/*DEBUG LINE*/  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    
    sprintf(uart_str, "NRF1 SET returns=%d\n",NRF_Init(&nrf1));
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
    
	sprintf(uart_str, "NRF2 SET returns=%d\n",NRF_Init(&nrf2));
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);
// NRF_PowerUp(&nrf1, 0);
// NRF_PowerUp(&nrf2, 0);
// HAL_Delay(333);
// NRF_PowerUp(&nrf1, 1);
// NRF_PowerUp(&nrf2, 1);
// HAL_Delay(333);
 /*HERE STARTS FUN*/
    NRF_SendPacket(&nrf1,push);

 /*TRY PIPE1*/
    if (NRF_SetPipeAddress(&nrf2, PIPE_1, 1) != NRF_OK)
        sprintf(uart_str, "PIPE1 NOT SET\n");
    else
        sprintf(uart_str, "PIPE1 SET\n");
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);    
    
    
    NRF_ReadRegister(&nrf2, NRF_EN_AA, &i);
    sprintf(uart_str, "NRF2 EN_AA=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);    
    
    /*let's enable ALL pipes ^^*/
    i=0x3F;
    NRF_WriteRegister(&nrf2, NRF_EN_RXADDR, &i);
    NRF_ReadRegister(&nrf2, NRF_EN_RXADDR, &i);
    sprintf(uart_str, "NRF2 EN_RXADDR=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
    
    //set NRF2_P1 payload width
    NRF_WriteRegister(&nrf2, NRF_RX_PW_P1, &nrf2.PayloadLength);
    /*NRF2 is PIPE1 set right?*/
    sprintf(uart_str, "NRF2 PIPE1_ADDR=");
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);
    NRF_SendCommand(&nrf2, NRF_RX_ADDR_P1, uart_str+20, uart_str, 5);
    uart_str[5]='\n';
    HAL_UART_Transmit(&huart1, uart_str, 6, 100);  
    
            NRF_SetTXAddress    (&nrf1,PIPE_1);
            NRF_SetRXAddress_P0 (&nrf1,PIPE_1);
            NRF_SendPacket(&nrf1,pull);
    
   /*read regs*/ 
    
    sprintf(uart_str, "NRF1 TX_ADDR=");
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);
    NRF_SendCommand(&nrf1, NRF_TX_ADDR, uart_str+20, uart_str, 5);
    uart_str[5]='\n';
    HAL_UART_Transmit(&huart1, uart_str, 6, 100);  
    
    sprintf(uart_str, "NRF1 RX_ADDR=");
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);
    NRF_SendCommand(&nrf1, NRF_RX_ADDR_P0, uart_str+20, uart_str, 5);
    uart_str[5]='\n';
    HAL_UART_Transmit(&huart1, uart_str, 6, 100);  
    
     
    NRF_ReadRegister(&nrf1, NRF_CONFIG, &i);
    sprintf(uart_str, "NRF1 CONFIG=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
    NRF_ReadRegister(&nrf2, NRF_CONFIG, &i);
    sprintf(uart_str, "NRF2 CONFIG=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
    NRF_ReadRegister(&nrf1, NRF_STATUS, &i);
    sprintf(uart_str, "NRF1 STATUS=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
    NRF_ReadRegister(&nrf2, NRF_STATUS, &i);
    sprintf(uart_str, "NRF2 STATUS=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
    NRF_ReadRegister(&nrf2, NRF_RX_PW_P1, &i);
    sprintf(uart_str, "NRF2 RX_PW_P1=%x\n",i);
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str), 100);  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if (IRQ1_counter != 0)
    {
        /*Debug here if interrupt occurs*/
        sprintf(uart_str, "NRF1 IRQ count: %d\n", IRQ1_counter);
        HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str),100);
        IRQ1_counter=0;
    }
    if (IRQ2_counter != 0)
    {        
        /*Debug here if interrupt occurs*/
        sprintf(uart_str, "NRF2 IRQ count: %d\n", IRQ2_counter);
        HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str),100);
        IRQ2_counter=0;
/*DEBUG LINE*/  NRF_ReadRXPayload(&nrf2, pull);
/*DEBUG LINE*/  HAL_UART_Transmit(&huart1, pull, strlen(pull),100);
/*DEBUG LINE*/  HAL_UART_Transmit(&huart1, "\n", 1,100);
    }
    HAL_Delay(10);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
