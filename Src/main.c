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

/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
#include "string.h"
#include "Alerts.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile uint8_t uart_in[32];
volatile char * uart_in_p = (char *) &uart_in;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void BUZZ (uint8_t time_x100ms);
void BLINK (uint8_t time_x100ms, uint8_t led, MODE mode);
uint8_t atoi8 (uint8_t* start);
uint8_t Parse_Packet (uint8_t *in);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* FOR USE */
LED_driver LD1, LD2, LD3, LD4;
LED_driver* LED_p[4];
BUZ_driver Buzzer;

#define IS_DIGIT(x) (((x)>47) && ((x)<58))
#define PACKET_SIZE 5   //NRFx.payload_length
nrf24l01_dev nrf1;
uint8_t i,reg;
uint8_t push[32] = "heyoo";
uint8_t pull[32] = "yoohe";
uint8_t ADDR_1[5] = {'R','F','1','0','3'};  //TX, RX_P0
uint8_t ADDR_2[5] = {'R','F','4','0','7'};  //PF_P1
uint8_t PIPE_1[5] = {'P','I','P','E','1'};
uint8_t uart_str[32] = "";
char* uart_str_p = (char *)uart_str;
/* FOR DEBUG */
HAL_SPI_StateTypeDef spi_state;
//static NRF_RESULT res;
volatile uint8_t IRQ1_counter, IRQ2_counter, IRQ_flags;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

    LD1.LED_Port = GPIOA;   LD1.LED_Pin = GPIO_PIN_15;
    LD2.LED_Port = GPIOB;   LD2.LED_Pin = GPIO_PIN_3;
    LD3.LED_Port = GPIOB;   LD3.LED_Pin = GPIO_PIN_4;
    LD4.LED_Port = GPIOB;   LD4.LED_Pin = GPIO_PIN_5;
    LED_p[0]=&LD1; LED_p[1]=&LD2; LED_p[2]=&LD3; LED_p[3]=&LD4; 
    
    Buzzer.TIM_Handle = &htim2;
    Buzzer.TIM_Channel = TIM_CHANNEL_1;
    
    
    
    IRQ1_counter = 0;
    nrf1.DATA_RATE = NRF_DATA_RATE_2MBPS;
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
    nrf1.NRF_CE_GPIO_PIN = GPIO_PIN_1;
    nrf1.NRF_IRQ_GPIOx = GPIOA;
    nrf1.NRF_IRQ_GPIO_PIN = GPIO_PIN_2;
    
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
//HAL_Delay(2000);  
    USART1->CR1 |= (1<<5); //RXNEIE = RX not empty interrupt enable;
    HAL_TIM_OC_Start (&htim4, TIM_CHANNEL_1);   //enable buzz timer
    HAL_TIM_OC_Start (&htim4, TIM_CHANNEL_2);   //enable blink timer
    TIM4->DIER |= TIM_IT_CC2;                   //enable blink interrupt
  sprintf(uart_str_p, "\nAssimilation successful. %u\n",SystemCoreClock);
  HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
/*DEBUG LINE*/  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
/*DEBUG LINE*/  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    
    sprintf(uart_str_p, "NRF1 SET returns=%d\n",NRF_Init(&nrf1));
    HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100); 


    
 /*HERE STARTS FUN*/
    NRF_SendPacket(&nrf1,push);
    //BUZZ (1);   //REPORT FOR DUTY!
    BLINK (5, 1, PERM);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      

      /*NO FUN WITHOUT CHECK*/
    NRF_ReadRegister(&nrf1, NRF_CONFIG, &reg);
      
//    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
//        NRF_EnableRXDataReadyIRQ(&nrf1, 0);
//    else NRF_EnableRXDataReadyIRQ(&nrf1, 1); //NRF_EnableTXDataSentIRQ
      //BAD IDEA - CONSTANT REG ABUSE
    
    if ( !(reg&0x80) && (reg & 2) )
    {           /*==========FUN HERE==========*/ 
        
        if ((IRQ_flags&(1<<6))&&(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)))   // RX FIFO Interrupt
        {
            //sprintf(uart_str_p, "\nSPAWN MORE OVERLORDS!!!\n");
            IRQ_flags &= ~(1<<6);    //drop flag
            sprintf(uart_str_p, "%s\n",nrf1.RX_BUFFER);
            HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
            BLINK(2, 2, ONCE);      //report outside
        }
        if (IRQ_flags&(1<<5))   // TX Data Sent Interrupt
        {
            IRQ_flags &= ~(1<<5);    //drop flag
            BLINK(2, 3, ONCE);      //report outside
        }
        if (IRQ_flags&(1<<4))   // MaxRetransmits reached
        {
            IRQ_flags &= ~(1<<4);    //drop flag
            BLINK(2, 4, ONCE);      //report outside
        }
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
                NRF_SendPacket(&nrf1,push);
            
    }
    else
        ;
//    IRQ_flags=0;
    if (uart_in[0] != 0)
    {
    Parse_Packet(uart_in);
        for (i=0;i<32;i++)
        {
            if (uart_in[i] == 0x0A || uart_in[i] == 0x0D)
                break;
            else
                uart_in[i]=0;
        } 
    }       
    HAL_Delay(500);             //REWRITE THIS SHIT ON TIM4_CH3
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18181;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 9091;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 49999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE1_Pin|BUTTON_Pin|CS1_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE2_Pin|CS2_Pin|LED2_Pin|LED3_Pin 
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE1_Pin BUTTON_Pin CS1_Pin LED1_Pin */
  GPIO_InitStruct.Pin = CE1_Pin|BUTTON_Pin|CS1_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ1_Pin */
  GPIO_InitStruct.Pin = IRQ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ2_Pin */
  GPIO_InitStruct.Pin = IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CE2_Pin CS2_Pin LED2_Pin LED3_Pin 
                           LED4_Pin */
  GPIO_InitStruct.Pin = CE2_Pin|CS2_Pin|LED2_Pin|LED3_Pin 
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : No_TX_Pin No_RX_Pin */
  GPIO_InitStruct.Pin = No_TX_Pin|No_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void BUZZ (uint8_t time_x100ms)
    {
        if (time_x100ms < 101)
        {
            Buzzer.timer = time_x100ms+1;  //due to pre-decrement in ISR it stops on reaching 1
            Buzzer.State = START;
            TIM4->DIER |= TIM_IT_CC1;
        }
        else return;
        return;
    }
    
    /*Send time=0 to stop*/
void BLINK (uint8_t time_x100ms, uint8_t led, MODE mode)
    {
        switch (led)
        {
            case 1: 
                {
                LD1.timer = time_x100ms;
                LD1.Mode  = mode;
                if (!time_x100ms)
                    LD1.State = STOP;
                else    
                LD1.State = START;
                break;
                };
            case 2: 
                {
                LD2.timer = time_x100ms;
                LD2.Mode  = mode;
                if (!time_x100ms)
                    LD2.State = STOP;
                else  
                LD2.State = START;
                break;
                }
            case 3: 
                {
                LD3.timer = time_x100ms;
                LD3.Mode  = mode;
                if (!time_x100ms)
                    LD3.State = STOP;
                else  
                LD3.State = START;
                break;
                }
            case 4: 
                {
                LD4.timer = time_x100ms;
                LD4.Mode  = mode;
                if (!time_x100ms)
                    LD4.State = STOP;
                else  
                LD4.State = START;
                break;
                }
            default:
            {
                return;
            }
        }
        return;
    }

uint8_t Parse_Packet (uint8_t *in)
{
	
	uint8_t *p;
	uint8_t temp;
	uint8_t led;
    if (!strncmp ((char *)in,"BUZZ",4))
    {
        if (in[4]=='\f' || in[4]=='\n')
            BUZZ(2);
        if (in[4]==' ')
            BUZZ(atoi8(&in[5]));
        return 0;
    }
    if (!strncmp ((char *)in,"BLINK",5))
    {
        if (in[5]=='\f' || in[5]=='\n')
            BLINK(3,4,ONCE);
        if (in[5]==' ')
        {
            //parse input values
            p = &in[6];
            temp=atoi8(p); //GET TIME
            led=0;        //GET LED
            while (IS_DIGIT(*p)) p++;
            p++;
            led = (*p) - 48;
            if (!( (led>0) && (led<5) ))    //wrong LED selected - do nothing
                return 2;
            p+=2;
            if (!strncmp ((char *)p,"PERM",4))
                BLINK (temp, led, PERM);
            else BLINK (temp,led, ONCE);     //default must be "ONCE"
        }
        return 0;
    }
    
    //NRF ReadReg
    if (!strncmp ((char *)in,"NRF_REG",7))
    {
        temp = atoi8 (&in[8]);
        NRF_ReadRegister(&nrf1, temp, &reg);
        sprintf(uart_str_p, "NRF_REG=0x%x\n",reg);
        HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
        return 0;
    }
    if (!strncmp ((char *)in,"CONFIG",6))
    {
        NRF_ReadRegister(&nrf1, NRF_CONFIG, &reg);
        sprintf(uart_str_p, "CONFIG=0x%x\n",reg);
        HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
        return 0;
    }
    if (!strncmp ((char *)in,"RF_SETUP",8))
    {
        NRF_ReadRegister(&nrf1, NRF_RF_SETUP, &reg);
        sprintf(uart_str_p, "RF_SETUP=0x%x\n",reg);
        HAL_UART_Transmit(&huart1, uart_str, strlen(uart_str_p), 100);
        return 0;
    }
    else return 1;                      //command not recognized
}

uint8_t atoi8 (uint8_t* start)
{
    uint8_t     i=0;
    uint16_t    res=0;  //for not to crop on "res*=10"
    for (i=0;i<32;i++)  //for if there's no digits at all don't run away too far
        while (!IS_DIGIT(*start))
            start++;
    for (i=0;i<3;i++)   //since it's 0..255 read no more than 3 chars
        {
            if (IS_DIGIT(start[i]))
            {
                res+=start[i]-48;
                res*=10;
            }
            else break;
        }
        return (uint8_t) (res/10);
}
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
