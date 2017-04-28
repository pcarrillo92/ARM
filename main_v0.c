/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

#define DIR_RIGHT 0 //CLOCKWISE
#define DIR_LEFT 1

#define MICSTP_32 0x5

//COMANDOS
#define COMM_NONE 0
#define COMM_STOP 1
#define COMM_MOVE 2
#define COMM_DIR 3
#define COMM_QUERY 4

//ESTADOS
#define STT_RDYSTP 0 //READY/STOP
#define STT_MOVE 1
#define STT_DIR 2
#define STT_QUERY 3

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

char buffer[2];
uint16_t TxData[11];
uint16_t RxData[11];

typedef struct data{	//global data
	float SPEED;
	short DIR;
	unsigned int MICSTP;
	//etc
};
struct data DATA;

int parser_res;
int status_res;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


int Parser();//decodifica el string de entrada y devuelve un COMMAND
int STT(int input, int comm);//interpreta el comando y devuelve un STATE
void Controller(int status);//no es necesario pasarle el struct de data al ser variable global
//Controller realiza las operaciones necesarias dependiendo del estado que reciba, usando
//las variables globales que ha actualizado el propio Parser


void HAL_Print(char *arg);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  DRV_Init();
  DRV_Stop();
  HAL_UART_Receive_IT(&huart2,buffer,1);

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

/** System Clock Configuration
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
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

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 300;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 150;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|SLEEPn_Pin|CS_step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_Pin SLEEPn_Pin CS_step_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|SLEEPn_Pin|CS_step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : nFAULT_Pin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

///////////////FUNCTIONS//////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	parser_res = Parser();
	status_res = STT(0, parser_res);
	Controller(status_res);
	//HAL_Delay(100); //un delay aqui atasca el micro(??)

	//Maintain UART listening
	HAL_UART_Receive_IT(&huart2,buffer,1);
}
int Parser(){

	switch(buffer[0]){
		case 'M':
			buffer[0] = 0;
			return COMM_MOVE;
			break;
		case 'S':
			buffer[0] = 0;
			return COMM_STOP;
			break;
		case 'D':
			buffer[0] = 0;

		case 'L':
			DATA.DIR=DIR_LEFT;
			return COMM_DIR;
			break;
		case 'R':
			DATA.DIR=DIR_RIGHT;
			return COMM_DIR;
			break;
		case 'Q':
			buffer[0] = 0;
			return COMM_QUERY;
			break;
		default:
			buffer[0] = 0;
			HAL_Print("COMANDO INVALIDO\n\r");
			return COMM_NONE;
			break;
	}
}
int STT(int input, int comm){//recibe los inputs, y parser_res
	static short status;
	switch(comm){
		case COMM_NONE://no commands received
			return status;
			break;
		case COMM_STOP://parser resolves a stop command incoming
			status=STT_RDYSTP;//state changes to READY/STOP
			return status;
			break;
		case COMM_MOVE:
			status=STT_MOVE;//state changes to MOVING
			return status;
			break;
		case COMM_DIR:
			status=STT_DIR;//state changes to directing
			return status;
			break;
		case COMM_QUERY:
			status=STT_QUERY;
			return status;
			break;
		default:
			break;
	}
}
void Controller(int status){
	switch(status){
		case STT_RDYSTP:
			DRV_Stop();
			break;
		case STT_MOVE:
			DRV_Start();
			break;
		case STT_DIR:
			DRV_Direction(DATA.DIR);
			break;
		case STT_QUERY:
			DRV_Query();
			break;
		default:
			break;
	}
}
void DRV_Start(){
	HAL_Print("MOTOR EN MOVIMIENTO\n\r");
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
void DRV_Stop(){
	HAL_Print("MOTOR PARADO\n\r");
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}
void DRV_Direction(int dir){
	HAL_GPIO_WritePin(GPIOA, DIR_Pin, dir); //RESET is counterclockwise
	HAL_Print("CAMBIO DE SENTIDO\n\r");
}
void DRV_Query(){
	HAL_Print("PETICION DE CONSULTA\n\r");
}
void HAL_Print(char *arg){
	int n;
	char aux_buff[100];
	n=sprintf(aux_buff,arg);
	HAL_UART_Transmit(&huart2,arg,n,400);
}
void DRV_Init(){
	//Inizialize demo board

		//Clear reset
		HAL_GPIO_WritePin(GPIOA, SLEEPn_Pin, GPIO_PIN_SET);
		// Wait fot wake
		HAL_Delay(10);

		/////////////////////WRITE MESSAGE/////////////////////
		// message data write 00
		// Activate SPI interface
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_SET);

		RxData[0] =
				(1 	<< 0) 	+	//Enable motor
				(0 	<< 1) 	+ 	//RDIR
				(0 	<< 2) 	+	//...
				(0x5<< 3) 	+	//Microsteping
				(0 	<< 7) 	+	//...
				(0x3<< 8) 	+	//...
				(0x3<< 10);		//...
		// message mode
		RxData[0] +=
				(0	<< 15) +		// read/write bit (0 for write)
				(0x0<< 12);		// internal register map address CTRL

		HAL_SPI_Transmit(&hspi1, RxData, 2, 500);//necesario pasarle RxData como parametro (no RxData[0])
		//HAL_UART_Transmit(&huart2,RxData,2, 1000);

		// End msg
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_RESET);
		//Needed to end the message

		// Wait for processing
		HAL_Delay(10);


		/////////////////////READ CTRL/////////////////////
		// message data read 00
		// Activate SPI interface
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_SET);

		RxData[0] = 0;//Clear all the bits
		// message mode
		RxData[0] +=
				(1	<< 15) +		// read/write bit (1 for read)
				(0x0<< 12);		// internal register map address CTRL

		HAL_SPI_TransmitReceive(&hspi1, RxData, TxData, 1, 500);


		// End msg
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_RESET);

		// Wait for processing
		HAL_Delay(10);
	/*
		/////////////////////WRITE STATUS/////////////////////
		// message data read STT (STATUS)
		// Activate SPI interface
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_SET);

		RxData[0] = 0;
		// message mode
		RxData[0] +=
				(0	<< 15) +		// read/write bit (0 for write)
				(0x7<< 12);		// internal register map address STATUS

		HAL_SPI_TransmitReceive(&hspi1, RxData, TxData, 1, 500);

		// End msg
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_RESET);

		// Wait for processing
		HAL_Delay(10);
	*/
		/////////////////////READ STATUS/////////////////////
		// message data read STT
		// Activate SPI interface
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_SET);

		RxData[0] = 0;
		// message mode
		RxData[0] +=
				(1	<< 15) +		// read/write bit (0 for write)
				(0x5<< 12);		// internal register map address

		HAL_SPI_TransmitReceive(&hspi1, RxData, TxData, 1, 500);
		//HAL_UART_Transmit(&huart2,TxData,2,500);//TxData recibe el anterior envio de datos, necesario siempre
												//enviar otro paquete de datos para recibir el paquete anterior
		HAL_SPI_TransmitReceive(&hspi1,RxData,TxData,1,500);
		//HAL_UART_Transmit(&huart2,TxData,2,500);//txData contiene el registro STATUS

		// End msg
		HAL_GPIO_WritePin(GPIOA, CS_step_Pin, GPIO_PIN_RESET);

		// Wait for processing
		HAL_Delay(10);

		// start step/dir interface
		HAL_Delay(1000);
		//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_GPIO_WritePin(GPIOA, DIR_Pin, GPIO_PIN_RESET); //RESET is counterclockwise
}

////////////END OF FUNCTIONS/////////////

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
