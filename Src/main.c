/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "key.h"
#include "malloc.h"
#include "custom_types.h"
#include "speex/speex.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern osSemaphoreId myBinarySemADCCpltHandle;
extern osSemaphoreId myBinarySemADCHalfCpltHandle;

int16_t buffer[FRAME_SIZE*2];
int16_t buffer_U[FRAME_SIZE];
char dec_in_buffer_1[ENCODED_FRAME_SIZE];
char dec_in_buffer_2[ENCODED_FRAME_SIZE];
uint8_t dec_in_buffer_1_dec_busy; //DMA双缓冲标记[DMA传输] 1忙，向2缓冲区传递数据；0不忙，向1缓冲区传递数据
uint32_t len,i;

//speex
__IO int16_t SPEEX_ENC_IN_BUFFER[FRAME_SIZE];
__IO uint16_t SPEEX_ENC_IN_BUFFER_U[FRAME_SIZE];
__IO int16_t SPEEX_DEC_OUT_Buffer_1[FRAME_SIZE];
__IO int16_t SPEEX_DEC_OUT_Buffer_2[FRAME_SIZE];

__IO uint16_t SPEEX_DEC_OUT_Buffer_1_U[FRAME_SIZE];
__IO uint16_t SPEEX_DEC_OUT_Buffer_2_U[FRAME_SIZE];
uint32_t Encoded_Frames=0;

SpeexBits bits,decBits;/* Holds bits so they can be read and written by the Speex routines */
void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */
int quality = 4, complexity=1, vbr=0, enh=1;/* SPEEX PARAMETERS, MUST REMAINED UNCHANGED */

extern uint8_t Start_Decoding;
extern uint8_t Playing_buf;

char out_bytes[ENCODED_FRAME_SIZE];
char remote_dec_in_bytes[ENCODED_FRAME_SIZE];
uint8_t can_bus_encoded_buf[ENCODED_FRAME_SIZE];
uint8_t can_bus_buf[ENCODED_FRAME_SIZE];
char input_bytes[ENCODED_FRAME_SIZE*2]; //使用双缓冲，DMA传输提供HALFCplt和Cple两个信号，每个信号间隔【10ms】【为数据流节拍】
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	int frame_size=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	KEY_Init();
	my_mem_init(SRAMIN); //初始化内部内存池
	
	speex_bits_init(&bits);
	//编码器初始化
	enc_state = speex_encoder_init(&speex_nb_mode);
	speex_encoder_ctl(enc_state, SPEEX_SET_VBR, &vbr);
	speex_encoder_ctl(enc_state, SPEEX_SET_QUALITY,&quality);
	speex_encoder_ctl(enc_state, SPEEX_SET_COMPLEXITY, &complexity);
	//解码器初始化
	speex_bits_init(&decBits);
	/*Create a new decoder state in narrowband mode*/
	dec_state = speex_decoder_init(&speex_nb_mode);
	speex_decoder_ctl(dec_state, SPEEX_SET_ENH, &enh);
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_DAC_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
	
	CAN1_USER_CONFIG();  //CAN 用户自定义，cubeMX有些内容没有自动生成
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); //启动canbus中断接收
	
	//HAL_UART_Transmit_DMA(&huart1, buffer, len);
	
	//vTraceEnable(TRC_INIT);
	//vTraceEnable(TRC_START);
	/*启动trace选项*/
	//vTraceEnable(TRC_START_AWAIT_HOST);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
/* ADC DMA 传输完成一半回调函数 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvHalfCpltCallback could be implemented in the user file
   */
	//HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);
	//xSemaphoreGiveFromISR(myBinarySemADCHalfCpltHandle, NULL);
	uint32_t i;
	int32_t tmp;
	//12bitADC采用左对齐方式【16bit half word】
	for(i=0;i<FRAME_SIZE;i++){
		tmp = (buffer[i] - 0x800) * 0x7FFF / 0x800;
		buffer[i] = (int16_t)tmp;
		//buffer[i] -= DATA_WIDTH_MAX_HALF; //平移操作，位于0值上下
	}
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)buffer, (uint32_t)SPEEX_ENC_IN_BUFFER, FRAME_SIZE);

	//临时启动DAC DMA传输，测试音频通路
	//for(i=0;i<)
	//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)buffer, FRAME_SIZE, DAC_ALIGN_12B_R);
}

/* ADC DMA 传输结束回调函数 */
/* 这里的ADC双缓冲是通过发送DMA传输一半信号和传输完成信号实现的*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
	
	//HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
	uint32_t i;
	int32_t tmp;
	//12bitADC采用左对齐方式【16bit half word】
	for(i=0;i<FRAME_SIZE;i++){
		tmp = (buffer[i + FRAME_SIZE] - 0x800) * 0x7FFF / 0x800;
		buffer[i + FRAME_SIZE] = (int16_t)tmp;
//		buffer[i + FRAME_SIZE] <<= 3;
//		buffer[i + FRAME_SIZE] -= DATA_WIDTH_MAX_HALF; //平移操作，位于0值上下
	}
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer, FRAME_SIZE*2);
	
	/*启动 memory to memory DMA传输，将双缓冲中的数据依次送往SPEEX编码缓冲区*/
	/*将双缓冲的后半部分通过DMA传输至SPEEX编码缓冲区*/
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)&buffer[FRAME_SIZE], (uint32_t)SPEEX_ENC_IN_BUFFER, FRAME_SIZE);

	//临时启动DAC DMA传输，测试音频通路
	//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)&buffer[FRAME_SIZE], FRAME_SIZE, DAC_ALIGN_12B_R);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdac);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_ConvHalfCpltCallbackCh1 could be implemented in the user file
   */
	
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(hdac);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_DAC_ConvCpltCallback could be implemented in the user file
   */
	//HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);
	if(Playing_buf == 1){
		Playing_buf =2; //切换播放buf2
		Start_Decoding = 1; //buf1播放完之后向buf1中解码数据
		HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_2_U, FRAME_SIZE, DAC_ALIGN_12B_R);
		
		
	}
	else if(Playing_buf == 2){  
		Playing_buf = 1;//切换播放buf1
		Start_Decoding = 2; //buf1播放完之后向buf1中解码数据
		HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_1_U, FRAME_SIZE, DAC_ALIGN_12B_R);
		
	}
}
/**
  * @brief  Ovveride the _speex_putc function of the speex library
  * @param  None
  * @retval : None
  */
void _speex_putc(int ch, void *file)
{
  while(1)
  {
  };
}



/**
  * @brief  Ovveride the _speex_fatal function of the speex library
  * @param  None
  * @retval : None
  */
void _speex_fatal(const char *str, const char *file, int line)
{
  while(1)
  {
  };
}		

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */
//	if (htim->Instance == TIM3) {
//		//HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
//	}
/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
