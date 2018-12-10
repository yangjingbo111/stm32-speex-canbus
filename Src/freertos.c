/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "key.h"
#include "usart.h"
#include "dma.h"
#include "stdlib.h"
#include "custom_types.h"
#include "speex/speex.h"
#include "speex/speex_config_types.h"
#include "dac.h"
#include "voice.h"
#include "tim.h"
#include "adc.h"
#include "can.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTaskSpeexEncHandle;
osThreadId myTaskSpeexDecHandle;
osThreadId myTaskUIHandle;
osThreadId myTaskCanBusSendHandle;
osThreadId myTaskCanRecvPreProcessHandle;
osSemaphoreId myBinarySemDMAFinishedHandle;
osSemaphoreId myBinarySemADCCpltHandle;
osSemaphoreId myBinarySemADCHalfCpltHandle;
osSemaphoreId myBinarySemTestHandle;
osSemaphoreId myBinarySemDecInBufferReadyHandle;
osSemaphoreId myBinarySemEncodCpltHandle;
osSemaphoreId myBinarySemCanRecvCpltHandle;

/* USER CODE BEGIN Variables */
extern SpeexBits bits,decBits;/* Holds bits so they can be read and written by the Speex routines */
extern void *enc_state, *dec_state;/* Holds the states of the encoder & the decoder */

extern int16_t buffer[FRAME_SIZE*2];
extern int16_t buffer_U[FRAME_SIZE];
extern char dec_in_buffer_1[ENCODED_FRAME_SIZE];
extern char dec_in_buffer_2[ENCODED_FRAME_SIZE];
extern uint8_t dec_in_buffer_1_dec_busy;
extern __IO int16_t SPEEX_ENC_IN_BUFFER[FRAME_SIZE];
extern __IO uint16_t SPEEX_ENC_IN_BUFFER_U[FRAME_SIZE];
extern __IO int16_t SPEEX_DEC_OUT_Buffer_1[FRAME_SIZE];
extern __IO int16_t SPEEX_DEC_OUT_Buffer_2[FRAME_SIZE];
extern __IO uint16_t SPEEX_DEC_OUT_Buffer_1_U[FRAME_SIZE];
extern __IO uint16_t SPEEX_DEC_OUT_Buffer_2_U[FRAME_SIZE];
extern char out_bytes[ENCODED_FRAME_SIZE];
extern char remote_dec_in_bytes[ENCODED_FRAME_SIZE];
extern uint8_t can_bus_encoded_buf[ENCODED_FRAME_SIZE];
extern uint8_t can_bus_buf[ENCODED_FRAME_SIZE];

uint8_t Start_Decoding;
uint8_t Playing_buf;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTaskSpeexEnc(void const * argument);
void StartTaskSpeexDec(void const * argument);
void StartTaskUI(void const * argument);
void StartTaskCanBusSend(void const * argument);
void StartTaskCanRecvPreProcess(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySemDMAFinished */
  osSemaphoreDef(myBinarySemDMAFinished);
  myBinarySemDMAFinishedHandle = osSemaphoreCreate(osSemaphore(myBinarySemDMAFinished), 1);

  /* definition and creation of myBinarySemADCCplt */
  osSemaphoreDef(myBinarySemADCCplt);
  myBinarySemADCCpltHandle = osSemaphoreCreate(osSemaphore(myBinarySemADCCplt), 1);

  /* definition and creation of myBinarySemADCHalfCplt */
  osSemaphoreDef(myBinarySemADCHalfCplt);
  myBinarySemADCHalfCpltHandle = osSemaphoreCreate(osSemaphore(myBinarySemADCHalfCplt), 1);

  /* definition and creation of myBinarySemTest */
  osSemaphoreDef(myBinarySemTest);
  myBinarySemTestHandle = osSemaphoreCreate(osSemaphore(myBinarySemTest), 1);

  /* definition and creation of myBinarySemDecInBufferReady */
  osSemaphoreDef(myBinarySemDecInBufferReady);
  myBinarySemDecInBufferReadyHandle = osSemaphoreCreate(osSemaphore(myBinarySemDecInBufferReady), 1);

  /* definition and creation of myBinarySemEncodCplt */
  osSemaphoreDef(myBinarySemEncodCplt);
  myBinarySemEncodCpltHandle = osSemaphoreCreate(osSemaphore(myBinarySemEncodCplt), 1);

  /* definition and creation of myBinarySemCanRecvCplt */
  osSemaphoreDef(myBinarySemCanRecvCplt);
  myBinarySemCanRecvCpltHandle = osSemaphoreCreate(osSemaphore(myBinarySemCanRecvCplt), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTaskSpeexEnc */
  osThreadDef(myTaskSpeexEnc, StartTaskSpeexEnc, osPriorityRealtime, 0, 512);
  myTaskSpeexEncHandle = osThreadCreate(osThread(myTaskSpeexEnc), NULL);

  /* definition and creation of myTaskSpeexDec */
  osThreadDef(myTaskSpeexDec, StartTaskSpeexDec, osPriorityRealtime, 0, 512);
  myTaskSpeexDecHandle = osThreadCreate(osThread(myTaskSpeexDec), NULL);

  /* definition and creation of myTaskUI */
  osThreadDef(myTaskUI, StartTaskUI, osPriorityRealtime, 0, 512);
  myTaskUIHandle = osThreadCreate(osThread(myTaskUI), NULL);

  /* definition and creation of myTaskCanBusSend */
  osThreadDef(myTaskCanBusSend, StartTaskCanBusSend, osPriorityHigh, 0, 512);
  myTaskCanBusSendHandle = osThreadCreate(osThread(myTaskCanBusSend), NULL);

  /* definition and creation of myTaskCanRecvPreProcess */
  osThreadDef(myTaskCanRecvPreProcess, StartTaskCanRecvPreProcess, osPriorityHigh, 0, 512);
  myTaskCanRecvPreProcessHandle = osThreadCreate(osThread(myTaskCanRecvPreProcess), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
	  HAL_GPIO_TogglePin(GPIOF, LED1_DS1_Pin);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint8_t keyVal;
	uint8_t src[]="this is a string";
	uint8_t dest[17];
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
		
		
	}
  /* USER CODE END StartTask02 */
}

/* StartTaskSpeexEnc function */
void StartTaskSpeexEnc(void const * argument)
{
  /* USER CODE BEGIN StartTaskSpeexEnc */
	int nbBits = 0;
	uint16_t i;
  /* Infinite loop */
	for(;;)
	{
		//osDelay(100);
		if (xSemaphoreTake(myBinarySemDMAFinishedHandle, 100)){
			//HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);
			//HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET);
			//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)SPEEX_ENC_IN_BUFFER, FRAME_SIZE);
			speex_bits_reset(&bits);
//			for(i=0;i<FRAME_SIZE;i++){
//				SPEEX_ENC_IN_BUFFER[i] -= (ADC_MAX+1)>>1;  //平移波形，分布在0上下
//			}
			speex_encode_int(enc_state, (spx_int16_t*)SPEEX_ENC_IN_BUFFER, &bits);
			nbBits = speex_bits_write(&bits, out_bytes, ENCODED_FRAME_SIZE);
			
			for(i=0;i<ENCODED_FRAME_SIZE;i++){
				can_bus_encoded_buf[i] = (uint8_t)out_bytes[i]; //将数据拷贝到canbus发送缓冲区
			}
			//HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
			//通知canbus发送语音包
			//HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);
			xSemaphoreGive( myBinarySemEncodCpltHandle ); 
			
			//以下是解码程序，暂时不使用[本地播放]
			//添加一级缓冲，为解码争取20ms【采样一帧时间决定】的处理时间。【不加缓冲的话只能和编码处在同一级20ms缓冲区中】
			// the following code can be used to test local enc-dec-play cycle when you uncomment.
			if(dec_in_buffer_1_dec_busy){
				HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream2, (uint32_t)out_bytes, (uint32_t)dec_in_buffer_2, ENCODED_FRAME_SIZE);
			}
			else{
				HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream2, (uint32_t)out_bytes, (uint32_t)dec_in_buffer_1, ENCODED_FRAME_SIZE);
			}
			
		}    
	}
  /* USER CODE END StartTaskSpeexEnc */
}

/* StartTaskSpeexDec function */
void StartTaskSpeexDec(void const * argument)
{
  /* USER CODE BEGIN StartTaskSpeexDec */
	uint16_t i;
  /* Infinite loop */
  for(;;)
  {
	  //osDelay(100);
	  if (xSemaphoreTake(myBinarySemDecInBufferReadyHandle, 100)){
//		HAL_GPIO_TogglePin(TEST3_GPIO_Port, TEST3_Pin);
		  switch(dec_in_buffer_1_dec_busy){
			case 0:
				speex_bits_read_from(&decBits, (const char *)dec_in_buffer_2, ENCODED_FRAME_SIZE);
				speex_decode_int(dec_state, &decBits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer_2);	
				/*处理数据溢出【超过12bit的4096】*/
				for(i=0;i<FRAME_SIZE;i++){
					SPEEX_DEC_OUT_Buffer_2_U[i] = SPEEX_DEC_OUT_Buffer_2[i];
					SPEEX_DEC_OUT_Buffer_2_U[i] += 32768;
					SPEEX_DEC_OUT_Buffer_2_U[i] >>= 4;
					
				}
				//解码完成，使用DAC测试效果
				//临时启动DAC DMA传输，测试音频通路
			//HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET);	
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_2_U, FRAME_SIZE, DAC_ALIGN_12B_R);
				
				break;
			case 1:
				speex_bits_read_from(&decBits, (const char *)dec_in_buffer_1, ENCODED_FRAME_SIZE);
				speex_decode_int(dec_state, &decBits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer_1);
				/*处理数据溢出【超过12bit的4096】*/
				for(i=0;i<FRAME_SIZE;i++){
					SPEEX_DEC_OUT_Buffer_1_U[i] = SPEEX_DEC_OUT_Buffer_1[i];
					SPEEX_DEC_OUT_Buffer_1_U[i] += 32768;
					SPEEX_DEC_OUT_Buffer_1_U[i] >>= 4;
					
				}
				//解码完成，使用DAC测试效果
				//临时启动DAC DMA传输，测试音频通路
			//HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET);	
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_1_U, FRAME_SIZE, DAC_ALIGN_12B_R);
				
				break;
			default:break;
		}
		
	  }



//	  if(xSemaphoreTake( myBinarySemTestHandle, 100 )){
//		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)src, 9);
//		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_SET);
//		/*Copy the data into the bit-stream struct*/
//		speex_bits_read_from(&bits, out_bytes, ENCODED_FRAME_SIZE);
//		speex_decode_int(dec_state, &bits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer);	

//		HAL_GPIO_WritePin(TEST2_GPIO_Port, TEST2_Pin, GPIO_PIN_RESET);
//	  }
  }
  /* USER CODE END StartTaskSpeexDec */
}

/* StartTaskUI function */
void StartTaskUI(void const * argument)
{
  /* USER CODE BEGIN StartTaskUI */
	uint8_t keyVal;
	uint8_t src[]="this is a string";
	uint8_t dest[17];
	uint16_t NB_Frames=0;
	uint32_t sample_index=0;
	uint16_t i=0;
	
	int16_t test_int16[2] = {0x1122,0x3344};
	Start_Decoding = 0;
	Playing_buf=0;
	/* Infinite loop */
	for(;;)
	{
		osDelay(100);
		keyVal = KEY_Scan(1); 
		//if(keyVal != 0){
			//HAL_UART_Transmit_DMA(&huart1, &keyVal, 1);
			switch(keyVal){
				case 0:
					HAL_ADC_Stop_DMA(&hadc1); 
					break;
				case 1:					
					//HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
					/*启动ADC DMA采集*/
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer, FRAME_SIZE*2);
					
				
					//xSemaphoreGive(myBinarySemTestHandle);
					//HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1, (uint32_t)src, (uint32_t)dest, 17);
				break;
				case 2:
					//回放时关闭实时声音采集
					HAL_ADC_Stop_DMA(&hadc1); 
					//回放应用
					//解码第一帧
					for(i=0;i<ENCODED_FRAME_SIZE; i++)
					{
					  dec_in_buffer_1[i] = male_voice[sample_index++];
					}
					
					/* Copy the encoded data into the bit-stream struct */
					speex_bits_read_from(&decBits, (const char *)dec_in_buffer_1, ENCODED_FRAME_SIZE);
					/* Decode the data */
					speex_decode_int(dec_state, &decBits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer_1);
					/*处理数据溢出【超过12bit的4096】*/
					for(i=0;i<FRAME_SIZE;i++){
						SPEEX_DEC_OUT_Buffer_1_U[i] = SPEEX_DEC_OUT_Buffer_1[i];
						SPEEX_DEC_OUT_Buffer_1_U[i] += 32768;
						SPEEX_DEC_OUT_Buffer_1_U[i] >>= 4;
						
					}
					//解码第二帧
					for(i=0;i<ENCODED_FRAME_SIZE; i++)
					{
					  dec_in_buffer_1[i] = male_voice[sample_index++];
					}
					
					/* Copy the encoded data into the bit-stream struct */
					speex_bits_read_from(&decBits, (const char *)dec_in_buffer_1, ENCODED_FRAME_SIZE);
					/* Decode the data */
					speex_decode_int(dec_state, &decBits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer_2);
					for(i=0;i<FRAME_SIZE;i++){
						SPEEX_DEC_OUT_Buffer_2_U[i] = SPEEX_DEC_OUT_Buffer_2[i];
						SPEEX_DEC_OUT_Buffer_2_U[i] += 32768;
						SPEEX_DEC_OUT_Buffer_2_U[i] >>= 4;
					}
					NB_Frames++;
					//开始播放
					Playing_buf = 1;  //标记 先播放buf1中数据
					HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_1_U, FRAME_SIZE, DAC_ALIGN_12B_R);
					//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)SPEEX_DEC_OUT_Buffer_1, FRAME_SIZE*2);
					
				    /* Now we wait until the playing of the buffers to re-decode ...*/
					while(NB_Frames < ALL_FRAMES)
					{
					  if(Start_Decoding == 1) /* we start decoding the first buffer */
					  {
						for(i=0;i<ENCODED_FRAME_SIZE; i++)
						{
						  dec_in_buffer_1[i] = male_voice[sample_index++];
						}
						//HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET);
						/* Copy the encoded data into the bit-stream struct */
						speex_bits_read_from(&decBits, (const char *)dec_in_buffer_1, ENCODED_FRAME_SIZE);
						/* Decode the data */
						speex_decode_int(dec_state, &decBits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer_1);
						for(i=0;i<FRAME_SIZE;i++){
							SPEEX_DEC_OUT_Buffer_1_U[i] = SPEEX_DEC_OUT_Buffer_1[i];
						SPEEX_DEC_OUT_Buffer_1_U[i] += 32768;
						SPEEX_DEC_OUT_Buffer_1_U[i] >>= 4;
						}
						//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_1, FRAME_SIZE, DAC_ALIGN_8B_R);
						//HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET);
						Start_Decoding = 0;
						NB_Frames++;
					  }
					  if(Start_Decoding == 2) /* we start decoding the second buffer */
					  {
						for(i=0;i<ENCODED_FRAME_SIZE; i++)
						{
						  dec_in_buffer_1[i] = male_voice[sample_index++];
						}
						
						/* Copy the encoded data into the bit-stream struct */
						speex_bits_read_from(&decBits, (const char *)dec_in_buffer_1, ENCODED_FRAME_SIZE);
						/* Decode the data */
						speex_decode_int(dec_state, &decBits, (spx_int16_t*)SPEEX_DEC_OUT_Buffer_2);
						for(i=0;i<FRAME_SIZE;i++){
							SPEEX_DEC_OUT_Buffer_2_U[i] = SPEEX_DEC_OUT_Buffer_2[i];
						SPEEX_DEC_OUT_Buffer_2_U[i] += 32768;
						SPEEX_DEC_OUT_Buffer_2_U[i] >>= 4;
						}
						
						//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)SPEEX_DEC_OUT_Buffer_2, FRAME_SIZE, DAC_ALIGN_8B_R);
						Start_Decoding = 0;
						NB_Frames++;
					  }
					}
					Playing_buf =0 ;//stop UARTTRASMIT
					sample_index = 0;
					NB_Frames = 0;
					//播放完毕后开启实时ADC采集
					HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer, FRAME_SIZE*2);
					//HAL_UART_Transmit_DMA(&huart1, dest, 17);
				break;
				case 3:
					/* 临时进行CAN的发送测试 */
					/* 拼装canbus语音包 */
					/* 第1个语音包 */
					can_bus_buf[0] = CANBUS_SPEEX_ENCODED_PACKET_1;
					for(i=1;i<STD_CAN_BUF_LEN_MAX;i++){
						can_bus_buf[i] = 0x13 + i;
					}
					CAN1_Send_Msg(can_bus_buf,STD_CAN_BUF_LEN_MAX);
					/* 第2个语音包 */
					can_bus_buf[0] = CANBUS_SPEEX_ENCODED_PACKET_2;
					for(i=1;i<STD_CAN_BUF_LEN_MAX;i++){
						can_bus_buf[i] = 0x23 + i;
					}
					CAN1_Send_Msg(can_bus_buf,STD_CAN_BUF_LEN_MAX);
					/* 第3个语音包 */
					can_bus_buf[0] = CANBUS_SPEEX_ENCODED_PACKET_3;
					for(i=1;i<STD_CAN_BUF_LEN_MAX-1;i++){
						can_bus_buf[i] = 0x33 + i;
					}
					can_bus_buf[STD_CAN_BUF_LEN_MAX-1] = 0; //第3个语音包最后一个字节保留，暂定为0【编码后的语音一共20bytes】
					CAN1_Send_Msg(can_bus_buf,STD_CAN_BUF_LEN_MAX);
					//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)test_int16, 2*2);
				break;
			}
		//}
		
	}
  /* USER CODE END StartTaskUI */
}

/* StartTaskCanBusSend function */
void StartTaskCanBusSend(void const * argument)
{
  /* USER CODE BEGIN StartTaskCanBusSend */
	uint8_t i,j=0;
	/* Infinite loop */
	for(;;)
	{
		//osDelay(1);
		if(xSemaphoreTake( myBinarySemEncodCpltHandle, 100 )){	
			//HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
			HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_SET);
			j=0; //重置计数器
			/* 拼装canbus语音包 */
			/* 第1个语音包 */
			can_bus_buf[0] = CANBUS_SPEEX_ENCODED_PACKET_1;
			for(i=1;i<STD_CAN_BUF_LEN_MAX;i++){
				can_bus_buf[i] = can_bus_encoded_buf[j++];
			}
			CAN1_Send_Msg(can_bus_buf,STD_CAN_BUF_LEN_MAX);
			/* 第2个语音包 */
			can_bus_buf[0] = CANBUS_SPEEX_ENCODED_PACKET_2;
			for(i=1;i<STD_CAN_BUF_LEN_MAX;i++){
				can_bus_buf[i] = can_bus_encoded_buf[j++];
			}
			CAN1_Send_Msg(can_bus_buf,STD_CAN_BUF_LEN_MAX);
			/* 第3个语音包 */
			can_bus_buf[0] = CANBUS_SPEEX_ENCODED_PACKET_3;
			for(i=1;i<STD_CAN_BUF_LEN_MAX-1;i++){
				can_bus_buf[i] = can_bus_encoded_buf[j++];
			}
			can_bus_buf[STD_CAN_BUF_LEN_MAX-1] = 0; //第3个语音包最后一个字节保留，暂定为0【编码后的语音一共20bytes】
			CAN1_Send_Msg(can_bus_buf,STD_CAN_BUF_LEN_MAX);
			HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_RESET);
		}
	}
  /* USER CODE END StartTaskCanBusSend */
}

/* StartTaskCanRecvPreProcess function */
void StartTaskCanRecvPreProcess(void const * argument)
{
  /* USER CODE BEGIN StartTaskCanRecvPreProcess */
	uint8_t i=0,j=0;
	
  /* Infinite loop */
  for(;;)
  {
    if (xSemaphoreTake(myBinarySemCanRecvCpltHandle, 20)){	
		/* 语音包接收完成，启动解码流程 */
		//添加一级缓冲，为解码争取10ms【采样一帧时间决定】的处理时间。【不加缓冲的话只能和编码处在同一级10ms缓冲区中】
		
		if(dec_in_buffer_1_dec_busy){
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream2, (uint32_t)out_bytes, (uint32_t)dec_in_buffer_2, ENCODED_FRAME_SIZE);
		}
		else{
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream2, (uint32_t)out_bytes, (uint32_t)dec_in_buffer_1, ENCODED_FRAME_SIZE);
		}

	}
  }
  /* USER CODE END StartTaskCanRecvPreProcess */
}

/* USER CODE BEGIN Application */


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(hcan);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxCpltCallback could be implemented in the user file
   */
	//CAN_Receive_IT()函数会关闭FIFO0消息挂号中断，因此我们需要重新打开
    static uint8_t i,j=0;
	static uint8_t can_audio_step=0;
	static uint8_t can_audio_packet_cnt=0;  //canbus语音包计数器，计数到3则为一完整帧，可启动解码，否则重置接收
	__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_FMP0);//重新开启FIF00消息挂号中断	
	//HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
	//HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
	for(i=0;i<hcan1.pRxMsg->DLC;i++){
		can_recv_buf[i] = hcan1.pRxMsg->Data[i];
	}
	switch(can_recv_buf[0]){
		case CANBUS_SPEEX_ENCODED_PACKET_1:
			HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET);
			can_audio_step=0;
			can_audio_packet_cnt++; 
			for(i=1;i<hcan1.pRxMsg->DLC;i++){
				out_bytes[j++] = (char)can_recv_buf[i];
			}
			break;
		case CANBUS_SPEEX_ENCODED_PACKET_2:
			can_audio_step=1;
			can_audio_packet_cnt++; 
			for(i=1;i<hcan1.pRxMsg->DLC;i++){
				out_bytes[j++] = (char)can_recv_buf[i];
			}
			break;
		case CANBUS_SPEEX_ENCODED_PACKET_3:
			can_audio_step=2;
			can_audio_packet_cnt++; 
			for(i=1;i<hcan1.pRxMsg->DLC - 1;i++){
				out_bytes[j++] = (char)can_recv_buf[i];
			}
			
			if(can_audio_packet_cnt == 3 && j == 20){					
				/* 重置接收过程 */
				j=0;
				can_audio_step = 0;
				can_audio_packet_cnt = 0; 
				/* 语音包接收完成，启动解码流程 */
							//添加一级缓冲，为解码争取10ms【采样一帧时间决定】的处理时间。【不加缓冲的话只能和编码处在同一级10ms缓冲区中】
				HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET);
				xSemaphoreGiveFromISR(myBinarySemCanRecvCpltHandle, NULL);
			}
			else {
				/* 重置接收过程 */
				j=0;
				can_audio_step = 0;
				can_audio_packet_cnt = 0; 
			}
			break;
		/* other canbus command*/
		default: 
			/* 重置接收过程 */
			j=0;
			can_audio_step = 0;
			can_audio_packet_cnt = 0; 
			break;
	}

	
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
