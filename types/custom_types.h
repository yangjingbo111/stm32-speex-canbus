#ifndef __CUSTOM_TYPES_H_
#define __CUSTOM_TYPES_H_

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define ALL_FRAMES      300   /* the encoded male voice length */

#define FRAME_SIZE 160
#define ENCODED_FRAME_SIZE      20
#define ADC_MAX 4095
#define DATA_WIDTH_MAX_HALF 0x3FFF	//数据宽度对应的最大值的一半

#define STD_CAN_BUF_LEN_MAX	8	//canbus标准数据包最大字节数
/* 

 */
 
 /* canbus语音包数据协议
	canbus标准数据包含有8个字节
	BYTE0		BYTE1		BYTE2		BYTE3		BYTE4		BYTE5		BYTE6		BYTE7
	
   ---------------------------------------------------------------------------*/
//语音包顺序
#define CANBUS_SPEEX_ENCODED_PACKET_1	1	//CANBUS第1个语音包
#define CANBUS_SPEEX_ENCODED_PACKET_2	2	//CANBUS第2个语音包
#define CANBUS_SPEEX_ENCODED_PACKET_3	3	//CANBUS第3个语音包

//命令
#define CANBUS_CMD_UNUSED				4	//CANBUS命令

#endif

