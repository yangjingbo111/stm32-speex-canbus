#ifndef __CUSTOM_TYPES_H_
#define __CUSTOM_TYPES_H_

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define ALL_FRAMES      300   /* the encoded male voice length */

#define FRAME_SIZE 160
#define ENCODED_FRAME_SIZE      20
#define ADC_MAX 4095
#define DATA_WIDTH_MAX_HALF 0x3FFF	//���ݿ�ȶ�Ӧ�����ֵ��һ��

#define STD_CAN_BUF_LEN_MAX	8	//canbus��׼���ݰ�����ֽ���
/* 

 */
 
 /* canbus����������Э��
	canbus��׼���ݰ�����8���ֽ�
	BYTE0		BYTE1		BYTE2		BYTE3		BYTE4		BYTE5		BYTE6		BYTE7
	
   ---------------------------------------------------------------------------*/
//������˳��
#define CANBUS_SPEEX_ENCODED_PACKET_1	1	//CANBUS��1��������
#define CANBUS_SPEEX_ENCODED_PACKET_2	2	//CANBUS��2��������
#define CANBUS_SPEEX_ENCODED_PACKET_3	3	//CANBUS��3��������

//����
#define CANBUS_CMD_UNUSED				4	//CANBUS����

#endif

