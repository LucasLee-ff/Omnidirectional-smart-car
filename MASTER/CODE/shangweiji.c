#include "shangweiji.h"
#include "headfile.h"

void CRC16(uint8 *Array,uint8 *Rcvbuf,unsigned int Len )  //产生校验码
{
	uint16 IX,IY,CRCa;
	CRCa=0xFFFF;
	
	if(Len<=0)
		CRCa=0;
	else
	{
		Len--;
		for(IX=0;IX<=Len;IX++)
		{
 			CRCa=CRCa^Array[IX];
			for(IY=0;IY<=7;IY++)
			{
			if((CRCa&1)!=0) CRCa=(CRCa>>1)^0xA001;
			else CRCa=CRCa>>1;
			}
		}
	}
	Rcvbuf[1]=(CRCa&0xFF00)>>8; //高位置
	Rcvbuf[0]=(CRCa&0x00FF); //低位置
}

void OutPut_Data(int ch1,int ch2,int ch3,int ch4)
{
	uint8 Array[8],Rcvbuf[2];
	Array[0]=(int8)(ch1&0x00FF);
	Array[1]=(int8)(ch1&0xFF00);
	Array[2]=(int8)(ch2&0x00FF);
	Array[3]=(int8)(ch2&0xFF00);
	Array[4]=(int8)(ch3&0x00FF);
	Array[5]=(int8)(ch3&0xFF00);
	Array[6]=(int8)(ch4&0x00FF);
	Array[7]=(int8)(ch4&0xFF00);
	CRC16(Array,Rcvbuf,8);
	/*printf("Array[0]:%d\r\n",Array[0]);
	printf("Array[1]:%d\r\n",Array[1]);
	printf("Array[2]:%d\r\n",Array[2]);
	printf("Array[3]:%d\r\n",Array[3]);
	printf("Array[4]:%d\r\n",Array[4]);
	printf("Array[5]:%d\r\n",Array[5]);
	printf("Array[6]:%d\r\n",Array[6]);
	printf("Array[7]:%d\r\n",Array[7]);
	printf("Rcvbuf[0]:%d\r\n",Rcvbuf[0]);
	printf("Rcvbuf[1]:%d\r\n",Rcvbuf[1]);*/
	uart_putbuff(UART_1, Array, 8); //发送4个通道的数据
	uart_putbuff(UART_1, Rcvbuf, 2);// 发送校验码
}
