/*
 * 803uart.cpp
 *
 *  Created on: 2019年7月31日
 *      Author: alex
 */

#include "803uart.h"
#include <math.h>


IPC_PRM_INT gIpcParam;

static C803COM* gThis;
	
C803COM::C803COM(sendIpcMsgCallback pfunc):pCom1(NULL),pCom2(NULL),existRecvThread(false),m_cmdlength(8)
{
	memset(m_senddata,0,sizeof(m_senddata));
	memset(m_recvdata,0,sizeof(m_recvdata));
	OSA_mutexCreate(&m_com1mutex);
	m_rcvBuf.clear();
	gThis = this;
	pFunc_SendIpc = pfunc;
}

C803COM::~C803COM()
{
	if(pCom1 != NULL)
	{
		pCom1->cclose();
		delete pCom1;
	}

	if(pCom2 != NULL)
	{
		pCom1->cclose();
		delete pCom1;
	}
	
	OSA_mutexDelete(&m_com1mutex);
}


void C803COM::createPort()
{
	pCom1 = PortFactory::createProduct(1);
	pCom2 = PortFactory::createProduct(3);

	if(pCom1 != NULL)
		com1fd = pCom1->copen();
	if(pCom2 != NULL)
		com2fd = pCom2->copen();

	packageSendbaseData();

	return;
}

void C803COM::sendtrkerr(int chid,int status,float errx,float erry,int rendercount)
{
	int x,y;
	memset(m_senddata,0,sizeof(m_senddata));
	
	m_senddata[2] = status;

	x = (int)round(errx);	
	m_senddata[3] = (x>>8)&(0xff);
	m_senddata[4] = x&(0xff);

	y = (int)round(erry);
	m_senddata[5] = (y>>8)&(0xff);
	m_senddata[6] = y&(0xff);

	m_senddata[7] = chid;
	m_senddata[8] = rendercount;
	calcCheckNum();

	OSA_mutexLock(&m_com1mutex);
	pCom1->csend(com1fd, m_senddata, sizeof(m_senddata));
	OSA_mutexUnlock(&m_com1mutex);
	
	return;	
}

void C803COM::packageSendbaseData()
{
	m_senddata[0] = 0x55;
	m_senddata[1] = 0xAA;
	m_senddata[10] = 0x0D;
	m_senddata[11] = 0x0A;
	return;
}

void C803COM::calcCheckNum()
{
	int sum = 0;
	for(int i=3;i<=9;i++)
		sum += m_senddata[i-1];

	m_senddata[9] = sum&(0xff);
	return;	
}

void* C803COM::runUpExtcmd(void *)
{
	gThis->getExtcmd();
	return NULL;
}

void C803COM::getExtcmd()
{
	int sizercv;
	while(existRecvThread == false)
	{
		sizercv = pCom2->crecv(com2fd, m_recvdata,sizeof(m_recvdata));
		findValidData(m_recvdata,sizercv);
	}
}

void C803COM::findValidData(unsigned char *tmpRcvBuff, int sizeRcv)
{
	unsigned int uartdata_pos = 0;
	unsigned char frame_head[]={0xAA, 0x55};

	static struct data_buf
	{
		unsigned int len;
		unsigned int pos;
		unsigned char reading;
		unsigned char buf[1024];
	}swap_data = {0, 0, 0,{0}};

	if(sizeRcv>0)
	{
		for(int j=0;j<sizeRcv;j++)
		{
			printf("%02x ",tmpRcvBuff[j]);
		}
		printf("\n");

		while (uartdata_pos < sizeRcv)
		{
			if((0 == swap_data.reading) || (2 == swap_data.reading))
			{
				if(frame_head[swap_data.len] == tmpRcvBuff[uartdata_pos])
				{
					swap_data.buf[swap_data.pos++] =  tmpRcvBuff[uartdata_pos++];
					swap_data.len++;
					swap_data.reading = 2;
					if(swap_data.len == sizeof(frame_head)/sizeof(char))
							swap_data.reading = 1;
				}
				else
				{
					uartdata_pos++;
					if(2 == swap_data.reading)
						memset(&swap_data, 0, sizeof(struct data_buf));
				}
			}
			else if(1 == swap_data.reading)
			{
				swap_data.buf[swap_data.pos++] = tmpRcvBuff[uartdata_pos++];
				swap_data.len++;
				if(swap_data.len == m_cmdlength)
				{
					for(int i=0;i<swap_data.len;i++)
					{
						m_rcvBuf.push_back(swap_data.buf[i]);
					}
					parsing();
					memset(&swap_data, 0, sizeof(struct data_buf));					
				}
			}
		}
	}
	return;
}

unsigned int C803COM::recvcheck_sum()
{
	unsigned int sum = 0;
	for(int i=2;i<=5;i++)
		sum += m_rcvBuf[i-1];
	return sum;
}


void C803COM::parsing()
{
	int ret =  -1;
	
	if(m_rcvBuf.size()<m_cmdlength)
		return;

	
	unsigned char checkSum = recvcheck_sum();

	if( ((checkSum>>8)&0xff) == m_rcvBuf[5] && ((checkSum&0xff) == m_rcvBuf[6]))
	{
		if(m_rcvBuf[2] != 0x0)
		{
			if(m_rcvBuf[2] == 0x2)
			{
				gIpcParam.intPrm[0] = 1; //up
				pFunc_SendIpc(mtdSelect, gIpcParam.intPrm, 4);
			}
			else if(m_rcvBuf[2] == 0x3)
			{
				gIpcParam.intPrm[0] = 2; //next
				pFunc_SendIpc(mtdSelect, gIpcParam.intPrm, 4);
			}
		}

		if(m_rcvBuf[3] != 0x0)
		{
			if(m_rcvBuf[3] == 0x2)
			{
				gIpcParam.intPrm[0] = 2;
				pFunc_SendIpc(switchPip, gIpcParam.intPrm, 4);	
			}
			else if(m_rcvBuf[3] == 0x3)
			{
				gIpcParam.intPrm[0] = 3;
				pFunc_SendIpc(switchPip, gIpcParam.intPrm, 4);
			}
		}

		//if(m_rcvBuf[4] != 0x0)
		{
			if(m_rcvBuf[4] == 0x1)
			{
				//printf("enable trk\n");
				gIpcParam.intPrm[0] = 1;
				pFunc_SendIpc(trk,gIpcParam.intPrm,4);
			}
			else if(m_rcvBuf[4] == 0x0)
			{
				//printf("disable trk\n");
				gIpcParam.intPrm[0] = 0;
				pFunc_SendIpc(trk,gIpcParam.intPrm,4);			
			}		
		}

		if(m_rcvBuf[5] != 0x0)
		{
			if(m_rcvBuf[5] == 0x2)
			{
				gIpcParam.intPrm[0] = 0;
				pFunc_SendIpc(mtd, gIpcParam.intPrm, 4);			
				pFunc_SendIpc(stb, gIpcParam.intPrm, 4);			
			}	
			else if(m_rcvBuf[5] == 0x3)
			{
				gIpcParam.intPrm[0] = 1;
				pFunc_SendIpc(mtd, gIpcParam.intPrm, 4);			
			}
			else if(m_rcvBuf[5] == 0x4)
			{
				gIpcParam.intPrm[0] = 1;
				pFunc_SendIpc(stb, gIpcParam.intPrm, 4);			
			}
		}
	}
	else
		printf("%s,%d, checksum error:,cal is %02x,recv is %02x\n",__FILE__,__LINE__,checkSum,((m_rcvBuf[5]<<8)|m_rcvBuf[6]));
	
	m_rcvBuf.erase(m_rcvBuf.begin(),m_rcvBuf.begin()+m_cmdlength);
	return ;
}


