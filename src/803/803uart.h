/*
 * 803uart.h
 *
 *  Created on: 2019年7月31日
 *      Author: alex
 */

#ifndef FILE803UART_H_
#define FILE803UART_H_

#include "PortFactory.hpp"
#include "osa_mutex.h"
#include <vector>

class C803COM
{
public:
	C803COM();
	~C803COM();
	
	void createPort();
	void sendtrkerr(int chid,int status,float errx,float erry,int rendercount);
	void getExtcmd();


protected:
	void packageSendbaseData();
	void findValidData(unsigned char *tmpRcvBuff, int sizeRcv);
	void parsing();
	unsigned int recvcheck_sum();
	void calcCheckNum();


private:
	CPortInterface* pCom1,*pCom2;
	int com1fd,com2fd;
	unsigned char m_senddata[12];
	unsigned char m_recvdata[1024];

	bool existRecvThread;
	OSA_MutexHndl m_com1mutex;
	std::vector<unsigned char>  m_rcvBuf;
	int m_cmdlength;
};




#endif /* 803UART_H_ */
