/*
 * main.cpp
 *
 *  Created on: 2019年5月8日
 *      Author: d
 */
#include "EventManager.hpp"

int main()
{
	CEventParsing parsing;
	CEventManager* eventManager = new CEventManager();

	OSA_thrCreate(&parsing.jsEvent_thid, parsing.thread_jsEvent, 0, 0, NULL);
	OSA_thrCreate(&parsing.comrecvEvent_thid, parsing.thread_comrecvEvent, 0, 0, NULL);
	OSA_thrCreate(&parsing.comsendEvent_thid, parsing.thread_comsendEvent, 0, 0, NULL);
	OSA_thrCreate(&parsing.Getaccept_thid, parsing.thread_Getaccept, 0, 0, NULL);
	OSA_thrCreate(&parsing.ReclaimConnect_thid, parsing.thread_ReclaimConnect, 0, 0, NULL);
	OSA_thrCreate(&eventManager->ipcEvent_thid, eventManager->thread_ipcEvent, 0, 0, NULL);
		
	struct timeval tmp;
	while(1)
	{
		tmp.tv_sec = 100;
		tmp.tv_usec = 0;
		select(0, NULL, NULL, NULL, &tmp);
	}

	OSA_thrDelete(&parsing.jsEvent_thid);
	OSA_thrDelete(&parsing.comrecvEvent_thid);
	OSA_thrDelete(&parsing.comsendEvent_thid);
	OSA_thrDelete(&parsing.Getaccept_thid);
	OSA_thrDelete(&parsing.ReclaimConnect_thid);
	OSA_thrDelete(&eventManager->ipcEvent_thid);
	return 0;
}

