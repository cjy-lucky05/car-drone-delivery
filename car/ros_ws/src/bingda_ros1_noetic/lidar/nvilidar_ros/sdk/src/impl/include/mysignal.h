#pragma once 

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <inttypes.h>
#if defined(_WIN32)
	#include <mmsystem.h>
	#include <sysinfoapi.h>
	#include <WinSock2.h>
	#include <windows.h>
#else 
	#include <iostream>
	#include <signal.h>
#endif 

namespace nvilidar
{
	bool nvilidar_running_state = true;

#if defined(_WIN32)

	BOOL WINAPI  consoleHandler(DWORD signal)
	{
		if (signal == CTRL_C_EVENT)
		{
			//printf("key press!");
			nvilidar_running_state = false;
		}
		return true;
	}

	inline void sigInit(void)
	{
		//printf("\nERROR: Could not set control handler");
		if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
			//printf("\nERROR: Could not set control handler");
			return;
		}
		//printf("signal init OK");
	}

	inline bool isOK()
	{
		return nvilidar_running_state;
	}
#else 
	void consoleHandler( int signal)
	{
		if (signal == SIGINT)
		{
			//printf("key press!");
			nvilidar_running_state = false;
		}
	}

	inline void sigInit(void)
	{
		//printf("\nERROR: Could not set control handler");
		signal(SIGINT,consoleHandler);
		//printf("signal init OK");
	}

	inline bool isOK()
	{
		return nvilidar_running_state;
	}
#endif 

}
