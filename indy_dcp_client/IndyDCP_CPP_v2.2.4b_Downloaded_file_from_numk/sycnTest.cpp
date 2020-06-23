#if 0

#include "IndyDCPConnector.h"

#include <stdio.h>
#include <signal.h>
#if defined (LINUX)
#include <unistd.h>
#elif defined (WINDOWS)
#pragma comment(lib,"ws2_32")	//Must Link Winsock Library "ws2_32.dll"
#endif

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <future>

using namespace std;
using namespace NRMKIndy::Service::DCP;

IndyDCPConnector connector1("192.168.0.72", ROBOT_INDY7);
IndyDCPConnector connector2("192.168.0.45", ROBOT_INDY7);

bool __sys_cleaned = false;
void system_cleanup(int sig=0)
{
	if (__sys_cleaned) return;
	__sys_cleaned = true;

	connector1.disconnect();
	connector2.disconnect();

#if defined(WINDOWS)
	cleanupWinSock();
#endif
}

void startProgram(IndyDCPConnector* connector)
{
	cout << "Start Reg-Default Program!" << endl;
	connector->startRegisteredDefaultProgram();
}

void stopProgram(IndyDCPConnector* connector)
{
	cout << "Stop Program!" << endl;
	connector->stopProgram();
}

void pauseProgram(IndyDCPConnector* connector)
{
	cout << "Pause Program!" << endl;
	connector->pauseProgram();
}

void resumeProgram(IndyDCPConnector* connector)
{
	cout << "Resume Program!" << endl;
	connector->resumeProgram();
}

void reset(IndyDCPConnector* connector)
{
	cout << "Reset Program!" << endl;
	connector->resetRobot();
}

void moveHome(IndyDCPConnector* connector)
{
	cout << "Move Home!" << endl;
	connector->moveJointHome();
}

void moveZero(IndyDCPConnector* connector)
{
	cout << "Move Zero!" << endl;
	connector->moveJointZero();
}

void stop(IndyDCPConnector* connector)
{
	cout << "Stop Move!" << endl;
	connector->stopMotion();
}

void emergencyStop(IndyDCPConnector* connector)
{
	cout << "Emergency Stop!" << endl;
	connector->stopEmergency();
}

void called_from_async(int a)
{
	std::cout << "Async call " << a << std::endl;
}

int main(int argc, char* argv[])
{
	signal(SIGTERM, system_cleanup);	//by pkill, kill
	signal(SIGINT, system_cleanup);
	signal(SIGABRT, system_cleanup);	//by aborted core dumped
#if defined (LINUX) || defined (UNIX)
	signal(SIGQUIT, system_cleanup);
	signal(SIGKILL, system_cleanup);
#endif

#if defined(WINDOWS)
	initWinSock();
#endif

#if 0
//	std::async(called_from_async, 33);
	future<void> result(async(called_from_async, 33));
	result.get();
	printf("Main!\n");
	usleep(1e5);

	future<void> result2(async(startProgram, &connector1));

#else
	if (connector1.connect() && connector2.connect())
	{
		char inChar;

		while(!__sys_cleaned) {
			cout << "Enter KeyCommand: ";
			cin >> inChar;

			switch (inChar) {
			case 'e':
			{
				future<void> result1(async(emergencyStop, &connector1));
				future<void> result2(async(emergencyStop, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'r':
			{
				future<void> result1(async(reset, &connector1));
				future<void> result2(async(reset, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'q':
			{
				future<void> result1(async(startProgram, &connector1));
				future<void> result2(async(startProgram, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'w':
			{
				future<void> result1(async(stopProgram, &connector1));
				future<void> result2(async(stopProgram, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'a':
			{
				future<void> result1(async(pauseProgram, &connector1));
				future<void> result2(async(pauseProgram, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 's':
			{
				future<void> result1(async(resumeProgram, &connector1));
				future<void> result2(async(resumeProgram, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'c':
			{
				future<void> result1(async(stop, &connector1));
				future<void> result2(async(stop, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'x':
			{
				future<void> result1(async(moveHome, &connector1));
				future<void> result2(async(moveHome, &connector2));
				result1.get();
				result2.get();
			}
			break;

			case 'z':
			{
				future<void> result1(async(moveZero, &connector1));
				future<void> result2(async(moveZero, &connector2));
				result1.get();
				result2.get();
			}
				break;

			default:
				cout << "Unknown Command" << endl;
				break;
			}

			//usleep(1e5);	//100ms
		}

		connector1.disconnect();
		connector2.disconnect();
	}
	else
	{
		cout << "Connection failed.\n" << endl;
	}

	system_cleanup();
#endif
	return 0;
}

#endif
