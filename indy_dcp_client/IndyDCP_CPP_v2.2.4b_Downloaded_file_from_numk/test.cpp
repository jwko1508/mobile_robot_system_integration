#if 1

#include "IndyDCPConnector.h"

#include <stdio.h>
#include <signal.h>
#if defined (LINUX)
#include <unistd.h>
#elif defined (WINDOWS)
#pragma comment(lib,"ws2_32")	//Must Link Winsock Library "ws2_32.dll"
#endif

#include <iostream>


using namespace std;
using namespace NRMKIndy::Service::DCP;

//IndyDCPConnector connector("127.0.0.1", ROBOT_INDY7);
IndyDCPConnector connector("192.168.0.72", ROBOT_INDY7);

bool __sys_cleaned = false;
void system_cleanup(int sig=0)
{
	if (__sys_cleaned) return;
	__sys_cleaned = true;

#if defined(WINDOWS)
	cleanupWinSock();
#endif
	exit(0);
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

	if (connector.connect())
	{
		char inChar;

		cout << "Enter Test Type (1) for AutoTest, (2) for KeyCommand: ";
		cin >> inChar;

		if (inChar == '2')	//KeyCommand
		{
			while(1)
			{
				cout << "Enter KeyCommand: ";
				cin >> inChar;

				switch (inChar)
				{
				case 27:	//esc
					connector.disconnect();
					break;

				case '1':
				{
					cout << "MoveJoint, Ret=" << connector.moveJointTo({0, -15, -90, 0, -75, 0}) << endl;
	//				double q[6] = {0, -15, -90, 0, -75, 0};
	//				cout << "MoveJoint, Ret=" << connector.moveJointTo(q) << endl;
				}
					break;

				case '2':
				{
					connector.moveJointWaypointSet({
						0, -15, -90, 0, -75, 0,
						0, 0, 0, 0, 0, 0,
						0, 15, 90, 0, 75, 0
					});
				}
					break;

				case 'h':
				{
					cout << "MoveHome, Ret=" << connector.moveJointHome() << endl;
				}
					break;

				case 'z':
				{
					cout << "MoveZero, Ret=" << connector.moveJointZero() << endl;
				}
					break;

				case 'q':
				{
					double q[6];
					cout << "Get Q, Ret=" << connector.getJointPosition(q) << endl;
					printf("Q: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
							q[0], q[1], q[2], q[3], q[4], q[5]);
				}
					break;

				case 'd':
				{
					connector.startCurrProgram();
				}
					break;

				case 'f':
				{
					connector.startRegisteredDefaultProgram();
				}
					break;

				case 's':
				{
					cout << "StopMotion, Ret=" << connector.stopMotion() << endl;
				}
					break;


				case '[':
				{
					bool servo[6] = {1, 1, 1, 1, 1, 1};
					cout << "Servo all on, Ret=" << connector.setServoOnOff(servo) << endl;
	//				cout << "Servo all on, Ret=" << connector.setServoOnOff({1, 1, 1, 1, 1, 1}) << endl;
				}
					break;

				case ']':
				{
					cout << "Servo all off, Ret=" << connector.setServoOnOff({0, 0, 0, 0, 0, 0}) << endl;
				}
					break;

				case 'v':	//Direct Variable test
				{
					uint8_t byteVars[20] = { 1, 2, 3, 4};
					int16_t wordVars[20] = {10, 20, 30, 40};
					int32_t dwordVars[20] = {100, 200, 300, 400};
					int64_t lwordVars[20] = {1000, 2000, 3000, 4000};
					float floatVars[20] = {1.1, 2.2, 3.3, 4.4};
					double doubleVars[20] = {0.01, 0.02, 0.03, 0.04};
					uint16_t uwordVars[20] = {11, 22, 33, 44};

					connector.writeDirectVariables(DIRECT_VAR_TYPE_BYTE, 2, 4, byteVars);
					connector.writeDirectVariables(DIRECT_VAR_TYPE_WORD, 2, 4, wordVars);
					connector.writeDirectVariables(DIRECT_VAR_TYPE_DWORD, 2, 4, dwordVars);
					connector.writeDirectVariables(DIRECT_VAR_TYPE_LWORD, 2, 4, lwordVars);
					connector.writeDirectVariables(DIRECT_VAR_TYPE_FLOAT, 2, 4, floatVars);
					connector.writeDirectVariables(DIRECT_VAR_TYPE_DFLOAT, 2, 4, doubleVars);
					connector.writeDirectVariables(DIRECT_VAR_TYPE_MODBUS_REG, 2, 4, uwordVars);

					if (connector.readDirectVariable(DIRECT_VAR_TYPE_BYTE, 2, &byteVars[10]) == ERR_NONE)
					{
						cout << "B002 = " << (int)byteVars[10] <<endl;
					}
					if (connector.readDirectVariable(DIRECT_VAR_TYPE_WORD, 3, &wordVars[10]) == ERR_NONE)
					{
						cout << "W003 = " << wordVars[10] <<endl;
					}
					if (connector.readDirectVariable(DIRECT_VAR_TYPE_DWORD, 4, &dwordVars[10]) == ERR_NONE)
					{
						cout << "I004 = " << dwordVars[10] <<endl;
					}
					if (connector.readDirectVariable(DIRECT_VAR_TYPE_MODBUS_REG, 2, &uwordVars[10]) == ERR_NONE)
					{
						cout << "M002 = " << uwordVars[10] <<endl;
					}

					int32_t dw = -99;
					double df = 99.999;
					uint16_t dm = 999;
					connector.writeDirectVariable(DIRECT_VAR_TYPE_DWORD, 0, &dw);
					connector.writeDirectVariable(DIRECT_VAR_TYPE_DFLOAT, 0, &df);
					connector.writeDirectVariable(DIRECT_VAR_TYPE_MODBUS_REG, 0, &dm);

					if (connector.readDirectVariables(DIRECT_VAR_TYPE_BYTE, 0, 10, byteVars) == ERR_NONE)
					{
						cout << "B000~B009 = ";
						for (int i=0; i<10; i++) cout << (int)byteVars[i] << ", ";
						cout << endl;
					}
					if (connector.readDirectVariables(DIRECT_VAR_TYPE_WORD, 0, 10, wordVars) == ERR_NONE)
					{
						cout << "W000~W009 = ";
						for (int i=0; i<10; i++) cout << wordVars[i] << ", ";
						cout << endl;
					}
					if (connector.readDirectVariables(DIRECT_VAR_TYPE_DWORD, 0, 10, dwordVars) == ERR_NONE)
					{
						cout << "I000~I009 = ";
						for (int i=0; i<10; i++) cout << dwordVars[i] << ", ";
						cout << endl;
					}
					if (connector.readDirectVariables(DIRECT_VAR_TYPE_LWORD, 0, 10, lwordVars) == ERR_NONE)
					{
						cout << "L000~L009 = ";
						for (int i=0; i<10; i++) cout << lwordVars[i] << ", ";
						cout << endl;
					}
					if (connector.readDirectVariables(DIRECT_VAR_TYPE_FLOAT, 0, 10, floatVars) == ERR_NONE)
					{
						cout << "F000~F009 = ";
						for (int i=0; i<10; i++) cout << floatVars[i] << ", ";
						cout << endl;
					}
					if (connector.readDirectVariables(DIRECT_VAR_TYPE_DFLOAT, 0, 10, doubleVars) == ERR_NONE)
					{
						cout << "D000~D009 = ";
						for (int i=0; i<10; i++) cout << doubleVars[i] << ", ";
						cout << endl;
					}
					if (connector.readDirectVariables(DIRECT_VAR_TYPE_MODBUS_REG, 0, 10, uwordVars) == ERR_NONE)
					{
						cout << "M000~M009 = ";
						for (int i=0; i<10; i++) cout << uwordVars[i] << ", ";
						cout << endl;
					}


//					if (connector.readDirectVariable(DIRECT_VAR_TYPE_BYTE, 0, byteVars) == ERR_NONE)
//						cout << "B000=" << connector.readDirectVariable(DIRECT_VAR_TYPE_BYTE, 0,) <<endl;
				}
					break;

				}
			}
		}

		else if (inChar == '3') //Utility Test (for Debugging)
		{
			cout << "Utility uSleep Test for Each OS!" << endl;
			for (int i = 0; i < 10; i++)
			{
				uSleep(1e6);	//1sec
				cout << i + 1 << " Seconds Later ..." << endl;
			}
		}

		else if (inChar == '1')	//Auto - Only for Indy7
		{
			using namespace NRMKIndy::Service::DCP;

			bool testRetBoolArr[32];
			char testRetCharArr[32];
			int testRetIntArr[12];
			double testRetDoubleArr[12];
			bool & testRetBool = testRetBoolArr[0];
			char & testRetChar = testRetCharArr[0];
			int & testRetInt = testRetIntArr[0];
			double & testRetDouble = testRetDoubleArr[0];
			std::string testString = "aaa";


			int testCmd = 0 /*startCmd*/	-1;
			int testCode = ERR_NONE;

			enum RET_TYPE
			{
				RET_BOOL = 1,
				RET_CHAR = 2,
				RET_INT = 3,
				RET_DOUBLE = 4,
			};

			while (testCode == ERR_NONE && testCmd<999)
			{
				int retType = -1;
				int retLen;
				testCmd++;
				testCode = -1;

				switch(testCmd)
				{
				case CMD_CHECK:
					testCode = connector.check();
					break;

				case CMD_EMERGENCY_STOP:
					testCode = connector.stopEmergency();
					uSleep(1e6);
					break;

				case CMD_RESET_ROBOT:
					testCode = connector.resetRobot();
					uSleep(6e6);
					break;

				case CMD_SET_SERVO:
					testCode = connector.setServoOnOff({1,1,1,1,1,1});
					break;

				case CMD_STOP:
					testCode = connector.stopMotion();
					uSleep(1e5);
					break;

				case CMD_MOVE:
					testCode = connector.executeMoveCommand("aaa");
					uSleep(10e6);
					break;

				case CMD_MOVE_HOME:
					testCode = connector.moveJointHome();
					uSleep(3e6);
					break;

				case CMD_MOVE_ZERO:
					testCode = connector.moveJointZero();
					uSleep(3e6);
					break;

				case CMD_JOINT_MOVE_TO:
					testCode = connector.moveJointTo({0, 15, 90, 0, 75, 0});
					uSleep(3e6);
					break;

				case CMD_JOINT_MOVE_BY:
					testCode = connector.moveJointBy({30, 0, 0, 0, 0, 0});
					uSleep(3e6);
					break;

				case CMD_TASK_MOVE_TO:
					testCode = connector.moveTaskTo({-0.3, -0.4, 0.4, 0, -180, 30});
					uSleep(3e6);
					break;

				case CMD_TASK_MOVE_BY:
					testCode = connector.moveTaskBy({0, 0.2, 0, 0, 0, 15});
					uSleep(3e6);
					break;

				case CMD_START_CURRENT_PROGRAM:
					testCode = connector.startCurrProgram();
					uSleep(4e6);
					break;

				case CMD_PAUSE_CURRENT_PROGRAM:
					testCode = connector.pauseProgram();
					uSleep(2e6);
					break;

				case CMD_RESUME_CURRENT_PROGRAM:
					testCode = connector.resumeProgram();
					uSleep(2e6);
					break;

				case CMD_STOP_CURRENT_PROGRAM:
					testCode = connector.stopProgram();
					uSleep(1e6);
					break;

				case CMD_START_DEFAULT_PROGRAM:
					testCode = connector.startRegisteredDefaultProgram();
					uSleep(8e6);
					break;

				case CMD_REGISTER_DEFAULT_PROGRAM_IDX:
					testCode = connector.registerDefaultProgram(7);
					uSleep(10e6);
					break;

				case CMD_GET_REGISTERED_DEFAULT_PROGRAM_IDX:
					testCode = connector.getRegisteredDefaultProgram(testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_IS_ROBOT_RUNNING:
					testCode = connector.isRobotRunning(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_READY:
					testCode = connector.isRobotReady(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_EMG:
					testCode = connector.isEmergencyStopped(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_COLLIDED:
					testCode = connector.isCollided(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_ERR:
					testCode = connector.isErrorState(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_BUSY:
					testCode = connector.isBusy(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_MOVE_FINISEHD:
					testCode = connector.isMoveFinished(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_HOME:
					testCode = connector.isHome(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_ZERO:
					testCode = connector.isZero(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_IN_RESETTING:
					testCode = connector.isInResetting(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_DIRECT_TECAHING:
					testCode = connector.isDirectTeachingMode(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_TEACHING:
					testCode = connector.isTeachingMode(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_PROGRAM_RUNNING:
					testCode = connector.isProgramRunning(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_PROGRAM_PAUSED:
					testCode = connector.isProgramPaused(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_IS_CONTY_CONNECTED:
					testCode = connector.isContyConnected(testRetBool);
					retType = RET_BOOL;
					retLen = 1;
					break;

				case CMD_CHANGE_DIRECT_TEACHING:
					testCode = connector.changeToDirectTeachingMode();
					uSleep(1e6);
					break;

				case CMD_FINISH_DIRECT_TEACHING:
					testCode = connector.finishDirectTeachingMode();
					uSleep(1e6);
					break;

				case CMD_JOINT_PUSH_BACK_WAYPOINT_SET:
					testCode = connector.addJointWaypointSet({30, 40, 50, 60, 50, 40});
					if (testCode!=ERR_NONE) { testCode += 10000; break; }
					testCode = connector.addJointWaypointSet({90, 90, 30, 40, 20, 80});
					if (testCode!=ERR_NONE) { testCode += 20000; break; }
					testCode = connector.addJointWaypointSet({30, 40, 50, 60, 50, 40});
					if (testCode!=ERR_NONE) { testCode += 30000; break; }
					testCode = connector.addJointWaypointSet({90, 90, 30, 40, 20, 80});
					if (testCode!=ERR_NONE) { testCode += 40000; break; }
					testCode = connector.removeJointWaypointSet();
					if (testCode!=ERR_NONE) { testCode += 50000; break; }
					testCode = connector.executeJointWaypointSet();
					if (testCode!=ERR_NONE) { testCode += 60000; break; }
					uSleep(10e6);
					testCode = connector.clearJointWaypointSet();
					break;

				case CMD_SET_DEFAULT_TCP:
					testCode = connector.setDefaultTCP({0, 0.1, 0.2, 0, 0, 0});
					break;

				case CMD_SET_COMP_TCP:
					testCode = connector.setTCPCompensation({0, 0.1, 0.2, 0, 0, 0});
					break;

				case CMD_RESET_COMP_TCP:
					testCode = connector.resetTCPCompensation();
					break;

				case CMD_SET_REFFRAME:
					testCode = connector.setReferenceFrame({0.2, -0.5, 0.2, 0, 180, 0});
					break;

				case CMD_RESET_REFFRAME:
					testCode = connector.resetReferenceFrame();
					break;
					break;

				case CMD_SET_COLLISION_LEVEL:
					testCode = connector.setCollisionDetectionLevel(2);
					break;

				case CMD_SET_JOINT_BOUNDARY:
					testCode = connector.setJointBoundaryLevel(8);
					break;

				case CMD_SET_TASK_BOUNDARY:
					testCode = connector.setTaskBoundaryLevel(6);
					break;

				case CMD_SET_JOINT_WTIME:
					testCode = connector.setJointWaypointTime(4.0);
					break;

				case CMD_SET_TASK_WTIME:
					testCode = connector.setTaskWaypointTime(2.5);
					break;

				case CMD_SET_TASK_CMODE:
					testCode = connector.setTaskBaseMode(0);
					break;

				case CMD_SET_JOINT_BLEND_RADIUS:
					testCode = connector.setJointBlendingRadius(10);
					break;

				case CMD_SET_TASK_BLEND_RADIUS:
					testCode = connector.setTaskBlendingRadius(0.12);
					break;

				case CMD_GET_DEFAULT_TCP:
					testCode = connector.getDefaultTCP(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_COMP_TCP:
					testCode = connector.getTCPCompensation(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_REFFRAME:
					testCode = connector.getReferenceFrame(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_COLLISION_LEVEL:
					testCode = connector.getCollisionDetectionLevel(testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_GET_JOINT_BOUNDARY:
					testCode = connector.getJointBoundaryLevel(testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_GET_TASK_BOUNDARY:
					testCode = connector.getTaskBoundaryLevel(testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_GET_JOINT_WTIME:
					testCode = connector.getJointWaypointTime(testRetDouble);
					retType = RET_DOUBLE;
					retLen = 1;
					break;

				case CMD_GET_TASK_WTIME:
					testCode = connector.getTaskWaypointTime(testRetDouble);
					retType = RET_DOUBLE;
					retLen = 1;
					break;

				case CMD_GET_TASK_CMODE:
					testCode = connector.getTaskBaseMode(testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_GET_JOINT_BLEND_RADIUS:
					testCode = connector.getJointBlendingRadius(testRetDouble);
					retType = RET_DOUBLE;
					retLen = 1;
					break;

				case CMD_GET_TASK_BLEND_RADIUS:
					testCode = connector.getTaskBlendingRadius(testRetDouble);
					retType = RET_DOUBLE;
					retLen = 1;
					break;

				case CMD_GET_RUNNING_TIME:
					testCode = connector.getRobotRunningTime(testRetDouble);
					retType = RET_DOUBLE;
					retLen = 1;
					break;

				case CMD_GET_CMODE:
					testCode = connector.getControlMode(testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_GET_JOINT_STATE:
					testCode = connector.getJointServoState(testRetCharArr);
					retType = RET_CHAR;
					retLen = 12;
					break;

				case CMD_GET_JOINT_POSITION:
					testCode = connector.getJointPosition(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_JOINT_VELOCITY:
					testCode = connector.getJointVelocity(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_TASK_POSITION:
					testCode = connector.getTaskPosition(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_TASK_VELOCITY:
					testCode = connector.getTaskVelocity(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_TORQUE:
					testCode = connector.getTorque(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_LAST_EMG_INFO:
					testCode = connector.getLastEmergencyInfo(testRetIntArr[0], &testRetIntArr[1], testRetDoubleArr);
					printf("\t-EmgData = [%d] %d, %d, %d, %.3f, %.3f, %.3f\n",
							testRetIntArr[0], testRetIntArr[1], testRetIntArr[2], testRetIntArr[3],
							testRetDoubleArr[0], testRetDoubleArr[1], testRetDoubleArr[2]);
					break;

				case CMD_GET_SMART_DI:
					testCode = connector.getSmartDigitalInput(20, testRetChar);
					retType = RET_CHAR;
					retLen = 1;
					break;

				case CMD_GET_SMART_DIS:
					testCode = connector.getSmartDigitalInputs(testRetCharArr);
					retType = RET_CHAR;
					retLen = 32;
					break;

				case CMD_SET_SMART_DO:
					testCode = connector.setSmartDigitalOutput(4, 1);
					break;

				case CMD_SET_SMART_DOS:
				{
					char b[32] = {1, 1, 0, 0, 0, 0, 1, 1};
					testCode = connector.setSmartDigitalOutputs(b);
				}
					break;

				case CMD_GET_SMART_AI:
					testCode = connector.getSmartAnalogInput(0, testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_SET_SMART_AO:
					testCode = connector.setSmartAnalogOutput(0, 234);
					break;

				case CMD_GET_SMART_DO:
					testCode = connector.getSmartDigitalOutput(20, testRetChar);
					retType = RET_CHAR;
					retLen = 1;
					break;

				case CMD_GET_SMART_DOS:
					testCode = connector.getSmartDigitalOutputs(testRetCharArr);
					retType = RET_CHAR;
					retLen = 32;
					break;

				case CMD_GET_SMART_AO:
					testCode = connector.getSmartAnalogOutput(0, testRetInt);
					retType = RET_INT;
					retLen = 1;
					break;

				case CMD_GET_EXTIO_FTCAN_ROBOT_RAW:
					testCode = connector.getRobotCanFTSensorRaw(testRetIntArr);
					retType = RET_INT;
					retLen = 6;
					break;

				case CMD_GET_EXTIO_FTCAN_ROBOT_TRANS:
					testCode = connector.getRobotCanFTSensorTransformed(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;

				case CMD_GET_EXTIO_FTCAN_CB_RAW:
					testCode = connector.getCBCanFTSensorRaw(testRetIntArr);
					retType = RET_INT;
					retLen = 6;
					break;

				case CMD_GET_EXTIO_FTCAN_CB_TRANS:
					testCode = connector.getCBCanFTSensorTransformed(testRetDoubleArr);
					retType = RET_DOUBLE;
					retLen = 6;
					break;
				}

				if (testCode >= 0)
				{
					cout << "Tested Command=" << testCmd << " ... " << endl;
					if (testCode == ERR_NONE && retType >= 0)
					{
						cout << "\t-Ret = [";
						for (int i=0; i<retLen; i++)
						{
							switch(retType)
							{
							case RET_BOOL: cout << (int)testRetBoolArr[i] << ", "; break;
							case RET_CHAR: cout << (int)testRetCharArr[i] << ", "; break;
							case RET_INT: cout << testRetIntArr[i] << ", "; break;
							case RET_DOUBLE: cout << testRetDoubleArr[i] << ", "; break;
							}
						}
						cout << "]" << endl;
					}
				}

				if (testCode == -1) testCode = ERR_NONE;
			}// while

			if (testCode != ERR_NONE)
			{
				cout << "\t- Failed!" << endl << "Test Failed with CMD=" << testCmd << ", RET=" << testCode << endl;
			}
		}

		else
		{
			cout << "Invalid Selection!" << endl;
			connector.disconnect();
		}
	}
	else
	{
		cout << "Connection failed.\n" << endl;
	}

	system_cleanup();
	return 0;
}

#endif
