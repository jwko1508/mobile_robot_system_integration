#include <unistd.h>
#include "ros/ros.h"
#include "indy_dcp_client/IndyDCPConnector.h"
#include <stdio.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <future>

#include "mobile_system_integration/OmronIndyCommand.h"

#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include "std_srvs/SetBool.h"

//#include "geometry_msgs"

using namespace std;

using namespace NRMKIndy::Service::DCP;

class Indy7DCPClient
{

private:
    IndyDCPConnector connector;
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub;
    ros::ServiceServer server_indy7;

    unsigned char m_Command;

    ros::Rate rate100;
    
public:
    Indy7DCPClient(string robotIP) : connector(robotIP, ROBOT_INDY7),
                                     joint_state_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 10)),
                                     m_Command(0),
                                     rate100(100)
    {
        ROS_INFO_STREAM("INDY7DCP Client init!");
        cout << "Connecting to the robot" << endl;
        connector.connect();
        server_indy7 = nh.advertiseService("control_indy", &Indy7DCPClient::IndyServerCB, this);
    }
    ~Indy7DCPClient()
    {

    }

    bool IndyServerCB(std_srvs::SetBool::Request  &req,
                        std_srvs::SetBool::Response &res)
    {
        
         switch (req.data)
        {
            case moveHome:
            {
                ROS_INFO("I'm moving home.");
                res.message = "I moved home.";
                cout << "Go to home position" << endl;
                connector.moveJointHome();
                WaitFinish(connector);

                res.success = true;
                return true;

                break;
            }
            case moveZero:
            {
                ROS_INFO("I'm moving zero positon.");
                res.message = "I moved zero positon.";
                cout << "Go to zero position" << endl;
                connector.moveJointZero();
                WaitFinish(connector);

                res.success = true;
                return true;
                
                break;
            }
            default:
            {
                break;
            }
        }
    }

    void WaitFinish(IndyDCPConnector& connector) {
        // Wait for motion finishing
        bool fin = false;
        do {
        #if defined (LINUX)
                sleep(0.5);
        #elif defined(WINDOWS)
                Sleep(500);
        #endif
        connector.isMoveFinished(fin); // check if motion finished
        } while (!fin);
    }

    void PublishCurrentJointState(double loopRate)
    {
        ros::Rate loop_rate(loopRate);
        sensor_msgs::JointState indyJointState;
        
        std::vector<string> name_of_joints;
        for (int i = 0; i < 6; ++i) {
            string name = "joint" + to_string(i);
            name_of_joints.push_back(name);
        }

        indyJointState.header.frame_id = ROBOT_INDY7;
        indyJointState.name = name_of_joints;

        bool ready;
        while (ros::ok())
        {
            connector.isRobotReady(ready); // 집에서 이거 추가했었음 반복문 안에 ready 넣는거
            if(ready)
            {
                double q[6];
                connector.getJointPosition(q);
                
                std::vector<double> q_vec;
                for (size_t i = 0; i < 6; i++)
                {
                    q_vec.push_back(q[i]);
                }
                indyJointState.position = q_vec;
                indyJointState.header.stamp = ros::Time::now();
                joint_state_pub.publish(indyJointState);
                loop_rate.sleep();
            }
        }
    }

    void doTask()
    {
        bool ready;
        connector.isRobotReady(ready);
        
        if (ready) {
            cout << "Robot is ready" << endl;

            int jointLevel = 1;
            cout << "Set the limit level(" << jointLevel <<
                    ") of velocity/acceleration of JointMove" << endl;
            connector.setJointBoundaryLevel(jointLevel);

            int taskLevel = 1;
            cout << "Set the limit level(" << taskLevel <<
                ") of velocity/acceleration of JointMove" << endl;
            connector.setTaskBoundaryLevel(taskLevel);

            cout << "Go to home position" << endl;
            connector.moveJointHome();
            WaitFinish(connector);

            cout << "Rotate last joint by 10 degrees" << endl;
            connector.moveJointBy({ 0,0,0,0,0,10 });
            WaitFinish(connector);

            connector.moveJointZero();
            WaitFinish(connector);

            connector.moveJointHome();
            WaitFinish(connector);

            cout << "Current end-effector position:" << endl;
            double p[6];
            connector.getTaskPosition(p);
            for (int i = 0; i < 6; i++) { cout << p[i] << ","; }
            cout << endl;

            cout << "Move end-effector upward by 5 cm" << endl;
            connector.moveTaskBy({ 0,0,0.05,0,0,0 });
            WaitFinish(connector);

            cout << "Move end-effector to saved position" << endl;
            connector.moveTaskTo(p);
            WaitFinish(connector);

            char ret;
            cout << "Read DI 20: " << endl;
            connector.getSmartDigitalInput(20, ret);
            cout <<  ret << endl;

            cout << "Write DO 4 as HIGH";
            connector.setSmartDigitalOutput(4, 1);

            connector.moveJointZero();
            WaitFinish(connector);

            cout << "Disconnecting robot" << endl;
            connector.disconnect();
        }
    }

    void RunThread()
    {
        boost::thread thread_b(boost::bind(&Indy7DCPClient::PublishCurrentJointState, this, 100));
    }

   


};



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "indy_dcp_test");
    
    Indy7DCPClient indy7DCPClient("192.168.0.129");

    // auto f1 = std::async(&Indy7DCPClient::PublishCurrentJointState, &indy7DCPClient, 100);
    boost::thread thread_b(boost::bind(&Indy7DCPClient::PublishCurrentJointState, &indy7DCPClient, 100));
    // indy7DCPClient.RunThread();

    indy7DCPClient.doTask();

    // f1.get();
    
}

// void joint_state_pub(int* publish_rate)
// {
//     bool ready;
//     connector.isRobotReady(ready);
//     if(ready)
//     {
//         cout << 1 << endl;
//         ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
//         ros::Publisher pub_b = node->advertise<sensor_msgs::JointState>("joint_states", 10);
//
//         ros::Rate loop_rate(*publish_rate);
//         sensor_msgs::JointState indyJointState;
//
//         cout << 2 << endl;
//         std::vector<string> name_of_joints;
//         for (int i = 0; i < 6; ++i) {
//             string name = "joint" + to_string(i);
//             name_of_joints.push_back(name);
//         }
//
//         indyJointState.header.frame_id = ROBOT_INDY7;
//         indyJointState.name = name_of_joints;
//
//         cout << 3 << endl;
//         while (ros::ok())
//         {
//             connector.isRobotReady(ready); // 집에서 이거 추가했었음 반복문 안에 ready 넣는거
//             if(ready)
//             {
//                 double q[6];
//                 connector.getJointPosition(q);
//                 IndyJointSubstitution(indyJointState, q);
//                 pub_b.publish(indyJointState);
//                 loop_rate.sleep();
//             }
//         }
//     }
//
// }