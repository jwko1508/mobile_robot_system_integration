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
#include "std_msgs/Float64MultiArray.h"
#include "tf2/LinearMath/Matrix3x3.h"

//#include "geometry_msgs"

using namespace std;

using namespace NRMKIndy::Service::DCP;

class Indy7DCPClient
{

private:
    IndyDCPConnector connector;
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub;
    ros::Subscriber marker_pose_sub;
    ros::ServiceServer server_indy7;

    unsigned char m_Command;

    std_msgs::Float64MultiArray HomogeneousTransformationMatrix;

    ros::Rate rate100;

    sensor_msgs::JointState indyJointState;
    
public:
    Indy7DCPClient(string robotIP) : connector(robotIP, ROBOT_INDY7),
                                     joint_state_pub(nh.advertise<sensor_msgs::JointState>("joint_states", 10)),
                                     marker_pose_sub(nh.subscribe("MarkerHomogeneousMatrix", 10, &Indy7DCPClient::MarkerPoseCB, this)),
                                     server_indy7(nh.advertiseService("control_indy7", &Indy7DCPClient::IndyServerCB, this)),
                                     m_Command(0),
                                     rate100(100)
    {
        ROS_INFO_STREAM("INDY7DCP Client init!");
        cout << "Connecting to the robot" << endl;
        connector.connect();

        int jointLevel = 1;
        cout << "Set the limit level(" << jointLevel <<
                ") of velocity/acceleration of JointMove" << endl;
        connector.setJointBoundaryLevel(jointLevel);

        int taskLevel = 1;
        cout << "Set the limit level(" << taskLevel <<
            ") of velocity/acceleration of taskMove" << endl;
        connector.setTaskBoundaryLevel(taskLevel);

        // declare indy Joint State
        std::vector<string> name_of_joints;
        for (int i = 0; i < 6; ++i) {
            string name = "joint" + to_string(i);
            name_of_joints.push_back(name);
        }
        indyJointState.header.frame_id = ROBOT_INDY7;
        indyJointState.name = name_of_joints;

        HomogeneousTransformationMatrix.data.reserve(16);
        for (size_t i = 0; i < 16; i++)
        {
            HomogeneousTransformationMatrix.data.push_back(0);
        }
    }
    
    void MarkerPoseCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        HomogeneousTransformationMatrix = *msg;
    }

    bool IndyServerCB(std_srvs::SetBool::Request  &req,
                        std_srvs::SetBool::Response &res)
    {
        switch (req.data)
        {
            case moveHome:
            {
                ROS_INFO("I'm moving home.");
                connector.moveJointHome();
                WaitFinish(connector);
                res.message = "I moved home position.";
                res.success = true;
                return true;

                break;
            }
            case moveZero:
            {
                ROS_INFO("I'm moving zero positon.");
                connector.moveJointZero();
                WaitFinish(connector);
                res.message = "I moved zero positon.";
                res.success = true;
                return true;
                
                break;
            }
            case Move5cmUpInMarkerPose:
            {
                ROS_INFO("Indy7 is moving up 5cm in marker pose.");
                tf2::Matrix3x3 rotationMatrix;
                rotationMatrix.setValue(HomogeneousTransformationMatrix.data.at(0), HomogeneousTransformationMatrix.data.at(1), HomogeneousTransformationMatrix.data.at(2),
                                        HomogeneousTransformationMatrix.data.at(4), HomogeneousTransformationMatrix.data.at(5), HomogeneousTransformationMatrix.data.at(6), 
                                        HomogeneousTransformationMatrix.data.at(8), HomogeneousTransformationMatrix.data.at(9), HomogeneousTransformationMatrix.data.at(10));
                double roll, pitch, yaw;
                rotationMatrix.getRPY(roll,pitch,yaw);

                cout << __LINE__ << " x : " << HomogeneousTransformationMatrix.data.at(3) << endl;
                cout << __LINE__ << " y : " << HomogeneousTransformationMatrix.data.at(7) << endl;
                cout << __LINE__ << " z : " << HomogeneousTransformationMatrix.data.at(11) << endl;
                cout << __LINE__ << " roll : " << roll << endl;
                cout << __LINE__ << " pitch : " << pitch << endl;
                cout << __LINE__ << " yaw : " << yaw << endl;

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
                sleep(0.25);
        #elif defined(WINDOWS)
                Sleep(250);
        #endif
        connector.isMoveFinished(fin); // check if motion finished
        } while (!fin);
    }

    void PublishCurrentJointState()
    {
        // ros::Rate loop_rate = std::move(loopRate);
       
        bool ready;
        connector.isRobotReady(ready); 
        if(ready)
        {
            while (ros::ok())
            {
                double q[6];
                connector.getJointPosition(q);
                
                std::vector<double> q_vec;
                for (size_t i = 0; i < 6; i++)
                {
                    q_vec.push_back(M_PI / 180 * q[i]);
                }
                indyJointState.position = q_vec;
                indyJointState.header.stamp = ros::Time::now();
                joint_state_pub.publish(indyJointState);
                // loop_rate.sleep();
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
        boost::thread thread_b(boost::bind(&Indy7DCPClient::PublishCurrentJointState, this));
    }

    void EmergencyStop()
    {
        double temp;
        cin >> temp;
        connector.stopEmergency();
    }
};



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "indy_dcp_test");
    
    Indy7DCPClient indy7DCPClient("192.168.0.129");

    // auto f1 = std::async(&Indy7DCPClient::PublishCurrentJointState, &indy7DCPClient, 100);
    // boost::thread thread_b(boost::bind(&Indy7DCPClient::PublishCurrentJointState, &indy7DCPClient, 100));
    boost::thread thread_b(boost::bind(&Indy7DCPClient::PublishCurrentJointState, &indy7DCPClient));
    boost::thread thread_c(boost::bind(&Indy7DCPClient::EmergencyStop, &indy7DCPClient));
    // indy7DCPClient.RunThread();

    // indy7DCPClient.doTask();
    ros::spin();
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