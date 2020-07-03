#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "visualization_msgs/Marker.h"
#include "std_srvs/SetBool.h"


#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <iostream>
#include <string>

using namespace std;

struct Location
{
    double x;
    double y;
    double heading;
};
struct Omron_State
{
    string status;
    double statusOfCharge;
    Location location;
    double Temperature;
};



#define MAX 4096
#define PORT 7171
#define SA struct sockaddr
#define TRUE 1
#define FALSE 0

Omron_State omron_state;
unsigned char g_Command = 0;
bool isInputOneCommand = false;

enum Omron_command
{
  gotogoal1 = 1, // 1
  gotogoal2, // 2
  gotogoal3, // ...
  gotogoal4,
  gotogoal5,
  omron_stop,
  omron_move,
  omron_success,
  omron_fail
};

// 현재 receive 함수만 만들었음 이제 연결해줘야 함. plc 수업 가기전 주석 200527 1329

bool OmronServerCB(std_srvs::SetBool::Request  &req,
                   std_srvs::SetBool::Response &res)
{
  ros::Rate loop_rate(10);
  cout << "Omron Server CB is requested..." << endl;
  switch (req.data)
  {
    case gotogoal1:
    {
      ROS_INFO("I'm going to goal1.");
      g_Command = gotogoal1;
      while(ros::ok())
      {
        if(g_Command == omron_success)
        {
          res.message = "I arrived at Goal1.";
          res.success = true;
          g_Command = 0;
          return true;
        }
        else if(g_Command == omron_fail)
        {
          res.message = "Fail Goal1 :(";
          res.success = false;
          g_Command = 0;
          return true;
        }
      }
      break;
    }
    case gotogoal2:
    {
      ROS_INFO("I'm going to goal2.");
      g_Command = gotogoal2;
      while(ros::ok())
      {
        if(g_Command == omron_success)
        {
          res.message = "I arrived at Goal2.";
          res.success = true;
          g_Command = 0;
          return true;
        }
        else if(g_Command == omron_fail)
        {
          res.message = "Fail Goal2 :(";
          res.success = false;
          g_Command = 0;
          return true;
        }
      }
      break;
    }
    default:
    {
      break;
    }
  }


}


Omron_State StringToDouble(char * buffer)
{


//    cout << 1 << endl;
//    char* tok1 = strtok(buffer, ": ");
    string receiveData = buffer;
    long firstIndex;
    long secondIndex;
    long numOfCharacters;
//    cout << 2 << endl;
    cout << "receiveData : " << receiveData << endl;
    if( (firstIndex = receiveData.find("Status")) >= 0 )
    {
//        cout << 3 << endl;
        // 012345678
        // status: s~    , (s~는 어떤 문자열으 시작을 의미함.)
        firstIndex += 8; // 8번째 인덱스 후에 우리가 원하는 데이터에 접근 가능함.
        secondIndex = receiveData.find("StateOfCharge");
        secondIndex -= 2; // StateOfCharge 이전은 공백임. 공백 전 인덱스까지.
        numOfCharacters = secondIndex - firstIndex + 1;
        omron_state.status = receiveData.substr(firstIndex, numOfCharacters);

//        cout << 4 << endl;
        firstIndex = receiveData.find("StateOfCharge");
        // 0123456789 10 11 12 13 14 15
        // StateOfCha r  g  e  :     s~
        firstIndex += 15;
        secondIndex = receiveData.find("Location");
        secondIndex -= 2;
        numOfCharacters = secondIndex - firstIndex + 1;
        string statusOfCharge_str = receiveData.substr(firstIndex, numOfCharacters);

        cout << "statusOfCharge_str : " << statusOfCharge_str << endl;
        omron_state.statusOfCharge = atof(statusOfCharge_str.c_str());

//        cout << 5 << endl;
        firstIndex = receiveData.find("Location");
        // 0123456789 10
        // Location:
        firstIndex += 10;
        secondIndex = receiveData.find("Temperature");
        secondIndex -= 2;

//        cout << 6 << endl;
        char* raw_location = new char(); // 메모리 할당을 해줘야 strcpy가 실행됨
        numOfCharacters = secondIndex - firstIndex + 1;
        string raw_location_str = receiveData.substr(firstIndex, numOfCharacters);

        cout << "raw_location_str : " << raw_location_str << endl;
//        cout << 6.1 << endl;
        strcpy(raw_location, raw_location_str.c_str());
//        cout << 6.101 << endl;
        char *tok1 = strtok(raw_location, " ");
//        cout << 6.11 << endl;
        omron_state.location.x = atof(tok1);
        tok1 = strtok(NULL, " ");
//        cout << 6.2 << endl;
        omron_state.location.y = atof(tok1);
        tok1 = strtok(NULL, " ");
        omron_state.location.heading = atof(tok1);

//        cout << 7 << endl;
        firstIndex = receiveData.find("Temperature");
        // 0123456789 10 11 12 13
        // Temperatur e  :
        firstIndex += 13;
        string temperature_str = receiveData.substr(firstIndex);
        omron_state.Temperature = atof(temperature_str.c_str());

//        cout << 8 << endl;
        printf("omron_status : \n");
        printf("status : %s\n", omron_state.status.c_str());
        printf("statusOfCharge : %f\n", omron_state.statusOfCharge);
        printf("x : %f\n", omron_state.location.x);
        printf("y : %f\n", omron_state.location.y);
        printf("heading : %f\n", omron_state.location.heading);
        printf("temperature : %f\n", omron_state.Temperature);



    }

    return omron_state;

}

void do_stuff(int* publish_rate)
{
  omron_state.location = {0};
  omron_state.statusOfCharge = 0;
  omron_state.status = "";
  omron_state.Temperature = 0;

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

  ros::Publisher vis_pub = node->advertise<visualization_msgs::Marker>( "visualization_marker", 1 );


//    double past = ros::Time::now().toSec();
//
//    while((ros::Time::now().toSec() - past) <= 1.0)
//    {
//        // 1초동안 여러번 입력해야 좀 나옴
//        vis_pub.publish(marker);
//    }
  char message[1000];
  char buffer[MAX];
  int clientSocket;
  struct sockaddr_in serverAddr;
  socklen_t addr_size;
  // Create the socket.
  clientSocket = socket(PF_INET, SOCK_STREAM, 0);
  //Configure settings of the server address
  // Address family is Internet
  serverAddr.sin_family = AF_INET;
  //Set port number, using htons function
  serverAddr.sin_port = htons(7171);
  //Set IP address to localhost
  serverAddr.sin_addr.s_addr = inet_addr("192.168.0.118");
  memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);
  //Connect the socket to the server using the address
  addr_size = sizeof serverAddr;
  connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size);

  strcpy(message,"admin\n");
  for (int i = 0; i < 2; ++i)
  {
    if(send(clientSocket , message , strlen(message) , 0) < 0)
    {
      printf("Send failed\n");
    }
    sleep(0.5);
  }

  std::string initialGuidance;
  while(initialGuidance.find("End of") == -1)
  {
    recv(clientSocket, buffer, 1024, 0);
    initialGuidance = buffer;
  }

  ros::Rate loop_rate(*publish_rate);
  while (ros::ok())
  {


    strcpy(message,"onelinestatus\r\n");
    bzero(buffer, sizeof(buffer));
    if( send(clientSocket , message , strlen(message) , 0) < 0)
    {
      printf("Send failed\n");
    }
    //Read the message from the server into the buffer
    if(recv(clientSocket, buffer, 1024, 0) < 0)
    {
      printf("Receive failed\n");
    }
    char receiveStatus[MAX];
    strcpy(receiveStatus, buffer);
    char* receiveStatus_p = receiveStatus;
//      cout << 0 << endl;

    StringToDouble(receiveStatus_p);

    if(g_Command == gotogoal1)
    {
      if(!isInputOneCommand)
      {
        strcpy(message,"goto goal1\r\n");
        bzero(buffer, sizeof(buffer));
        cout << "goto goal1!!" << endl;
        if( send(clientSocket , message , strlen(message) , 0) < 0)
        {
          printf("Send failed\n");
        }
        isInputOneCommand = true;
      }

      if(recv(clientSocket, buffer, 1024, 0) < 0)
      {
        printf("Receive failed\n");
      }
      strcpy(receiveStatus, buffer);
      string receiveStatus_S = receiveStatus;

      if(receiveStatus_S.find("Arrived at Goal1") >= 0)
      {
        cout << "we Arrived at Goal1" << endl;
        isInputOneCommand = false;
        g_Command = omron_success;
      }
    }
    else if(g_Command == gotogoal2)
    {
      if(!isInputOneCommand)
      {
        strcpy(message,"goto goal2\r\n");
        bzero(buffer, sizeof(buffer));
        cout << "goto goal2!!" << endl;
        if( send(clientSocket , message , strlen(message) , 0) < 0)
        {
          printf("Send failed\n");
        }
        isInputOneCommand = true;
      }

      if(recv(clientSocket, buffer, 1024, 0) < 0)
      {
        printf("Receive failed\n");
      }
      strcpy(receiveStatus, buffer);
      string receiveStatus_S = receiveStatus;

      if(receiveStatus_S.find("Arrived at Goal2") >= 0)
      {
        cout << "we Arrived at Goal2" << endl;
        isInputOneCommand = false;
        g_Command = omron_success;
      }
    }

    // tf publish
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "ground";
    transformStamped.child_frame_id = "omron_mobile";
    transformStamped.transform.translation.x = 0.001 * omron_state.location.x;
    transformStamped.transform.translation.y = 0.001 * omron_state.location.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI/180 * omron_state.location.heading);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);


    // marker for visualization.
    visualization_msgs::Marker marker;
    marker.header.frame_id = "omron_mobile";
    marker.header.stamp = ros::Time();
    marker.lifetime = ros::Duration();
    marker.ns = "hello";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.001 * omron_state.location.x;
    marker.pose.position.y = 0.001 * omron_state.location.y;
    double height = 0.95;
    marker.pose.position.z = height/2;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.6;
    marker.scale.y = 0.4;
    marker.scale.z = height;
    marker.color.a = 0.6; // Don't forget to set the alpha!
    marker.color.r = 0.9;
    marker.color.g = 0.9;
    marker.color.b = 0.9;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";


    //Print the received message
//    printf("DR : %s\n",buffer);
//      cout << 10 << endl;

    loop_rate.sleep();
  }

  close(clientSocket);
}

int main(int argc, char** argv)
{
  int rate_b = 10; // 1 Hz

  ros::init(argc, argv, "boost_tcp_client");

  // spawn another thread
  boost::thread thread_b(do_stuff, &rate_b);

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceServer server_omron = node->advertiseService("control_omron", OmronServerCB);

  ros::spin();

  return 0;
}

////// \n 나올 때 까지 기다리는 거
//n = 0;
//    while ((buff[n++] = getchar()) != '\n');
//    if ((strncmp(buff, "exit", 4)) == 0) {
//      printf("inital func Exit...\n");
//      break;
//    }




//    while (tok1 != NULL)
//    {
//        if(strcmp(tok1, "Status") == 0)
//        {
//            tok1 = strtok(NULL, ": ");
//            cout << "1. " << tok1 << endl;
//            string buffer_s = buffer;
//            tok1 = strtok(NULL, " ");
//            omron_state.status = tok1;
//            cout << "omron_state.status : " << tok1 << endl;
//            tok1 = strtok(NULL, ": ");
////            cout << "2. " << tok1 << endl;
//        }
//        else if (strcmp(tok1, "StateOfCharge") == 0)
//        {
//            tok1 = strtok(NULL, ": ");
//            omron_state.statusOfCharge = atof(tok1);
//            cout << "omron_state.statusOfCharge : " << atof(tok1) << endl;
//            tok1 = strtok(NULL, ": ");
//        }
//        else if (strcmp(tok1, "Location") == 0)
//        {
//            tok1 = strtok(NULL, ": ");
//            omron_state.location.x = atof(tok1);
//            cout << "omron_state.Location.x : " << atof(tok1) << endl;
//            tok1 = strtok(NULL, " ");
//            omron_state.location.y = atof(tok1);
//            cout << "omron_state.Location.y : " << atof(tok1) << endl;
//            tok1 = strtok(NULL, " ");
//            omron_state.location.heading = atof(tok1);
//            cout << "omron_state.Location.heading : " << atof(tok1) << endl;
////            tok1 = strtok(NULL, " ");
//        }
//        else if (strcmp(tok1, "Temperature") == 0)
//        {
//            tok1 = strtok(NULL, ": ");
//            omron_state.Temperature = atof(tok1);
//            cout << "omron_state.Temperature : " << atof(tok1) << endl;
//        }
//        else
//        {
//            return omron_state;
//        }
//
//    }