#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "visualization_msgs/Marker.h"
#include "std_srvs/SetBool.h"

#include <stdio.h>
#include <term.h>
#include <termios.h>
#include <unistd.h>

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

Omron_State omron_state;

#define MAX 4096
#define PORT 7171
//#define SA struct sockaddr
#define TRUE 1
#define FALSE 0

enum Omron_command
{
  gotogoal1 = 1, // 1
  gotogoal2, // 2
  gotogoal3, // ...
  gotogoal4,
  gotogoal5,
  omron_stop,
  omron_move
};


char isEmergency = FALSE;
unsigned char g_Command = 0;

// 현재 receive 함수만 만들었음 이제 연결해줘야 함. plc 수업 가기전 주석 200527 1329

int getch(void)
{
  int ch;
  struct termios buf;
  struct termios save;

  tcgetattr(0, &save); // terminal 저장
  buf = save; // buf로 똑같이 만들기.
  buf.c_lflag &= ~(ICANON|ECHO); // canonical 과 echo 종료
  buf.c_cc[VMIN] = 1; // 최소문자 1
  buf.c_cc[VTIME] = 0; // 최소 시간 0
  tcsetattr(0, TCSAFLUSH, &buf); // 설정값 적용
  ch = getchar(); // 값 받기
  tcsetattr(0, TCSAFLUSH, &save); // 설정 원래대로 돌아가기
  return ch;
}

void CheckEmergencyInput(int* publish_rate)
{
  ros::NodeHandlePtr checkNode = boost::make_shared<ros::NodeHandle>();
  ros::Rate loopRate(*publish_rate); // 100 Hz Check

  int ch;
  int i = 0;
  while(ros::ok() && !isEmergency)
  {
    ch = getch();
    if(ch == ' ' && !isEmergency)
    {
      isEmergency = TRUE;
    }
    printf("You pushed the emergency button(space bar'%c'). \n", ch);
    cout << "i : " << i << endl; // i이거 안돌아감 왜냐하면 기다리고 있는 중이거든. getchar 함수가
    // 그런 의미에서 loop rate 의미 없음.
    ++i;
    loopRate.sleep();
  }

}

void NoticeProgress()
{
  ros::Rate loop_rate(10);
  auto start_time = ros::Time::now();
  while(g_Command) // It doesn't matter proceeding time.
  {
    double proceeding_time = ros::Time::now().toSec() - start_time.toSec();
    cout <<  "proceeding_time : " << proceeding_time << endl;
    loop_rate.sleep();
  }
}

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
        if(g_Command == omron_stop)
        {
          res.message = "I arrived at Goal1.";
          res.success = true;
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
    omron_state.location = {0};
    omron_state.statusOfCharge = 0;
    omron_state.status = "";
    omron_state.Temperature = 0;

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

  int x_var(1000);
  int y_var(1500);
  int heading_var(0);

  ros::Rate loop_rate(*publish_rate);
  while (ros::ok() && !isEmergency)
  {
    if(g_Command == gotogoal1)
    {
      Location location_for_sim;
      location_for_sim.x = x_var;
      location_for_sim.y = y_var;
      location_for_sim.heading = heading_var;
      x_var += 10;
      y_var += 10;
      heading_var += 1;
      if(x_var >= 2000)
      {
        x_var = 1000;
        y_var = 1500;
        heading_var = 0;
        g_Command = omron_stop;
      }
    }

    stringstream onelinestatus_for_simulation_ss;

    onelinestatus_for_simulation_ss << "Status: Arrived at goal1 StateOfCharge: 23 Location: "
                                  << x_var << " " << y_var << " " << heading_var << " Temperature: 35.2\r\n";
    string onelinestatus_for_simulation = onelinestatus_for_simulation_ss.str();

    strcpy(buffer, onelinestatus_for_simulation.c_str());

    //Status: Arrived at <goal> BatteryVoltage: <volts_dc> Location: <X_mm> <Y_mm> <heading>
      //  Temperature: <degrees>

    char receiveStatus[MAX];
    strcpy(receiveStatus, buffer);
    char* receiveStatus_p = receiveStatus;
//      cout << 0 << endl;

    StringToDouble(receiveStatus_p);

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
    marker.header.frame_id = "ground";
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

    vis_pub.publish(marker);

    //Print the received message
    printf("DR : %s\n",buffer);
//      cout << 10 << endl;

    loop_rate.sleep();
  }

  close(clientSocket);
}

int main(int argc, char** argv)
{
  int rate_b = 10; // 1 Hz
  int rate_c = 100; // 1 Hz
  struct termios save_for_ctrl_c; // 터미널 갑자기 꺼질 때를 대비해서
  tcgetattr(0, &save_for_ctrl_c); // terminal 저장

  ros::init(argc, argv, "boost_tcp_client_for_simulation");

  // spawn another thread
  boost::thread thread_b(do_stuff, &rate_b);
  boost::thread thread_CheckEmergencyInput(CheckEmergencyInput, &rate_c);

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

  ros::ServiceServer server_omron = node->advertiseService("control_omron", OmronServerCB);

  ros::spin();

  tcsetattr(0, TCSAFLUSH, &save_for_ctrl_c); // 터미널 설정 원래대로 돌아가기

  // wait the second thread to finish
//  thread_b.join();
//  thread_CheckEmergencyInput.join();

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