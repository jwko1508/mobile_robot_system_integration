#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

using namespace std;

bool setBool(std_srvs::SetBool::Request  &req,
             std_srvs::SetBool::Response &res)
{
  ros::Rate loop_rate(10);
  cout << "proceeding..." << endl;
  cout << " req.data " << (int)req.data << endl;
  auto start_time = ros::Time::now();
//  cout << "ros::Time::now().toSec() - start_time.toSec() : " << ros::Time::now().toSec() - start_time.toSec() << endl;
  while(ros::Time::now().toSec() - start_time.toSec() <= req.data) // It doesn't matter proceeding time.
  {
    double proceeding_time = ros::Time::now().toSec() - start_time.toSec();
    cout <<  "proceeding_time : " << proceeding_time << endl;
    loop_rate.sleep();
  }
  cout << "process complete!" << endl;
  res.message = "good set bool";
  res.success = true;
  return true;
}

bool Trigger(std_srvs::Trigger::Request  &req,
           std_srvs::Trigger::Response &res)
{
  ros::Rate loop_rate(10);
  cout << "proceeding..." << endl;
  auto start_time = ros::Time::now();
//  cout << "ros::Time::now().toSec() - start_time.toSec() : " << ros::Time::now().toSec() - start_time.toSec() << endl;
  while(ros::Time::now().toSec() - start_time.toSec() <= 1.0) // It doesn't matter proceeding time.
  {
    double proceeding_time = ros::Time::now().toSec() - start_time.toSec();
    cout <<  "proceeding_time : " << proceeding_time << endl;
    loop_rate.sleep();
  }
  cout << "process complete!" << endl;
  res.message = "good";
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger_server_node");
  ros::NodeHandle n;
  ros::Time::init();

  ros::ServiceServer service = n.advertiseService("trigger", Trigger);
  ros::ServiceServer service2 = n.advertiseService("setBool", setBool);
  ROS_INFO("Ready to trigger server.");
  ros::spin();

  return 0;
}