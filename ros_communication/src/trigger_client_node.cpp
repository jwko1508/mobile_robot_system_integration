#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <cstdlib>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trigger_client_node");
  ros::NodeHandle n;

  ROS_INFO("trigger client started.");
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("trigger");
  std_srvs::Trigger srv;
  string any_string;
  cin >> any_string;
  if (client.call(srv))
  {
    ROS_INFO("trigger operation success.");
    cout << srv.response.message << endl;
  }
  else
  {
    ROS_ERROR("Failed to call service trigger");
    return 1;
  }

  return 0;
}