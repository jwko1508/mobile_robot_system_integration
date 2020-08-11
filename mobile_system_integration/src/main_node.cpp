#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/String.h"
#include "darknet_ros_msgs/ObjectCount.h"
#include "mobile_system_integration/OmronIndyCommand.h"

#include <cstdlib>

bool emergency_state = false;
bool isCoffee = false;
bool isDetectComplete = false;

using namespace std;
int robot_behavior_flow = listening;
int detection_count = 0;

void speech_cb(const std_msgs::String& msg)
{
  string temp = msg.data;
  if(temp.find("coffee") >= 0)
  {
    isCoffee = true;
  }
}

void darknet_cb(const darknet_ros_msgs::ObjectCount& msg)
{
  if(msg.count > 0)
  {
    detection_count++;
    cout << "detection_count : " << detection_count << endl;
  }
}

bool Call_service_and_move_next_flow(ros::ServiceClient& srv_client,
                      std_srvs::SetBool& srv,
                      int flow_after_successive_call)
{
  if(srv_client.call(srv))
  {
    ROS_INFO("trigger operation success.");
    cout << "srv.res : " << srv.response.message << endl;
    robot_behavior_flow = flow_after_successive_call;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service trigger");
    return false;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;

  ROS_INFO("mobile system started.");
  ros::Subscriber speech_sub = n.subscribe("dictation_text", 10, speech_cb);
  ros::Subscriber darknet_sub = n.subscribe("/darknet_ros/found_object", 10, darknet_cb);

  ros::ServiceClient service_omron_client = n.serviceClient<std_srvs::SetBool>("control_omron");
  ros::ServiceClient control_indy7_to_pick_up_cl = n.serviceClient<std_srvs::SetBool>("control_indy7");
  ros::ServiceClient get_marker_pose_by_using_indyeye = n.serviceClient<std_srvs::SetBool>("getMarkerPose");

  while(ros::ok())
  {
    switch (robot_behavior_flow)
    {
      case listening:
      {
//        cout << "listening" << endl;
        ros::spinOnce();
//        if(isCoffee)
//        {
          double temp;
          cin >> temp;
          cout << "I has listened." << endl;
          robot_behavior_flow = control_indy7_to_pick_up;
//        }
        break;
      }

      case go_to_goal_by_using_omron:
      {
        // cout << "go_to_goal_by_using_omron" << endl;
        // std_srvs::SetBool srv;
        // srv.request.data = gotogoal1;
        // if(!Call_service_and_move_next_flow(service_omron_client, srv, control_indy7_to_pick_up))
        //   return 0;
        // sleep(1);
        // break;
      }

      case control_indy7_to_pick_up:
      {
        ROS_INFO_STREAM("control indy7 to move home position");
        std_srvs::SetBool srv;
        // srv.request.data = moveHome;
        // if(!Call_service_and_move_next_flow(control_indy7_to_pick_up_cl, srv, control_indy7_to_pick_up))
        //   return 0;
        // sleep(1);

        ROS_INFO_STREAM("control indyeye to get object pose data.");
        srv.request.data = getObjectPose;
        if(Call_service_and_move_next_flow(get_marker_pose_by_using_indyeye, srv, listening))
        {
          // pass
        }
        else // when failed to get a marker pose.
        {
          robot_behavior_flow = listening;
          continue;
        }
        sleep(1);

        // ROS_INFO_STREAM("control indy7 to move 5cm up from marker position");
        // srv.request.data = Move5cmUpInMarkerPose;
        // if(!Call_service_and_move_next_flow(control_indy7_to_pick_up_cl, srv, listening))
        //   return 0;
        // sleep(1);

        // ROS_INFO_STREAM("control indy7 to move zero position");
        // srv.request.data = moveZero;
        // if(!Call_service_and_move_next_flow(control_indy7_to_pick_up_cl, srv, listening))
        //   return 0;
        // sleep(1);

        break;
      }

      case check_to_pick_up_complete:
      {
//        cout << "checking.." << endl;
//        std_srvs::SetBool srv;
//        srv.request.data = 2;
//
//        if(!Service_set_bool(control_indy7_to_pick_up_cl, srv, check_to_pick_up_complete))
//          return 0;
//
//        int detection_count_threshold = 3;
//        cout << "check complete." << endl;
//        auto past_time = ros::Time::now().toSec();
//        while(ros::Time::now().toSec() - past_time <= 5.0)
//        {
//          ros::spinOnce();
//          if(detection_count > detection_count_threshold)
//          {
//            robot_behavior_flow = go_to_home_by_using_omron;
//            break;
//          }
//        }
//        if(detection_count <= detection_count_threshold)
//        {
//          cout << "I'll unwillingly back to control_indy7_to_pick_up state." << endl;
//          robot_behavior_flow = control_indy7_to_pick_up;
//        }
//        detection_count = 0;
//        break;
      }

      // case go_to_home_by_using_omron:
      // {
      //   cout << "control_indy7_to_pick_up" << endl;
      //   std_srvs::SetBool srv;
      //   srv.request.data = gotogoal2;
      //   if(!Call_service_and_move_next_flow(service_omron_client, srv, listening))
      //     return 0;
      //   cout << "srv.response.success : " << srv.response.success << endl;
      //   break;
      // }

      case hand_over_something:
      {
//        cout << "here you are~" << endl;
//
//        robot_behavior_flow = listening;
//        isCoffee = false;
//        isDetectComplete = false;
//        break;
      }
      default:
        cout << "Unknown case robot stats : [" << robot_behavior_flow << "]" << endl;
    }

//    ros::spinOnce();
  }

//  ros::spin(); 이거 안뜨면 ros::spin 안먹음 그러니까 while문 안에 spinOnce가 있어줘야함.

  return 0;
}