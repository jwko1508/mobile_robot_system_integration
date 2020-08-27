#ifndef OMRONINDYCOMMAND_H
#define OMRONINDYCOMMAND_H

#define listening 1
#define go_to_goal_by_using_omron 2
#define control_indy7_to_pick_up 3
#define check_to_pick_up_complete 4
#define go_to_home_by_using_omron 5
#define hand_over_something 6

enum Omron_command
{
  gotogoal1 = 1, // 1
  gotogoal2, // 2
  gotogoal3, // ...
  gotogoal4,
  omron_success,
  omron_fail,
  omron_stop,
  omron_move
};
enum Indy_command
{
  moveHome = 100,
  moveZero,
  moveOrganazation,
  drawLine,
  getMarkerPose,
  Move5cmUpInMarkerPose,
  getObjectPose,
  indy_success,
  indy_fail
};

#endif