#include <winch_action/WinchTargetAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<winch_action::WinchTargetAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "winch_client");
  Client client("winch", true); // true -> don't need ros::spin()
  client.waitForServer();
  winch_action::WinchTargetGoal goal;
  goal.mode = winch_action::WinchTargetGoal::MODE_GOTO;
  goal.depth = 42;
  goal.speed = 17;
  client.sendGoal(goal);
  client.waitForResult();
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay!\n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
