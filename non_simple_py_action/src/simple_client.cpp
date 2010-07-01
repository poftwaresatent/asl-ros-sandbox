#include <non_simple_py_action/CountAction.h>
#include <actionlib/client/simple_action_client.h>
#include <err.h>

typedef actionlib::SimpleActionClient<non_simple_py_action::CountAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_count_client");

  warnx("waiting for server");
  Client client("count", true); // true -> don't need ros::spin()
  client.waitForServer();

  warnx("sending goal");
  non_simple_py_action::CountGoal goal;
  goal.begin = -10;
  goal.end = 10;
  client.sendGoal(goal);
  
  warnx("waiting for result");
  client.waitForResult();
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay!\n");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
