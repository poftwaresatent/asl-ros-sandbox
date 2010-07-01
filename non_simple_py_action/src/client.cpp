#include <non_simple_py_action/CountAction.h>
#include <actionlib/client/action_client.h>
#include <err.h>


typedef actionlib::ActionClient<non_simple_py_action::CountAction> Client;
typedef boost::function<void (Client::GoalHandle) > TransitionCallback;
typedef boost::shared_ptr<const non_simple_py_action::CountFeedback> FeedbackConstPtr;
typedef boost::function<void (Client::GoalHandle, const FeedbackConstPtr &) > FeedbackCallback;


static void transition_cb(Client::GoalHandle gh)
{
  warnx("transition_cb\n");
}


static void feeback_cb(Client::GoalHandle gh, const FeedbackConstPtr & fb)
{
  warnx("feeback_cb\n");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "count_client");
  
  Client client("count");
  warnx("waiting for server");
  if ( ! client.waitForActionServerToStart(ros::Duration(10.0))) {
    errx(EXIT_FAILURE, "no action server");
  }
  
  warnx("sending goal");
  non_simple_py_action::CountGoal goal;
  goal.begin = -10;
  goal.end = 10;
  Client::GoalHandle gh(client.sendGoal(goal));
  
  warnx("waiting until we're done");
  while (ros::ok() && (gh.getCommState() != actionlib::CommState::DONE)) {
    ros::spinOnce();
    usleep(10000);
  }
}
