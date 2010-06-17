#include <winch_action/WinchTargetAction.h>
#include <winch_action/Debug.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<winch_action::WinchTargetAction> Server;

static double current_depth(0);
static double target_depth(0);
static ros::Publisher dbg_pub;


void execute(const winch_action::WinchTargetGoalConstPtr& goal, Server* as)
{
  winch_action::WinchTargetResult result;
  result.depth = current_depth;
  
  switch (goal->mode) {
  case winch_action::WinchTargetGoal::MODE_PARK:
    break;
  case winch_action::WinchTargetGoal::MODE_READY:
    break;

  case winch_action::WinchTargetGoal::MODE_GOTO:
    if (goal->speed <= 0) {
      as->setRejected(result, "invalid speed");
      return;
    }
    if (goal->depth <= 0) {
      as->setRejected(result, "invalid depth");
      return;
    }
    if (fabs(current_depth - target_depth) < goal->speed) {
      current_depth = target_depth;
    }
    else {
      if (current_depth > target_depth) {
	current_depth -= goal->speed;
      }
      else {
	current_depth += goal->speed;
      }
    }
    {
      winch_action::Debug dbg;
      dbg.depth = current_depth;
    }
    break;
    
  default:
    {
      as->setRejected(result, "invalid mode");
      return;
    }
  }
  
  if (fabs(current_depth - target_depth) < 1e-2) {
    as->setSucceeded(result, "Yeah!");
  }
  else {
    winch_action::WinchTargetFeedback fb;
    fb.state = winch_action::WinchTargetFeedback::STATE_ACTIVE; //uint32 STATE_PAUSED = 2
    fb.depth = current_depth;
    fb.speed = goal->speed;
    as->publishFeedback(fb);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "winch_server");
  ros::NodeHandle nn;
  dbg_pub = nn.advertise<winch_action::Debug>("dbg", 1);
  Server server(nn, "winch", boost::bind(&execute, _1, &server));
  ros::spin();
  return 0;
}
