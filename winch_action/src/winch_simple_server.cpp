#include <winch_action/WinchTargetAction.h>
#include <winch_action/Debug.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<winch_action::WinchTargetAction> Server;

static double current_depth(0);
static ros::Publisher dbg_pub;


static void execute_goto(double depth, double speed, Server* as)
{
  winch_action::WinchTargetResult result;
  result.depth = current_depth;
  
  if (speed <= 0) {
    as->setRejected(result, "invalid speed");
    return;
  }
  
  winch_action::WinchTargetFeedback fb;
  fb.state = winch_action::WinchTargetFeedback::STATE_ACTIVE;
  fb.depth = current_depth;
  fb.speed = speed;
  as->publishFeedback(fb);
  
  while (fabs(current_depth - depth) > speed) {
    
    if (current_depth > depth) {
      current_depth -= speed;
    }
    else {
      current_depth += speed;
    }
    
    fb.depth = current_depth;
    as->publishFeedback(fb);
    
    usleep(100000);
  }
  
  current_depth = depth;
  
  result.depth = current_depth;
  as->setSucceeded(result, "Yeah!");
}


static void execute_park(Server* as)
{
  execute_goto(-1, 0.1, as);
}


static void execute_ready(Server* as)
{
  execute_goto(0, 0.5, as);
}


void execute(const winch_action::WinchTargetGoalConstPtr& goal, Server* as)
{
  switch (goal->mode) {
  case winch_action::WinchTargetGoal::MODE_PARK:
    execute_park(as);
    break;
    
  case winch_action::WinchTargetGoal::MODE_READY:
    execute_ready(as);
    break;
    
  case winch_action::WinchTargetGoal::MODE_GOTO:
    execute_goto(goal->depth, goal->speed, as);
    break;
    
  default:
    {
      winch_action::WinchTargetResult result;
      result.depth = current_depth;
      as->setRejected(result, "invalid mode");
      return;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "winch_simple_server");
  ros::NodeHandle nn;
  dbg_pub = nn.advertise<winch_action::Debug>("dbg", 1);
  Server server(nn, "winch", boost::bind(&execute, _1, &server));
  ros::spin();
  return 0;
}
