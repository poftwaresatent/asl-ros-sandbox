#include <winch_action/WinchTargetAction.h>
#include <actionlib/server/action_server.h>
#include <boost/scoped_ptr.hpp>

typedef actionlib::ActionServer<winch_action::WinchTargetAction> server_t;

static boost::scoped_ptr<server_t> action_server;
static server_t::GoalHandle goal_handle;
static bool have_goal(false);
static double current_depth(0);


static bool update_goto(double depth, double speed)
{
  winch_action::WinchTargetResult result;
  result.depth = current_depth;
  
  if (speed <= 0) {
    goal_handle.setRejected(result, "invalid speed");
    return false;
  }
  
  if (fabs(current_depth - depth) > speed) {
    
    if (current_depth > depth) {
      current_depth -= speed;
    }
    else {
      current_depth += speed;
    }
    
  }
  else {
    current_depth = depth;
  }
  
  winch_action::WinchTargetFeedback fb;
  fb.state = winch_action::WinchTargetFeedback::STATE_ACTIVE;
  fb.depth = current_depth;
  fb.speed = speed;
  goal_handle.publishFeedback(fb);
  ROS_INFO("Probe pos is at %f m depth, moving at %f m/s", current_depth, speed);
  
  if (fabs(current_depth - depth) <= speed) {
    result.depth = current_depth;
    goal_handle.setSucceeded(result, "Yeah!");
    ROS_INFO("Probe has reached its goal");
    return false;
  }
  
  return true;
}


static bool update_park()
{
  return update_goto(-1, 0.1);
}


static bool update_ready()
{
  return update_goto(0, 0.5);
}


static void goal_cb(server_t::GoalHandle gh)
{
  if (have_goal) {
    goal_handle.setCanceled();
    have_goal = false;
  }
  
  gh.setAccepted();
  goal_handle = gh;
  have_goal = true;
}


static void cancel_cb(server_t::GoalHandle gh)
{
  if (goal_handle != gh) {
    return;
  }
  
  // stop the controller...
  
  have_goal = false;
  goal_handle.setCanceled();
}

static void update_step(const ros::TimerEvent& timer_event)
{
  if ( ! have_goal)
    return;
  
  switch (goal_handle.getGoal()->mode) {
  case winch_action::WinchTargetGoal::MODE_PARK:
    ROS_INFO("Probe parking");
    have_goal = update_park();
    break;
    
  case winch_action::WinchTargetGoal::MODE_READY:
    ROS_INFO("Probe readying");
    have_goal = update_ready();
    break;
    
  case winch_action::WinchTargetGoal::MODE_GOTO:
    ROS_INFO("Probe goto");
    have_goal = update_goto(goal_handle.getGoal()->depth, goal_handle.getGoal()->speed);
    break;
    
  default:
    {
      ROS_INFO("Invalid probe mode");
      winch_action::WinchTargetResult result;
      result.depth = current_depth;
      goal_handle.setRejected(result, "invalid mode");
      have_goal = false;
    } 
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "winch_server");
  ros::NodeHandle nn;
  ros::Timer timer = nn.createTimer(ros::Duration(0.1), update_step);
  action_server.reset(new server_t(nn, "winch", goal_cb, cancel_cb));
  ros::spin();
  return 0;
}
