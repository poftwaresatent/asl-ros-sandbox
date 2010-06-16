#include <ros/ros.h>
#include <asltutorial/GoCommand.h>
#include <asltutorial/SetGoal.h>

using namespace asltutorial;

static double goal;
static double current;
static double increment;

static bool set_goal_callback(SetGoal::Request & request, SetGoal::Response & response);


int main(int argc, char*argv[])
{
  goal = 0;
  current = 0;
  increment = 0.1;
  
  ros::init(argc, argv, "go_server");
  ros::NodeHandle nn("~");
  
  ROS_INFO ("creating set_goal service");
  ros::ServiceServer set_goal_server(nn.advertiseService("set_goal", set_goal_callback));
  
  ROS_INFO ("creating go_command publisher");
  ros::Publisher command_pub(nn.advertise<GoCommand>("go_command", 100));
  
  ROS_INFO ("entering loop");
  GoCommand command_msg;
  while (ros::ok()) {
    
    if (fabs(current - goal) < increment) {
      current = goal;
    }
    else {
      if (current > goal) {
	current -= increment;
      }
      else {
	current += increment;
      }
    }
    
    command_msg.command = current;
    command_pub.publish(command_msg);
    ros::spinOnce();
    
    usleep(10000);
  }
}


bool set_goal_callback(SetGoal::Request & request, SetGoal::Response & response)
{
  if (request.goal < -99) {
    response.ok = false;
    response.errstr = "ERROR: goal < -99";
    return true; // always return true, otherwise the reply does not get sent
  }

  goal = request.goal;
  ROS_INFO ("new goal: %g", goal);
  
  response.ok = true;
  return true;
}
