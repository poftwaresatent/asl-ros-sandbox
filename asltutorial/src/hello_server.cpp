#include <ros/ros.h>
#include <asltutorial/SayHello.h>

using namespace asltutorial;

static bool say_hello_callback(SayHello::Request & request, SayHello::Response & response);


int main(int argc, char*argv[])
{
  ros::init(argc, argv, "hello_server");
  ros::NodeHandle nn("~");
  
  ROS_INFO ("creating callback");
  ros::ServiceServer
    say_hello_server(nn.advertiseService("say_hello", say_hello_callback));
  
  ROS_INFO ("entering event loop");
  ros::spin();
}


bool say_hello_callback(SayHello::Request & request,
			SayHello::Response & response)
{
  for (uint8_t ii(0); ii < request.count; ++ii) {
    std::cout << request.hello << "\n";
  }
  response.reply = request.hello + " to you!";
  return true;
}
