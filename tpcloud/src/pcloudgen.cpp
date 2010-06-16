#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Geometry>
#include <vector>
#include <stdio.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcloudgen");
  ros::NodeHandle nn;
  ros::Publisher cube_pub;
  cube_pub = nn.advertise<sensor_msgs::PointCloud>("pcloudgen/cube", 1);
  
  std::vector<Eigen::Vector3d> cube;
  for (size_t ii(0); ii < 6; ++ii) {
    double const dd((ii + 0.5) / 6.0);
    // four struts along X
    cube.push_back(Eigen::Vector3d(dd, 0, 0));
    cube.push_back(Eigen::Vector3d(dd, 1, 0));
    cube.push_back(Eigen::Vector3d(dd, 0, 1));
    cube.push_back(Eigen::Vector3d(dd, 1, 1));
    // four struts along Y
    cube.push_back(Eigen::Vector3d(0, dd, 0));
    cube.push_back(Eigen::Vector3d(1, dd, 0));
    cube.push_back(Eigen::Vector3d(0, dd, 1));
    cube.push_back(Eigen::Vector3d(1, dd, 1));
    // four struts along Z
    cube.push_back(Eigen::Vector3d(0, 0, dd));
    cube.push_back(Eigen::Vector3d(1, 0, dd));
    cube.push_back(Eigen::Vector3d(0, 1, dd));
    cube.push_back(Eigen::Vector3d(1, 1, dd));
  }
  
  Eigen::Vector3d axis(1, 0, 0);
  double angle(0);
  double angle_inc(0.5 * M_PI / 180);
  double angle_max(30 * M_PI / 180);
  sensor_msgs::PointCloud cube_msg;
  cube_msg.header.seq = 0;
  cube_msg.header.frame_id = "/cube";
  cube_msg.points.resize(cube.size());
  
  while (ros::ok()) {
    Eigen::Transform<double, 3> tt(Eigen::AngleAxis<double>(angle, axis));
    cube_msg.header.stamp = ros::Time::now();
    for (size_t jj(0); jj < cube.size(); ++jj) {
      Eigen::Vector3d cc(tt * cube[jj]);
      cube_msg.points[jj].x = cc.x();
      cube_msg.points[jj].y = cc.y();
      cube_msg.points[jj].z = cc.z();
      // fprintf(stderr, "%d: %g %g %g\n", jj, cc.x(), cc.y(), cc.z());
    }
    cube_pub.publish(cube_msg);
    
    if (fabs(angle) >= angle_max) {
      angle_inc = -angle_inc;
    }
    angle += angle_inc;
    ++cube_msg.header.seq;
    
    usleep(100000);
  }
}
