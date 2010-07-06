/* Copyright (C) 2010 ETHZ --- Swiss Federal Institute of Technology Zurich
 * http://www.asl.ethz.ch/
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <asl_velodyne/calibration.h>
#include <velodyne_common/RawScan.h>
#include <sensor_msgs/PointCloud.h>
#include <err.h>

static std::string default_calibfile(""); // constructed at runtime using pkg path
static int const default_qdepth(1);

static std::string calibfile("");
static std::string raw_topic("velodyne/rawscan"); // XXXX to do: add an option to change this
static std::string pc_topic("asl_velodyne/pointcloud"); // XXXX to do: add an option to change this
static bool restamp(false);
static int qdepth(default_qdepth);

static ros::Subscriber raw_sub;
static ros::Publisher pc_pub;
static sensor_msgs::PointCloud pc_msg;
static asl_velodyne::Calibration calib;


static void usage(char const * msg, int exitcode) 
{
  errx(exitcode,
       "%s\n"
       "Convert raw Velodyne scans to PointClouds\n"
       "\t-h            print usage message\n"
       "\t-n            re-stamp messages with ros::Time::now()\n"
       "\t-f <filename> calibration file (default: %s)\n"
       "\t-q <integer>  set ROS topic queue depth (default: %d)\n",
       msg, default_calibfile.c_str(), default_qdepth);
}


void parse_options(int argc, char ** argv)
{
  std::string const prefix(ros::package::getPath("asl_velodyne"));
  default_calibfile = prefix + "/etc/calibration.txt";
  
  char ch;
  const char* optflags = "hnf:q:";
  while (-1 != (ch = getopt(argc, argv, optflags))) {
    switch(ch) {
    case 'h':
      usage("usage", EXIT_SUCCESS);
    case 'n':
      restamp = true;
      ROS_INFO("restamping messages");
      break;
    case 'f':
      calibfile = optarg;
      ROS_INFO("calibration file %s", calibfile.c_str());
      break;
    case 'q':
      qdepth = atoi(optarg);
      if (qdepth < 1)
	qdepth = 1;
      ROS_INFO("queue depth %d", qdepth);
      break;
    default:
      usage("invalid option", EXIT_FAILURE);
    }
  }
  
  if (calibfile.empty()) {
    calibfile = default_calibfile;
  }
}


static void raw_cb(velodyne_common::RawScan::ConstPtr const & raw)
{
}


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "cloud");
  parse_options(argc, argv);
  
  int const status(calib.load(calibfile));
  if (0 != status) {
    errx(EXIT_FAILURE,
	 "failed to load calibration file `%s' (error code %d)",
	 calibfile.c_str(), status);
  }
  
  ros::NodeHandle nh;
  raw_sub = nh.subscribe(raw_topic, qdepth, raw_cb, ros::TransportHints().tcpNoDelay(true));
  pc_pub = nh.advertise<sensor_msgs::PointCloud>(pc_topic, qdepth);
  
  ros::spin();
}