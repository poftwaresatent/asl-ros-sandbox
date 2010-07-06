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
static int const default_in_qdepth(1);
static int const default_out_qdepth(260);

static std::string calibfile("");
static std::string raw_topic("velodyne/rawscan"); // XXXX to do: add an option to change this
static std::string pc_topic("asl_velodyne/pointcloud"); // XXXX to do: add an option to change this
static bool restamp(false);
static int in_qdepth(default_in_qdepth);
static int out_qdepth(default_out_qdepth);
static bool verbose(false);

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
       "\t-v            verbose operation\n"
       "\t-n            re-stamp messages with ros::Time::now()\n"
       "\t-f <filename> calibration file (default: %s)\n"
       "\t-q <integer>  set output queue depth (default: %d)\n"
       "\t-Q <integer>  set input queue depth (default: %d)\n",
       msg, default_calibfile.c_str(), default_out_qdepth, default_in_qdepth);
}


void parse_options(int argc, char ** argv)
{
  std::string const prefix(ros::package::getPath("asl_velodyne"));
  default_calibfile = prefix + "/etc/calibration.txt";
  
  char ch;
  const char* optflags = "hvnf:q:Q:";
  while (-1 != (ch = getopt(argc, argv, optflags))) {
    switch(ch) {
    case 'h':
      usage("usage", EXIT_SUCCESS);
    case 'v':
      verbose = true;
      ROS_INFO("verbose mode");
      break;
    case 'n':
      restamp = true;
      ROS_INFO("restamping messages");
      break;
    case 'f':
      calibfile = optarg;
      ROS_INFO("calibration file %s", calibfile.c_str());
      break;
    case 'q':
      out_qdepth = atoi(optarg);
      if (out_qdepth < 1)
	out_qdepth = 1;
      ROS_INFO("output queue depth %d", out_qdepth);
      break;
    case 'Q':
      in_qdepth = atoi(optarg);
      if (in_qdepth < 1)
	in_qdepth = 1;
      ROS_INFO("input queue depth %d", in_qdepth);
      break;
    default:
      usage("invalid option", EXIT_FAILURE);
    }
  }
  
  if (calibfile.empty()) {
    calibfile = default_calibfile;
  }
}


/** \return The number of points in the published pointcloud
    message. Errors are signaled via ros::shutdown(), which you can
    test by calling ros::ok(). */
static size_t convert_packet(roslib::Header const & msg_header,
			     uint8_t const * packet)
{
  pc_msg.points.resize(12 * 32); // make space for max number of pts
  size_t nvalid(0);		 // keep track of how many are valid
  uint8_t const * block(packet);
  for (size_t iblock(0); iblock < 12; ++iblock, block += 100) {
    
    size_t ray_begin, ray_end;
    uint16_t const header(*reinterpret_cast<uint16_t const *>(block));
    if (header == 0xEEFF) {
      // UPPER: from 0 to 31
      // ROS_DEBUG ("block %zu is UPPER", iblock);
      ray_begin = 0;
      ray_end = 32;
    }
    else if (header == 0xDDFF) {
      // LOWER: from 32 to 63
      // ROS_DEBUG ("block %zu is LOWER", iblock);
      ray_begin = 32;
      ray_end = 64;
    }
    else {
      ROS_ERROR ("unexpected header 0x%4X in block %zu",
		 header, iblock);
      ros::shutdown();
      return 0;
    }
    
    double const raw_angle(calib.cdeg_to_rad(*reinterpret_cast<uint16_t const *>(block + 2)));
    uint16_t const * raw_dist(reinterpret_cast<uint16_t const *>(block + 4));
    for (size_t iray(ray_begin); iray < ray_end; ++iray, raw_dist += 3) {
      double px, py, pz;
      int const status(calib.convert(raw_angle, iray, *raw_dist, px, py, pz));
      if (0 > status) {
	ROS_ERROR ("conversion failed for ray %zu in block %zu (error code %d)", iray - ray_begin, iblock, status);
	ros::shutdown();
	return 0;
      }
      else if (0 == status) {
	pc_msg.points[nvalid].x = px;
	pc_msg.points[nvalid].y = py;
	pc_msg.points[nvalid].z = pz;
	++nvalid;
      }
      // else {
      // 	ROS_DEBUG ("range too close or too far (code %d)", status);
      // }
    }
    // ROS_DEBUG ("block %zu nvalid %zu", iblock, nvalid);
  }
  
  if (0 == nvalid) {
    // ROS_DEBUG ("no valid points in raw scan");
    return 0;
  }
  
  pc_msg.header = msg_header;
  if (restamp) {
    pc_msg.header.stamp = ros::Time::now();
  }
  pc_msg.points.resize(nvalid);	// trim message down to actual nunmber of points
  pc_pub.publish(pc_msg);
  
  return nvalid;
}


static void raw_cb(velodyne_common::RawScan::ConstPtr const & raw)
{
  if (raw->data.size() % 1206 != 0) {
    ROS_ERROR ("raw data size %zu is not an integer multiple of 1206)",
	       raw->data.size());
    ros::shutdown();
    return;
  }
  
  size_t const npackets(raw->data.size() / 1206);
  uint8_t const * packet(&raw->data[0]);
  size_t ttnvalid(0);
  for (size_t ipacket(0); ipacket < npackets; ++ipacket, packet += 1206) {
    size_t nvalid(convert_packet(raw->header, packet));
    if ( ! ros::ok()) {
      ROS_ERROR ("problem occurred in packet %zu of %zu", ipacket, npackets);
      return;
    }
    ttnvalid += nvalid;
  }
  
  if (verbose) {
    ROS_INFO ("published %zu points from %zu packets", ttnvalid, npackets);
  }
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
  raw_sub = nh.subscribe(raw_topic, in_qdepth, raw_cb,
			 ros::TransportHints().tcpNoDelay(true));
  pc_pub = nh.advertise<sensor_msgs::PointCloud>(pc_topic, out_qdepth);
  
  ros::spin();
}
