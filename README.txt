README for ASL ROS Sandbox
==========================

**************************************************
http://github.com/poftwaresatent/asl-ros-sandbox

This repository contains code to play around with various aspects of
ROS http://www.ros.org/ for the Autonomous Systems Lab of ETH Zurich
http://www.asl.ethz.ch/
**************************************************

Contents of the +rosinstalls+ Directory
---------------------------------------

Example rosinstall configuration files that should make it easier for
projects at ASL to create their own source-based installations of ROS,
depending on the needs of specific projects.

For example, in order to use
rosinstalls/cturtle_limno_helios.rosinstall (which presumably lies on
your harddisk somewhere), just do:

1. wget http://ros.org/rosinstall
2. chmod u+x rosinstall
3. ./rosinstall /desired/path/to/ros/tree /path/to/cturtle_limno_helios.rosinstall
4. drink coffee
5. source /desired/path/to/ros/tree/setup.sh


Contents of the +asltutorial+ Directory
---------------------------------------

Examples used for a high-level walkthrough of ROS:

- service server implemented in C++
- service client from command line (rosservice)
- service client with GUI implemented in Python with Tix
- example on-the-fly graphical visualization (rxplot)

See asltutorial/mainpage.dox for more info.


Contents of the +winch_action+ Directory
----------------------------------------

Actionlib server and client code written with Stephane Magnenat during
a one-day hackathon to learn actionlib in order to apply it to the
limnobotics project with Gregory Hitz.

Contains actionlib servers and clients, from simplistic to mildly
complex. Shows an important difference between simple and non-simple
servers, the latter are much more useful (otherwise you might almost
just prefer to use service servers).

See winch_action/mainpage.dox for more info.


Contents of the +non_simple_action+ Directory
---------------------------------------------

Exploring the possibilities and limits of non-simple action servers
and clients, in Python and C+\+. At the time of writing, not all was
well with the non-simple versions of action clients in C++ and
Python. The servers seemed to behave as expected though.

See non_simple_action/mainpage.dox for more info.


Contents of the +pingpong+ Directory
------------------------------------

Tool for exploring how ROS reacts to unreliable networking. The "ping"
side sends incrementing sequence numbers, the "pong" side gets the
pings and sends back the same sequence number.  Both sides publish
stats which summarize the difference between sent and received
sequence numbers.

See pingpong/mainpage.dox for more info.


Contents of the +tpcloud+ Directory
-----------------------------------

A tool for testing your point cloud visualization. It is essentially
aimed at debugging your rviz setup, by sending 3D point clouds whose
shape is known.

See tpcloud/mainpage.dox for more info.


Contents of the +utexas-art-velodyne+ Directory
-----------------------------------------------

A fork of the Velodyne HDL-64E 3D LIDAR support by the University of
Texas, maintained by Jack O'Quin. At ASL we have a Velodyne that uses
a different scan-to-3D-point transform, and the idea was to patch this
driver uch that it can use both ours and the UTexas method.

Based on
http://utexas-art-ros-pkg.googlecode.com/svn/trunk/stacks/velodyne
revision 233
