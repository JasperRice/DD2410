/*
 *  world_node.cpp
 *
 *
 *  Created on: Aug 25, 2014
 *  Authors:   Rares Ambrus
 *            raambrus <at> kth.se
 */

/* Copyright (c) 2014, Rares Ambrus, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>

// Boost includes
#include <stdio.h>
#include <stdlib.h>

// Random seed
#include <ctime>
bool reset_world = false;

void initWorld(ros::NodeHandle &n, tf::TransformListener &listener, visualization_msgs::Marker &wall_marker, double angle_z)
{
  tf::StampedTransform kobuki_transform;
  bool got_transform = false;

  while (!got_transform)
  {
    try
    {
      listener.lookupTransform("/base_link", "/odom", ros::Time(0), kobuki_transform);
      got_transform = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  tf::Quaternion q = kobuki_transform.getRotation();
  tf::Vector3 p = kobuki_transform.inverse().getOrigin();

  ROS_INFO_STREAM("Kobuki angle: " << q.getAngle());
  ROS_INFO_STREAM("Kobuki axis: " << q.getAxis().getX() << " " << q.getAxis().getY() << " " << q.getAxis().getZ());
  ROS_INFO_STREAM("Kobuki position: " << kobuki_transform.getOrigin().getX() << " " << kobuki_transform.getOrigin().getY() << " " << kobuki_transform.getOrigin().getZ());

  if(angle_z <= 5*M_PI/180.0 && angle_z >= 0.0) angle_z += 5*M_PI/180.0;
  if(angle_z >= -5*M_PI/180.0 && angle_z <= 0.0) angle_z -= 5*M_PI/180.0;
  angle_z += q.getAngle()*(-q.getAxis().getZ());

  KDL::Frame pose;
  p.setX(p.getX() - 0.4*sin(angle_z));
  p.setY(p.getY() + 0.4*cos(angle_z));
  tf::vectorTFToKDL(p, pose.p);
  pose.M = KDL::Rotation::RotZ(angle_z);
  // pose.p[1] += 0.4;

  tf::poseKDLToMsg(pose, wall_marker.pose);
}

bool reset(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  reset_world = true;
  return true;
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "world_node");
    ros::NodeHandle n;
    ros::Rate r(10); // 50 Hz
    visualization_msgs::Marker wall_marker;
    tf::TransformListener listener;

    // Create random number generators
    srand(time(NULL));
    double angle_z = ((double)(rand()%40)-20)*M_PI/180.0;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "wall_marker", 0 );
    ros::Publisher reset_pub = n.advertise<std_msgs::Empty>("reset_distance", 0);
    ros::ServiceServer service = n.advertiseService("reset_world", reset);

    wall_marker.header.frame_id = "/odom";
    wall_marker.header.stamp = ros::Time();
    wall_marker.ns = "world";
    wall_marker.id = 0;
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.scale.x = 500.0;
    wall_marker.scale.y = 0.01;
    wall_marker.scale.z = 0.2;
    wall_marker.color.a = 1.0;
    wall_marker.color.r = (255.0/255.0);
    wall_marker.color.g = (0.0/255.0);
    wall_marker.color.b = (0.0/255.0);
    initWorld(n, listener, wall_marker, angle_z);

    // Main loop.
    while (n.ok())
    {
        vis_pub.publish(wall_marker);
        if (reset_world)
        {
          srand(time(NULL));
          double angle_z = ((double)(rand()%40)-20)*M_PI/180.0;

          initWorld(n, listener, wall_marker, angle_z);
          reset_world = false;
          reset_pub.publish(std_msgs::Empty());
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
