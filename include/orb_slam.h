/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <System.h> // ORB_SLAM
#include <opencv2/opencv.hpp>
#include <ueye.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose2D.h>

namespace orb_slam_wrp
{

class OrbSlam
{
public:
  OrbSlam(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~OrbSlam();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent& e);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceServer params_srv_;
  ros::Timer timer_;
  ros::Publisher pose_pub_;

  HIDS camera_ = 1;
  char* raw_data_ = NULL;
  int memory_id_;

  ORB_SLAM2::System* orb_slam_;

  // Parameters
  bool p_active_;
  bool p_display_window_;

  int p_width_;
  int p_height_;

  double p_sampling_time_;

  std::string p_camera_params_file_;
  std::string p_vocabluary_file_;

  std::string p_parent_frame_;
  std::string p_child_frame_;
};

} // namespace orb_slam_wrp
