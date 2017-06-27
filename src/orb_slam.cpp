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

#include "orb_slam.h"

using namespace orb_slam_wrp;

OrbSlam::OrbSlam(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  timer_ = nh_.createTimer(ros::Duration(1.0), &OrbSlam::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &OrbSlam::updateParams, this);

  { // Initialize parameters
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  orb_slam_ = new ORB_SLAM2::System(p_vocabluary_file_, p_camera_params_file_, ORB_SLAM2::System::MONOCULAR, p_display_window_);

  camera_ = 1;
  raw_data_ = NULL;
  memory_id_ = 0;

  is_InitCamera(&camera_, NULL);
  is_AllocImageMem(camera_, p_width_, p_height_, 24, &raw_data_, &memory_id_);
  is_SetImageMem(camera_, raw_data_, memory_id_);
}

OrbSlam::~OrbSlam() {
  orb_slam_->Shutdown();
}

bool OrbSlam::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("display_window", p_display_window_, false);

  nh_local_.param<int>("width", p_width_, 792);
  nh_local_.param<int>("height", p_height_, 480);

  nh_local_.param<double>("sampling_time", p_sampling_time_, 0.1);

  nh_local_.param<string>("camera_file", p_camera_params_file_, "/home/ksis/CameraParams.yaml");
  nh_local_.param<string>("vocabluary_file", p_vocabluary_file_, "/home/ksis/ORB_SLAM2/Vocabulary/ORBvoc.txt");

  nh_local_.param<string>("parent_frame", p_parent_frame_, "world");
  nh_local_.param<string>("child_frame", p_child_frame_, "orb_slam");

  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_) {
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("orb_slam_pose", 5);
    timer_.start();
  }
  else {
    pose_pub_.shutdown();
    timer_.stop();
  }

  return true;
}

void OrbSlam::timerCallback(const ros::TimerEvent& e) {
  IplImage* tmpImg;

  is_CaptureVideo(camera_, p_sampling_time_ * 1000.0);

  tmpImg = cvCreateImageHeader(cvSize(p_width_, p_height_), IPL_DEPTH_8U, 3);
  tmpImg->imageData = raw_data_;

  cv::Mat image = cv::cvarrToMat(tmpImg, false);

  cv::Mat tf = orb_slam_->TrackMonocular(image, e.current_real.toSec());
}
