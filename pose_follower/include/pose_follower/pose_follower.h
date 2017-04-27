/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef POSE_FOLLOWER_POSE_FOLLOWER_H_
#define POSE_FOLLOWER_POSE_FOLLOWER_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/trajectory_planner_ros.h>

class lpf
{
  public:
    float filter(const float& new_msrmt);
    float c_ = 1.0; // Related to the cutoff frequency of the filter.
      // c=1 results in a cutoff at 1/4 of the sampling rate.
      // See bitbucket.org/AndyZe/pid if you want to get more sophisticated.
      // Larger c --> trust the filtered data more, trust the measurements less --> lower cutoff frequency

  private:
    float prev_msrmts_ [3] = {0., 0., 0.};
    float prev_filtered_msrmts_ [2] = {0., 0.};
};

float lpf::filter(const float& new_msrmt)
{
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  float new_filtered_msrmt = (1/(1+c_*c_+1.414*c_))*(prev_msrmts_[2]+2*prev_msrmts_[1]+prev_msrmts_[0]-(c_*c_-1.414*c_)*prev_filtered_msrmts_[1]-(-2*c_*c_+2)*prev_filtered_msrmts_[0]);;

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}

namespace pose_follower {
  class PoseFollower : public nav_core::BaseLocalPlanner {
    public:
      PoseFollower();
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool isGoalReached();
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }

      geometry_msgs::Twist diff2D(const tf::Pose& pose1, const tf::Pose&  pose2);
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);

      bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
          const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
          std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool stopped();

      tf::TransformListener* tf_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      ros::Publisher vel_pub_;
      double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
      double tolerance_timeout_;
      double max_vel_lin_, max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      bool allow_backwards_;
      bool turn_in_place_first_;
      double max_heading_diff_before_moving_;
      bool holonomic_;
      boost::mutex odom_lock_;
      ros::Subscriber odom_sub_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
      ros::Time goal_reached_time_;
      unsigned int current_waypoint_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      base_local_planner::TrajectoryPlannerROS collision_planner_;
      int samples_;
      // Low pass filter
      lpf linear_x_filter_;
      lpf linear_y_filter_;
      lpf linear_z_filter_;
      lpf rot_x_filter_;
      lpf rot_y_filter_;
      lpf rot_z_filter_;
  };
};
#endif
