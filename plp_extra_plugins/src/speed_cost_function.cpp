/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <plp_extra_plugins/speed_cost_function.h>
#include <angles/angles.h>
using plugin_local_planner::Trajectory;

namespace plp_extra_plugins {

void SpeedCostFunction::initialize(std::string base_name, std::string plugin_name, plugin_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(base_name, plugin_name, planner_util);

    ros::NodeHandle nh("~/" + name_);
    nh.param("target_speed", target_speed_, 0.5);
    nh.param("command_factor", command_factor_, 1.0);
    nh.param("x_speed_only", x_speed_only_, true);
}

bool SpeedCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  return true;
}


double SpeedCostFunction::scoreTrajectory(Trajectory &traj) {
 
  double abs_speed;
  if(x_speed_only_){
    abs_speed = traj.xv_;
  } else {
    abs_speed = sqrt( pow(traj.xv_, 2) + pow(traj.yv_, 2) );
  }
  return fabs(abs_speed - target_speed_ / command_factor_);
}

} /* namespace plp_extra_plugins */

PLUGINLIB_EXPORT_CLASS(plp_extra_plugins::SpeedCostFunction, plugin_local_planner::TrajectoryCostFunction)
