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
 * Author: CDondrup
 *********************************************************************/

#ifndef VELOCITY_COSTMAPS_COST_FUNCTION_H_
#define VELOCITY_COSTMAPS_COST_FUNCTION_H_

#include <plugin_local_planner/trajectory_cost_function.h>

#include <plp_basic_cost_functions/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/thread/mutex.hpp>

namespace plp_basic_cost_functions {

/**
 * class ObstacleCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class VelocityCostmapsCostFunction : public plugin_local_planner::TrajectoryCostFunction {

public:
  VelocityCostmapsCostFunction() {}
  ~VelocityCostmapsCostFunction();

  virtual void initialize(std::string base_name, std::string plugin_name, plugin_local_planner::LocalPlannerUtil *planner_util);
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
  double scoreTrajectory(plugin_local_planner::Trajectory &traj);

  virtual float getCost(unsigned int cx, unsigned int cy){ return 0; }

  void callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

private:
  void update_parameters();

  inline unsigned int getIndex(unsigned int mx, unsigned int my, unsigned int size_x) const
  {
    return my * size_x + mx;
  }

  inline void check_map() {
      if(ros::Time::now() > map_.header.stamp + ros::Duration(10))
          map_.info.width = 0;
  }

  double max_vel_x_, costs_xv_, costs_tv_;
  ros::Subscriber sub;
  ros::Publisher pub;
  nav_msgs::OccupancyGrid map_, pub_map_;
  boost::mutex mutex;
};

} /* namespace plugin_local_planner */
#endif /* VELOCITY_COSTMAPS_COST_FUNCTION_H_ */
