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

#include <plp_basic_cost_functions/velocity_costmaps_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {

void VelocityCostmapsCostFunction::initialize(std::string base_name, std::string plugin_name, plugin_local_planner::LocalPlannerUtil *planner_util) {
    plugin_local_planner::TrajectoryCostFunction::initialize(base_name, plugin_name, planner_util);

    costs_xv_ = 0.0;
    costs_tv_ = 0.0;

    ros::NodeHandle nh("~/" + name_);
    sub = nh.subscribe("/velocity_costmap_server/map", 10, &VelocityCostmapsCostFunction::callback, this);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);

    update_parameters();
}

VelocityCostmapsCostFunction::~VelocityCostmapsCostFunction() {}


bool VelocityCostmapsCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
                                           tf::Stamped<tf::Pose> global_vel,
                                           std::vector<geometry_msgs::Point> footprint_spec) {
    update_parameters();
    if(pub.getNumSubscribers() && map_.info.width != 0){
        pub.publish(pub_map_);
    }
    return true;
}

void VelocityCostmapsCostFunction::update_parameters(){
    ros::NodeHandle nh("~/" + base_name_);
    nh.param("max_vel_x", max_vel_x_, 1.0);
}

double VelocityCostmapsCostFunction::scoreTrajectory(Trajectory &traj) {
    boost::mutex::scoped_lock lock(mutex);
    double cost = 0;
    if (sub.getNumPublishers() && map_.info.width != 0 && map_.info.height != 0) {
        double xv = traj.xv_ * 100;
        double tv = traj.thetav_ * M_PI;
        int x = xv * cos(tv);
        int y = xv * sin(tv);

        unsigned int index = VelocityCostmapsCostFunction::getIndex((x+(map_.info.width/2))-1, (y+(map_.info.height/2))-1, map_.info.width);
        costs_xv_ = double(map_.data[index]);
        costs_tv_ = double(map_.data[index]);
        cost = fmax(traj.xv_ * costs_xv_,fabs(traj.thetav_ * costs_tv_));
//        cost += fabs(traj.thetav_ * costs_tv_);

        //  ROS_INFO("Speed: x: %f, theta: %f, index: (%d, %d) = %d -> costs: %.2f", traj.xv_, traj.thetav_, x, y, index, cost);
        if(pub.getNumSubscribers()){
            pub_map_.data[index] = 98; // Just for visualisation, these costs have no effect on the planning
        }
        VelocityCostmapsCostFunction::check_map();
    }
    return cost;
}

void VelocityCostmapsCostFunction::callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    boost::mutex::scoped_lock lock(mutex);
    map_ = *msg;
    pub_map_ = map_;
}
} /* namespace plp_basic_cost_functions */

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::VelocityCostmapsCostFunction, plugin_local_planner::TrajectoryCostFunction)
