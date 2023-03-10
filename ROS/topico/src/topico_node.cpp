// ---------------------------------------------------------------------
// Package:    TopiCo (https://github.com/AIS-Bonn/TopiCo)
// Version:    2021-03-30 12:12:52
// Maintainer: Marius Beul (mbeul@ais.uni-bonn.de)
// License:    BSD
// ---------------------------------------------------------------------

// Software License Agreement (BSD License)
// Copyright (c) 2021, Computer Science Institute VI, University of Bonn
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of University of Bonn, Computer Science Institute
//   VI nor the names of its contributors may be used to endorse or
//   promote products derived from this software without specific
//   prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <topico/TopiCoConfig.h>

#include "rt_nonfinite.h"
#include "topico_wrapper.h"
#include "topico_wrapper_terminate.h"
#include "topico_wrapper_types.h"
#include "coder_array.h"
#include <sstream>
#include <stdexcept>
#include <string>
#include "topico/Waypoints.h"

struct CTX {
  // Initial state of the axes
  coder::array<double, 2U> initial_state;
  // Target waypoints
  coder::array<double, 3U> waypoints;
  // Max/Min velocities/jerks/accelerations, per axis, per waypoint
  coder::array<double, 2U> velocity_max;
  coder::array<double, 2U> velocity_min;
  coder::array<double, 2U> acceleration_max;
  coder::array<double, 2U> acceleration_min;
  coder::array<double, 2U> jerk_max;
  coder::array<double, 2U> jerk_min;
  // Global acceleration (?)
  coder::array<double, 1U> acceleration_global;
  // If the velocity/acceleration/jerk should be synced between the fastest & slowest axes (slows fastest axes to sync slowest ones)
  coder::array<bool, 2U> sync_velocities;
  coder::array<bool, 2U> sync_accelerations;
  coder::array<bool, 2U> sync_jerks;
  // If axes should be synced by waiting
  coder::array<bool, 2U> sync_axes;
  // Rotates axes to point towards the movement between waypoints
  coder::array<bool, 2U> rotate_axes;
  // Minimize time allowed outside velocity limits (alternative: more time outside limits but smoother movements)
  coder::array<bool, 2U> strict_velocity_limits;
  // If synchronized axes should try to catch up with unsynchronized ones
  coder::array<bool, 2U> catch_up_synced_axes;
  // (?)
  coder::array<signed char, 2U> direction;
  // Timesteps when the trajectory is sampled
  double sampling_timestamps;
  // If there's been an update
  bool updated;
  // If the CTX was initialized
  bool initialized;

  // Outputs
  coder::array<struct0_T, 2U> J_setp_struct;
  coder::array<int, 2U> solution_out;
  coder::array<double, 2U> T_waypoints;
  coder::array<double, 2U> P;
  coder::array<double, 2U> V;
  coder::array<double, 2U> A;
  coder::array<double, 2U> J;
  coder::array<double, 2U> t;

  // ROS stuff
  ros::NodeHandle elevator_node;
  ros::NodeHandle fourbar_node;

  CTX() :
    sampling_timestamps{0.1},
    updated{false},
    initialized{false},
    elevator_node("/frcrobot_jetson/elevator_controller_2023"),
    fourbar_node("/frcrobot_jetson/four_bar_controller_2023")
  {}

  // Resize all the arrays, eg for new waypoints/axes
  void resize(int axes, int waypoints) {
    waypoints--;
    this->initial_state.set_size(axes, 3);
    this->waypoints.set_size(axes, 5, waypoints);
    this->velocity_max.set_size(axes, waypoints);
    this->velocity_min.set_size(axes, waypoints);
    this->acceleration_max.set_size(axes, waypoints);
    this->acceleration_min.set_size(axes, waypoints);
    this->jerk_max.set_size(axes, waypoints);
    this->jerk_min.set_size(axes, waypoints);
    this->acceleration_global.set_size(axes);
    this->sync_velocities.set_size(axes, waypoints);
    this->sync_accelerations.set_size(axes, waypoints);
    this->sync_jerks.set_size(axes, waypoints);
    this->sync_axes.set_size(axes, waypoints);
    this->rotate_axes.set_size(axes - 1, waypoints);
    this->strict_velocity_limits.set_size(axes, waypoints);
    this->catch_up_synced_axes.set_size(axes, waypoints);
    this->direction.set_size(axes, waypoints);

    double elevator_max_velocity;
    double elevator_max_acceleration;
    double fourbar_max_velocity;
    double fourbar_max_acceleration;
    this->elevator_node.getParam("motion_magic_velocity", elevator_max_velocity);
    this->elevator_node.getParam("motion_magic_acceleration", elevator_max_acceleration);
    this->fourbar_node.getParam("motion_magic_velocity", fourbar_max_velocity);
    this->fourbar_node.getParam("motion_magic_acceleration", fourbar_max_acceleration);
    // idx_dim + num_dim * idx_wayp
    for (int waypoint_id = 0; waypoint_id < waypoints; waypoint_id++) {
      this->velocity_max[0 + axes * waypoint_id] = elevator_max_velocity;
      this->velocity_min[0 + axes * waypoint_id] = 0;
      this->acceleration_max[0 + axes * waypoint_id] = elevator_max_acceleration;
      this->acceleration_min[0 + axes * waypoint_id] = 0;
      this->velocity_max[1 + axes * waypoint_id] = fourbar_max_velocity;
      this->velocity_min[1 + axes * waypoint_id] = 0;
      this->acceleration_max[1 + axes * waypoint_id] = fourbar_max_acceleration;
      this->acceleration_min[1 + axes * waypoint_id] = 0;

      for (int axis_id = 0; axis_id < axes; axis_id++) {
        this->direction[axis_id + axes * waypoint_id] = 1;
        this->jerk_min[axis_id + axes * waypoint_id] = 0;
        this->jerk_max[axis_id + axes * waypoint_id] = 2;
        this->acceleration_global[axis_id] = 0;
        this->sync_accelerations[axis_id + axes * waypoint_id] = false;
        this->sync_axes[axis_id + axes * waypoint_id] = false;
        this->sync_jerks[axis_id + axes * waypoint_id] = false;
        this->sync_velocities[axis_id + axes * waypoint_id] = false;
        this->catch_up_synced_axes[axis_id + axes * waypoint_id] = false;
      }
    }
  }

  // (Re)initialize the CTX
  void init(const topico::WaypointsConstPtr& waypoints) {
    // Number of axes in each waypoint
    const int AXES = waypoints->waypoints[0].positions.size();
    const int WAYPOINTS = waypoints->waypoints.size();
    this->resize(AXES, WAYPOINTS);

    for (int i = 0; i < AXES; i++) {
        // Position X/Y/Z (respectively)
        this->initial_state[i + (AXES * 0)] = waypoints->waypoints[0].positions[i];
        // Velocity X/Y/Z (respectively, starts at 0 since nothing's moving)
        this->initial_state[i + (AXES * 1)] = 0;
        // Acceleration X/Y/Z (respectively)
        this->initial_state[i + (AXES * 2)] = 0.0;
    }

    for (int waypoint = 1; waypoint < WAYPOINTS; waypoint++) {
        for (int axis = 0; axis < AXES; axis++) {
            // Pos
            this->waypoints[(axis + AXES * 0) + AXES * 5 * (waypoint - 1)] = waypoints->waypoints[waypoint].positions[axis];
            // Vel
            this->waypoints[(axis + AXES * 1) + AXES * 5 * (waypoint - 1)] = std::numeric_limits<double>::quiet_NaN();
            // Accel
            this->waypoints[(axis + AXES * 2) + AXES * 5 * (waypoint - 1)] = std::numeric_limits<double>::quiet_NaN();
            // Movement vel
            this->waypoints[(axis + AXES * 3) + AXES * 5 * (waypoint - 1)] = 0.0;
            // Reserved
            this->waypoints[(axis + AXES * 4) + AXES * 5 * (waypoint - 1)] = 0.0;
        }
    }

    topico_wrapper(
      // State_start
      this->initial_state,
      // Waypoints
      this->waypoints,
      // V_max
      this->velocity_max,
      // V_min
      this->velocity_min,
      // A_max
      this->acceleration_max,
      // A_min
      this->acceleration_min,
      // J_max
      this->jerk_max,
      // J_min
      this->jerk_min,
      // A_global
      this->acceleration_global,
      // b_sync_V
      this->sync_velocities,
      // b_sync_A
      this->sync_accelerations,
      // b_sync_J
      this->sync_jerks,
      // b_sync_W
      this->sync_axes,
      // b_rotate
      this->rotate_axes,
      // b_hard_V_lim
      this->strict_velocity_limits,
      // b_catch_up
      this->catch_up_synced_axes,
      // direction
      this->direction,
      // ts_rollout
      this->sampling_timestamps,
      // J_setp_struct
      this->J_setp_struct,
      // solution_out
      this->solution_out,
      // T_waypoints
      this->T_waypoints,
      // P
      this->P,
      // V
      this->V,
      // A
      this->A,
      // J
      this->J,
      // t
      this->t
    );

    // It was updated
    this->updated = true;
    // It was initialized
    this->initialized = true;
  }
};


// void dynamic_reconfigure_callback(topico::TopiCoConfig &config, uint32_t level)
// {
//   for (int idx_dim = 0; idx_dim < num_dim; idx_dim++) {  
//     A_global[idx_dim] = 0.0; 
//     for (int idx_wayp = 0; idx_wayp < num_wayp; idx_wayp++) {
//       V_max[idx_dim + num_dim * idx_wayp]        = config.V_max;
//       V_min[idx_dim + num_dim * idx_wayp]        = config.V_min;
//       A_max[idx_dim + num_dim * idx_wayp]        = config.A_max;
//       A_min[idx_dim + num_dim * idx_wayp]        = config.A_min;
//       J_max[idx_dim + num_dim * idx_wayp]        = config.J_max;
//       J_min[idx_dim + num_dim * idx_wayp]        = config.J_min;
//       b_sync_V[idx_dim + num_dim * idx_wayp]     = config.b_sync_V;
//       b_sync_A[idx_dim + num_dim * idx_wayp]     = config.b_sync_A;
//       b_sync_J[idx_dim + num_dim * idx_wayp]     = config.b_sync_J;
//       b_sync_W[idx_dim + num_dim * idx_wayp]     = config.b_sync_W;
//       b_hard_V_lim[idx_dim + num_dim * idx_wayp] = config.b_hard_V_lim;
//       b_catch_up[idx_dim + num_dim * idx_wayp]   = config.b_catch_up;
//       direction[idx_dim + num_dim * idx_wayp]    = config.direction;
//     }
//   }  
//   for (int idx_dim = 0; idx_dim < num_dim-1; idx_dim++) {  
//     for (int idx_wayp = 0; idx_wayp < num_wayp; idx_wayp++) {
//       b_rotate[idx_dim + num_dim * idx_wayp]     = config.b_rotate;
//     }
//   }
//   ts_rollout = config.ts_rollout;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "topico");
  ros::NodeHandle nh("/topico");
  ros::Rate loop_rate(100);
  CTX ctx;
  
  ros::Subscriber wayp_sub = nh.subscribe("wayp_odometry", 1, &CTX::init, &ctx, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber init_sub = nh.subscribe("init_odometry", 1, init_callback, ros::TransportHints().tcpNoDelay());
  ros::Publisher path_pub  = nh.advertise<nav_msgs::Path>("trajectory_rollout", 0);
  
  std::string map_frame;
  nh.param<std::string>( "frame_id", map_frame, "world" );
  
  // dynamic_reconfigure::Server<topico::TopiCoConfig> server;
  // dynamic_reconfigure::Server<topico::TopiCoConfig>::CallbackType f;
  // f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  // server.setCallback(f);
 
  nav_msgs::Path path_rollout;
  
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time t_now = ros::Time::now();

    if (ctx.initialized && ctx.updated) { // only replan when new data arrived...
      ctx.updated = false;

      int size_rollout = ctx.P.size(1);
      path_rollout.poses.resize(size_rollout);
      path_rollout.header.stamp = t_now;
      path_rollout.header.frame_id = map_frame;
      for (int idx = 0; idx < size_rollout; idx++)
      {
        path_rollout.poses[idx].pose.position.x = ctx.P[3*idx+0];
        path_rollout.poses[idx].pose.position.y = ctx.P[3*idx+1];
        path_rollout.poses[idx].pose.position.z = ctx.P[3*idx+2];
      }
      path_pub.publish(path_rollout);
    } else if(!ctx.initialized) {
      printf("Warning: Initial state and/or waypoint not published yet!\n");       
    }
    loop_rate.sleep();
  }

  topico_wrapper_terminate();
  return 0;
}

