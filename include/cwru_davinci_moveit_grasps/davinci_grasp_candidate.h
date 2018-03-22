/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western Reserve University
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
 *   * Neither the name of SRI International nor the names of its
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
 *********************************************************************/

/* Author: Su Lu <sxl924@case.edu>
   Desc:   Filters grasps based on kinematic feasibility and collision
*/

#ifndef CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_CANDIDATE_H
#define CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_CANDIDATE_H

// ROS
#include <ros/ros.h>
#include <moveit_msgs/Grasp.h>

// Grasping
//#include <moveit_grasps/grasp_data.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>

namespace davinci_moveit_grasps
{
  class DavinciGraspCandidate
  {
  public:
    DavinciGraspCandidate(moveit_msgs::Grasp grasp, const GraspDataPtr grasp_data, Eigen::Affine3d cuboid_pose);

    moveit_msgs::Grasp grasp_;

    bool grasp_filtered_by_ik_;
    bool grasp_filtered_by_ik_closed_;      // ik solution was fine with fingers opened, but failed with fingers closed
    bool grasp_filtered_by_cutting_plane_;  // grasp pose is in an unreachable part of the environment (ex: inside
                                            // or behind a wall)
    bool grasp_filtered_by_orientation_;    // grasp pose is not desireable


  };

  typedef boost::shared_ptr<GraspCandidate> GraspCandidatePtr;
}  // namespace



#endif //CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_CANDIDATE_H
