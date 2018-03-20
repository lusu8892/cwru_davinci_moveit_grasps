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

#ifndef CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_FILTER_H
#define CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_FILTER_H

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// Grasping
#include <cwru_davinci_moveit_grasps/davinci_grasp_generator.h>
//#include <cwru_davinci_moveit_grasps/grasp_candidate.h>

// Rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// C++
#include <boost/thread.hpp>
#include <math.h>

namespace davinci_moveit_grasps
{

  struct IkThreadStruct
  {
    IkThreadStruct(std::vector<GraspCandidatePtr> &grasp_candidates,  // the input
                   planning_scene::PlanningScenePtr planning_scene,
                   Eigen::Affine3d &link_transform,
                   std::size_t grasp_id,
                   kinematics::KinematicsBaseConstPtr kinematic_solver,
                   robot_state::RobotStatePtr robot_state,
                   double timeout,
                   bool filter_pregrasp,
                   bool verbose,
                   std::size_t thread_id)
      : grasp_candidates_(grasp_candidates),
        planning_scene_(planning_scene),
        link_transform_(link_transform),
        grasp_id_(grasp_id),
        kinematic_solver_(kinematic_solver),
        robot_state_(robot_state),
        timeout_(timeout),
        filter_pregrasp_(filter_pregrasp),
        verbose_(verbose),
        thread_id_(thread_id)
    {
      // blank
    }
    std::vector<GraspCandidatePtr>& grasp_candidates_;
    planning_scene::PlanningScenePtr planning_scene_;
    Eigen::Affine3d link_transform_;
    std::size_t grasp_id_;
    kinematics::KinematicsBaseConstPtr kinematic_solver_;
    robot_state::RobotStatePtr robot_state_;
    const robot_model::JointModelGroup* arm_jmg_;
    double timeout_;
    bool filter_pregrasp_;
    bool verbose_;
    std::size_t thread_id_;

    // Used within processing function
    geometry_msgs::PoseStamped ik_pose_;
    moveit_msgs::MoveItErrorCodes error_code_;
    std::vector<double> ik_seed_state_;
  };

  typedef boost::share_ptr<IkThreadStruct> IkThreadStructPtr;

  class DavinciGraspFilter
  {
  public:
    DavinciGraspFilter(robot_state::RobotStatePtr robot_state, moveit_visual_tools::MoveItVisualToolsPtr &visual_tool);

    /**
     * @brief return grasps that aare kinematically feasible
     * @param grasp_candidates - all possible grasps that this will test. this vector is returned modified
     * @param planning_scene_monitor
     * @param arm_jmg - the arm to solve the IK problem on
     * @param seed_state
     * @param filter_pregrasp - whether to also check ik feasibility for the pregrasp position
     * @return number of grasps remaining
     */
    bool filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                      const robot_model::JointModelGroup * arm_jmg, const moveit::core::RobotStatePtr seed_state,
                      bool filter_pregrasp = false);

    /**
     * @brief filter grasps by desired orientation.
     * @param grasp_candidate - all possible grasps that this will test. this vector is returned modified
     * @param desired_pose - the desired grasp pose ( using standard grasping orientation )
     * @param max_angular_offset - maximum angle allowed between the grasp pose and the desired pose
     * @return true if grasp is filtered by operation
     */
    bool filterGraspByOrientation(GraspCandidatePtr grasp_candidate,
                                  Eigen::Affine3d desired_pose,
                                  double max_angular_offset);


    /**
   * \brief Helper for filterGrasps
   * \return number of grasps remaining
   */
    std::size_t filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                   const robot_model::JointModelGroup* arm_jmg,
                                   const moveit::core::RobotStatePtr seed_state, bool filter_pregrasp, bool verbose);

    /**
     * @brief thread for checking part of the posiible grasps list
     * @param ik_thread_struct
     * @return
     */
    bool processCandidateGrasp(IkThreadStructPtr& ik_thread_struct);



  };

}

#endif //CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_FILTER_H
