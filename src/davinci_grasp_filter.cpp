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

// moveit_grasps
#include <cwru_davinci_moveit_grasps/davinci_grasp_filter.h>
//#include <moveit_grasps/state_validity_callback.h>

// moveit
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_tools.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <cwru_davinci_moveit_grasps/davinci_grasp_candidate.h>

namespace davinci_moveit_grasps
{
  // Constructor
  DavinciGraspFilter::DavinciGraspFilter(robot_state::RobotStatePtr robot_state,
                                         moveit_visual_tools::MoveItVisualToolsPtr &visual_tool)
    : visual_tools_(visual_tool), nh_("~/davinci_moveit_grasps/filter")
  {
    // make a copy of the robot state so that we are sure the outside influence does not break our grasp filter
    robot_state_.reset(new moveit::core::RobotState(*robot_state));
    robot_state_->update();  // make sure transforms are computed

    // Load visulization settings
    const std::string parent_name = "davinci_grasp_filter";
    rosparam_shortcuts::get(parent_name, nh_, "collision_verbose", collision_verbose_);
    rosparam_shortcuts::get(parent_name, nh_, "statistics_verbose", statistics_verbose_);
    rosparam_shortcuts::get(parent_name, nh_, "collision_verbose_speed", collision_verbose_speed_);
    rosparam_shortcuts::get(parent_name, nh_, "show_filtered_grasps", show_filtered_grasps_);
    rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions", show_filtered_arm_solutions_);
    rosparam_shortcuts::get(parent_name, nh_, "show_cutting_planes", show_cutting_planes_);
    rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions_speed", show_filtered_arm_solutions_speed_);
    rosparam_shortcuts::get(parent_name, nh_, "show_filtered_arm_solutions_pregrasp_speed",
                            show_filtered_arm_solutions_pregrasp_speed_);
    rosparam_shortcuts::get(parent_name, nh_, "show_grasp_filter_collision_if_failed",
                            show_grasp_filter_collision_if_failed_);

    ROS_INFO_STREAM_NAMED("davinci_grasp_filter", "GraspFilter Ready.");
  }

  bool DavinciGraspFilter::filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                                        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                        const robot_model::JointModelGroup *arm_jmg,
                                        const moveit::core::RobotStatePtr seed_state,
                                        bool filter_pregrasp = false)
  {
    bool verbose = false;

    // -----------------------------------------------------------------------------------------------
    // Error check
    if (grasp_candidates.empty())
    {
      ROS_ERROR_NAMED("grasp_filter", "Unable to filter grasps because vector is empty");
      return false;
    }
    if (!filter_pregrasp)
      ROS_WARN_STREAM_NAMED("grasp_filter", "Not filtering pre-grasp - GraspCandidate may have bad data");

    // -----------------------------------------------------------------------------------------------
    // Visualize the cutting planes if desired
    visualizeCuttingPlanes();

    // -----------------------------------------------------------------------------------------------
    // Get the solver timeout from kinematics.yaml
    solver_timeout_ = arm_jmg->getDefaultIKTimeout();
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Grasp filter IK timeout " << solver_timeout_);

    // -----------------------------------------------------------------------------------------------
    // Choose how many degrees of freedom
    num_variables_ = arm_jmg->getVariableCount();
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Solver for " << num_variables_ << " degrees of freedom");

    // -----------------------------------------------------------------------------------------------
    // Get the end effector joint model group
    if (arm_jmg->getAttachedEndEffectorNames().size() == 0)
    {
      ROS_ERROR_STREAM_NAMED("grasp_filter", "No end effectors attached to this arm");
      return false;
    }
    else if (arm_jmg->getAttachedEndEffectorNames().size() > 1)
    {
      ROS_ERROR_STREAM_NAMED("grasp_filter", "More than one end effectors attached to this arm");
      return false;
    }

    // Try to filter grasps not in verbose mode
    std::size_t remaining_grasps =
      filterGraspsHelper(grasp_candidates, planning_scene_monitor, arm_jmg, seed_state, filter_pregrasp, verbose);

    if (remaining_grasps == 0)
    {
      ROS_WARN_STREAM_NAMED("grasp_filter", "Grasp filters removed all grasps!");
      if (show_grasp_filter_collision_if_failed_)
      {
        ROS_INFO_STREAM_NAMED("grasp_filter", "Re-running in verbose mode since it failed");
        verbose = true;
        remaining_grasps =
          filterGraspsHelper(grasp_candidates, planning_scene_monitor, arm_jmg, seed_state, filter_pregrasp, verbose);
      }
      else
        ROS_INFO_STREAM_NAMED("grasp_filter", "NOT re-running in verbose mode");
    }

    // Visualize valid grasps as arrows with cartesian path as well
    if (show_filtered_grasps_)
    {
      ROS_INFO_STREAM_NAMED("grasp_filter", "Showing filtered grasps");
      visualizeGrasps(grasp_candidates, arm_jmg);
    }

    // Visualize valid grasp as arm positions
    if (show_filtered_arm_solutions_)
    {
      ROS_INFO_STREAM_NAMED("grasp_filter", "Showing filtered arm solutions");
      visualizeCandidateGrasps(grasp_candidates);
    }

    if (grasp_candidates.empty())
    {
      ROS_WARN_STREAM_NAMED("grasp_filter", "No grasps remaining after filtering");
      return false;
    }

    return true;

  }

  bool DavinciGraspFilter::filterGraspByPlane(GraspCandidatePtr grasp_candidate,
                                              const Eigen::Affine3d &filter_pose,
                                              grasp_parallel_plane plane,
                                              const int direction)
  {
    Eigen::Affine3d grasp_pose;
    Eigen::Vector3d grasp_position;

    // get grasp translationin in filter pose CS
    grasp_pose = visual_tools_->convertPose(grasp_candidate->grasp_.grasp_pose.pose);
    grasp_position = filter_pose.inverse() * grasp_pose.translation();  // TODO

    // filter grasps by cutting plane
    double epsilon = 10e-8;
    switch(plane)
    {
      case XY:

        break;
      case XZ:

        break;
      case YZ:

        break;
      default:
        ROS_WARN_STREAM_NAMED("filter_by_plane", "plane not specified correctly");
        break;
    }

    return grasp_candidate->grasp_filtered_by_cutting_plane_;
  }

  bool DavinciGraspFilter::filterGraspByOrientation(GraspCandidatePtr grasp_candidate,
                                                    const Eigen::Affine3d desired_pose,
                                                    double max_angular_offset)
  {
    Eigen::Affine3d standard_grasp_pose;
    Eigen::Affine3d grasp_pose;
    Eigen::Vector3d desired_z_axis;
    Eigen::Vector3d grasp_z_axis;

    double angle;

    // convert grasp pose back to standard grasping orientation
    grasp_pose = visual_tools_->convertPose(grasp_candidate->grasp_.grasp_pose.pose);
    standard_grasp_pose = grasp_pose * grasp_candidate->grasp_data_->grasp_pose_to_eef_pose.inverser();

    // compute the angel btw the z-axes of the desired and grasp poses
    grasp_z_axis = standard_grasp_pose.rotation() * Eigen::Vector3d(0, 0, 1);
    desired_z_axis  = desired_pose.rotation() * Eigen::Vector3d(0, 0, 1);

    angle = acos(grasp_z_axis.normalized().dot(desired_z_axis.normalized()));

    if(angle > max_angular_offset)
    {
      grasp_candidate->grasp_filtered_by_orientaiton_ = true;
      return true;
    }
    else
    {
      return false;
    }
  }

  std::size_t DavinciGraspFilter::filterGraspsHelper(std::vector<GraspCandidatePtr> &grasp_candidates,
                                                     planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                                     const robot_model::JointModelGroup *arm_jmg,
                                                     const moveit::core::RobotStatePtr seed_state,
                                                     bool filter_pregrasp,
                                                     bool verbose)
  {
    // -----------------------------------------------------------------------------------------------
    // Setup collision checking

    // Copy planning scene that is locked
    planning_scene::PlanningScenePtr cloned_scene;
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);
      cloned_scene = planning_scene::PlanningScene::clone(scene);
    }
  }

  bool DavinciGraspFilter::visualizeGrasps(const std::vector<GraspCandidatePtr> &grasp_candidates,
                                           const moveit::core::JointModelGroup *arm_jmg)
  {

  }


  bool DavinciGraspFilter::visualizeCandidateGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates)
  {

  }

}  // namespace
