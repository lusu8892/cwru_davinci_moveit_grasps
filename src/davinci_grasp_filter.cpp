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

    *robot_state_ = cloned_scene->getCurrentState();


    // -----------------------------------------------------------------------------------------------
    // Choose Number of cores
    std::size_t num_threads = omp_get_max_threads();
    if(num_threads > grasp_candidates.size())
    {
      num_threads = grasp_candidates.size();
    }

    // debug
    if(verbose || collision_verbose_)
    {
      num_threads = 1;
      ROS_WARN_STREAM_NAMED("grasp_filter", "Using only " << num_threads << " threads because verbose is true");
    }

    ROS_INFO_STREAM_NAMED("grasp_filter", "Filtering " << grasp_candidates.size() << " candidate grasps with "
                                                       << num_threads << " threads");

    // -----------------------------------------------------------------------------------------------
    // load kinematic solvers if not already loaded
    if(kin_solvers_[arm_jmg->getName()].size() != num_threads)
    {
      kin_solvers_[arm_jmg->getName()].clear();

      // create an ik solver for every thread
      for(std::size_t i = 0; i < num_threads; ++i)
      {
        kin_solvers_[arm_jmg->getName()].push_back(arm_jmg->getSolverInstance());

        // test to make sure we have a valid kinematic solver
        if(!kin_solvers_[arm_jmg->getName()][i])
        {
          ROS_ERROR_STREAM_NAMED("grasp_filter", "No kinematic solver found");
          return 0;
        }
      }
    }

    // robot stats -------------------------------------------------------------------------------
    // create a robot state for every thread
    if(robot_states_list_.size() != num_threads)
    {
      robot_states_list_.clear();
      for(std::size_t i = 0; i < num_threads; ++i)
      {
        // copy the previous robot state
        robot_states_list_.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*robot_state_)));
      }
    }
    else
    {
      for(std::size_t i = 0; i < num_threads; ++i)
      {
        // copu the previous robot state
        *(robot_states_list_[i]) = *robot_state_;
      }
    }

    // transform poses -------------------------------------------------------------------------------
    // bring the pose to the frame of the IK solver
    const std::string& ik_frame = kin_solvers_[arm_jmg->getName()][0]->getBaseFrame();
    Eigen::Affine3d link_transform;
    ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug",
                           "Frame transform from ik_frame: " << ik_frame << " and robot model frame: "
                                                             << robot_state_->getRobotModel()->getModelFrame());

    if(!moveit::core::Transforms::sameFrame(ik_frame, robot_state_->getRobotModel()->getModelFrame()))
    {
      const robot_model::LinkModel *lm = robot_state_->getLinkModel(
        (!ik_frame.empty() && ik_frame[0] == '/') ? ik_frame.substr(1) : ik_frame);

      if(!lm)
      {
        ROS_ERROR_STREAM_NAMED("grasp_filter", "Unable to find frame for link transform");
        return 0;
      }

      link_transform = robot_state_->getGlobalLinkTransform(lm).inverse();
    }


    // create the seed state vector
    std::vector<double> ik_seed_state;
    seed_state->copyJointGroupPositions(arm_jmg, ik_seed_state);

    // Thread data -------------------------------------------------------------------------------
    // allocate only once to increase performance
    std::vector<IkThreadStructPtr> ik_thread_structs;
    ik_thread_structs.resize(num_threads);
    for(std::size_t thread_id = 0; thread_id < num_threads; ++thread_id)
    {
      ik_thread_structs[thread_id].reset(
        new davinci_moveit_grasps::IkThreadStruct(grasp_candidates, cloned_scene, link_transform, 0,
                                                  kin_solvers_[arm_jmg->getName()][thread_id],
                                                  robot_states_list_[thread_id], solver_timeout_,
                                                  filter_pregrasp, verbose, thread_id));
      ik_thread_structs[thread_id]->ik_seed_state_ = ik_seed_state;
    }

    // Benchmark time
    ros::Time start_time;
    start_time = ros::Time::now();

    // -----------------------------------------------------------------------------------------------
    // Loop through poses and find those that are kinematically feasible

    omp_set_num_threads(num_threads);
    #pragma omp parallel for schedule(dynamic)
    for(std::size_t grasp_id = 0; grasp_id < grasp_candidates.size(); ++grasp_id)
    {
      std::size_t thread_id = omp_get_thread_num();
      ROS_DEBUG_STREAM_NAMED("grasp_filter.superdebug", "Thread " << thread_id << " processing grasp " << grasp_id);

      // if in verbose mode allow for quick exit
      if(ik_thread_structs[thread_id]->verbose_ && !ros::ok())
      {
        continue;
      }

      // assign grasp to process
      ik_thread_structs[thread_id]->grasp_id = grasp_id;

      // procss the grasp
      processCandidateGrasp(ik_thread_structs[thread_id]);
    }

    // count number of grasps remaining
    std::size_t remaining_grasps = 0;
    std::size_t grasp_filtered_by_ik = 0;
    std::size_t grasp_filtered_by_cutting_plane = 0;
    std::size_t grasp_filtered_by_orientation = 0;
    std::size_t pregrasp_filtered_by_ik = 0;

    for(std::size_t i = 0; i < grasp_candidates.size(); ++i)
    {
      if (grasp_candidates[i]->grasp_filtered_by_ik_)
        grasp_filtered_by_ik++;
      else if (grasp_candidates[i]->grasp_filtered_by_cutting_plane_)
        grasp_filtered_by_cutting_plane++;
      else if (grasp_candidates[i]->grasp_filtered_by_orientation_)
        grasp_filtered_by_orientation++;
      else if (grasp_candidates[i]->pregrasp_filtered_by_ik_)
        pregrasp_filtered_by_ik++;
      else
        remaining_grasps++;
    }

    if (remaining_grasps + grasp_filtered_by_ik + grasp_filtered_by_cutting_plane + grasp_filtered_by_orientation +
        pregrasp_filtered_by_ik != grasp_candidates.size())
    {
      ROS_ERROR_STREAM_NAMED("grasp_filter", "Logged filter reasons do not add up to total number of grasps. Internal "
        "error.");
    }

    // end benchmark time
    double duration = (ros::Time::now() - start_time).toSec();

    // keep a running average of calucation time
    static double total_duration = 0;
    static std::size_t total_filter_calls = 0;
    total_duration += duration;
    total_filter_calls += 1;
    double average_duration = total_duration / total_filter_calls;

    if(statistics_verbose_)
    {
      std::cout << "-------------------------------------------------------" << std::endl;
      std::cout << "GRASP FILTER RESULTS " << std::endl;
      std::cout << "total candidate grasps          " << grasp_candidates.size() << std::endl;
      std::cout << "grasp_filtered_by_cutting_plane " << grasp_filtered_by_cutting_plane << std::endl;
      std::cout << "grasp_filtered_by_orientation   " << grasp_filtered_by_orientation << std::endl;
      std::cout << "grasp_filtered_by_ik            " << grasp_filtered_by_ik << std::endl;
      std::cout << "pregrasp_filtered_by_ik         " << pregrasp_filtered_by_ik << std::endl;
      std::cout << "remaining grasps                " << remaining_grasps << std::endl;
      std::cout << "time duration:                  " << duration << std::endl;
      std::cout << "average time duration:          " << average_duration << std::endl;
      std::cout << "-------------------------------------------------------" << std::endl;
    }

    return remaining_grasps;
  }

  bool DavinciGraspFilter::visualizeGrasps(const std::vector<GraspCandidatePtr> &grasp_candidates,
                                           const moveit::core::JointModelGroup *arm_jmg)
  {

  }


  bool DavinciGraspFilter::visualizeCandidateGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates)
  {

  }

}  // namespace
