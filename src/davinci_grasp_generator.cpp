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
   Desc:   Generates geometric grasps for cuboids and blocks, not using physics or contact wrenches
*/


#include <cwru_davinci_moveit_grasps/davinci_grasp_generator.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace davinci_moveit_grasps {
DavinciGraspGenerator::DavinciGraspGenerator(moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                                             bool verbose = false) : visual_tools_(visual_tools), verbose_(verbose),
                                                                     nh_("~/davinci_moveit_grasps/generator")
{
  // load visulization settings
  const std::string parent_name = "grasps";  // for namespacing logging messages

  rosparam_shortcuts::get(parent_name, nh_, "show_grasp_arrows", show_grasp_arrows_);
  rosparam_shortcuts::get(parent_name, nh_, "show_grasp_arrows_speed", show_grasp_arrows_speed_);

  rosparam_shortcuts::get(parent_name, nh_, "show_prefiltered_grasps", show_prefiltered_grasps_);
  rosparam_shortcuts::get(parent_name, nh_, "show_grasp_arrows_speed", show_grasp_arrows_speed_);

  // load scoring weights
  rosparam_shortcuts::get(parent_name, nh_, "depth_score_weight", depth_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "width_score_weight", width_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "orientation_x_score_weight", orientation_x_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "orientation_y_score_weight", orientation_y_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "orientation_z_score_weight", orientation_z_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "translation_x_score_weight", translation_x_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "translation_y_score_weight", translation_y_score_weight_);
  rosparam_shortcuts::get(parent_name, nh_, "translation_z_score_weight", translation_z_score_weight_);

  ROS_INFO_STREAM_NAMED("grasps", "GraspGenerator Ready.");

  // set ideal grasp pose (currently only uses orientation of pose)
  // TODO this ideal grasp pose needs to be changed later
  ideal_grasp_pose_ = Eigen::Affine3d::Identity();
  ideal_grasp_pose_ = ideal_grasp_pose_*Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ())
      *Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX());

  ideal_grasp_pose_.translation() = Eigen::Vector3d(0, 0, 2.0);
}

bool DavinciGraspGenerator::getIntersectionHelper(Eigen::Affined3d cuboid_pose,
                                                  double depth,
                                                  double width,
                                                  double height,
                                                  Eigen::Affined3d grasp_pose,
                                                  const GraspDataPtr grasp_data)
{
  // TODO
}
}
