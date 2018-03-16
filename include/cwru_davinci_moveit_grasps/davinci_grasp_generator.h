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

#ifndef CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_GENERATOR_H
#define CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_GENERATOR_H

// ROS
#include <ros/ros.h>

// TF
#include <tf_conversions/tf_eigen.h>

// Msgs
#include <geometry_msgs/PoseArray.h>

// MoveIt
#include <moveit_msgs/Grasp.h>
#include <moveit/macros/deprecation.h>

// geometric_shapes
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <eigen_conversions/eigen_msg.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// moveit_grasps
#include <moveit_grasps/grasp_candidate.h>
#include <moveit_grasps/grasp_scorer.h>

// bounding_box
//#include <bounding_box/bounding_box.h>

// C++
#include <cstdlib>
#include <string>
#include <math.h>
#include <limits>
#define _USE_MATH_DEFINES

#include <moveit_grasps/grasp_data.h>

namespace davinci_moveit_grasps
{
  // Grasp axis orientation
  enum grasp_axis_t
  {
    X_AXIS,
    Y_AXIS,
    Z_AXIS
  };


  class DavinciGraspGenerator
  {
  public:
    /**
     * @brief construct
     * @param visual_tools
     * @param verbose
     */
    DavinciGraspGenerator(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose = false);

    /**
     * @brief Create possible grasp positions around a cuboid
     * @param cuboid_pose - centroid of object to grasp in world frame
     * @param depth length of cuboid along local x-axis
     * @param width length of cuboid along local y-axis
     * @param height length of cuboid along local z-axis
     * @param grasp_data data describing end effector
     * @param grasp_candidates possible grasps generated
     * @return true if successful
     */
    bool generateGrasps(const Eigen3d::Affine3d &needle_pose, double depth, double width, double height,
                        const GraspDatePtr grasp_data, std::vector <GraspCandidatePtr> &grasp_candidates);

    /**
     * @brief helper function for determining if the grasp will intersect the cuboid
     * @param cuboid_pose - centroid of object to grasp in world frame
     * @param depth - size of cuboid along x axis
     * @param width - size of cuboid along y axis
     * @param height - size of cuboid along z axis
     * @param grasp_pose - pose of grasp
     * @param grasp_data - data describing end effector
     * @return true if the grasp intersects the cuboid
     */
    bool graspIntersectionHelper(Eigen::Affine3d cuboid_pose, double depth, double width, double height,
                                 Eigen::Affine3d grasp_pose, const GraspDataPtr grasp_data);

    /**
     * @brief helper function to test intersection of a line with a plane
     * @param cuboid_pose centroid of object to grasp in world frame
     * @param depth size of cuboid along x axis
     * @param width size of cuboid along y axis
     * @param height size of cuboid along z axis
     * @param grasp_pose pose of grasp
     * @param grasp_data data describing end effector
     * @return true if the grasp intersects the cuboid
     */
    bool getIntersectionHelper(Eigen::Affined3d cuboid_pose, double depth, double width, double height,
                               Eigen::Affined3d grasp_pose, const GraspDataPtr grasp_data);

    /**
     * @brief creates grasp messages from the generated grasp pose
     * @param pose the grasp pose
     * @param grasp_data data describing the end effector
     * @param grasp_candidates list possible grasps
     * @param object_pose pose of object to grasp
     * @param object_width
     * @return true on success
     */
    bool addGrasp(const Eigen::Affine3d &pose, const GraspDataPrt grasp_data,
                  std::vector <GraspCandidatePtr> &grasp_candidates, const Eigen::Affine3d &object_pose,
                  double object_width);

    /**
      * @brief Getter for Verbose
      */
    bool getVerbose()
    {
      return verbose_;
    }

    /**
     * @brief Getter for ideal grasp pose
     */
    Eigen::Affine3d getIdealGraspPose()
    {
      return ideal_grasp_pose_;
    }

    /**
      * @brief Setter for Verbose
      */
    void setVerbose(bool verbose)
    {
      verbose_ = verbose;
    }

    /**
      * @brief Visualize animated grasps
      * @return true on success
      */
    bool visualizeAnimatedGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates,
                                 const moveit::core::JointModelGroup* ee_jmg, double animation_speed);

    // Ideal grasp pose for scoring purposes
    Eigen::Affine3d ideal_grasp_pose_;

  private:
    // class for publishing stuff to rviz
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    // display more output both in console
    bool verbose_;

    // shared node handle
    ros::NodeHandle nh_;

    // transform from frame of box to global frame
    Eigen::Affine3d object_global_transform_;

    // visualization levels
    bool show_grasp_arrows_;
    double show_grasp_arrows_speed_;

    bool show_prefiltered_grasps_;
    double show_prefiltered_grasps_speed_l;

    double min_grasp_distance_;
    double max_grasp_distance_;

    Eigen::Vector3d min_translations_;
    Eigen::Vector3d max_translations_;

    double depth_score_weight_;
    double width_score_weight_;
    double height_score_weight_;
    double orientation_x_score_weight_;
    double orientation_y_score_weight_;
    double orientation_z_score_weight_;
    double translation_x_score_weight_;
    double translation_y_score_weight_;
    double translation_z_score_weight_;

  };  //end of class

  typedef boost::shared_ptr<GraspGenerator> GraspGeneratorPtr;
  typedef boost::shared_ptr<const GraspGenerator> GraspGeneratorConstPtr;

}


#endif //CWRU_DAVINCI_MOVEIT_GRASPS_DAVINCI_GRASP_GENERATOR_H
