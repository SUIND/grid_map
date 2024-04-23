/*
 * PointCloudVisualization.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <grid_map_core/GridMap.hpp>
#include <grid_map_visualization/visualizations/VisualizationBase.hpp>

// ROS
#include <ros/ros.h>

namespace grid_map_visualization
{
/*!
 * Visualization of the grid map as a point cloud.
 */
class PointCloudVisualization : public VisualizationBase
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param name the name of the visualization.
   */
  PointCloudVisualization(ros::NodeHandle& nodeHandle, const std::string& name);

  /*!
   * Destructor.
   */
  virtual ~PointCloudVisualization();

  /*!
   * Read parameters from ROS.
   * @param config the parameters as XML.
   * @return true if successful.
   */
  bool readParameters(XmlRpc::XmlRpcValue& config);

  /*!
   * Initialization.
   */
  bool initialize();

  /*!
   * Generates the visualization.
   * @param map the grid map to visualize.
   * @return true if successful.
   */
  bool visualize(const grid_map::GridMap& map);

 private:
  //! Type that is transformed to points.
  std::string layer_;
};

}  // namespace grid_map_visualization
