#ifndef ELASTIC_MAP_MERGE_3D_ELASTIC_MAP_MERGING_H_
#define ELASTIC_MAP_MERGE_3D_ELASTIC_MAP_MERGING_H_

#include <ostream>

#include <ros/ros.h>

#include <elastic_map_merge_3d/typedefs.h>
#include <elastic_map_merge_3d/features.h>
#include <elastic_map_merge_3d/keyframe.h>
#include <elastic_map_merge_3d/loop_detector.h>

namespace elastic_map_merge_3d
{
/**
 * @defgroup map_merging Map merging
 * @brief High-level map-merging interface.
 * @details High-level interface to estimate transformations between n
 * pointclouds and compositing global map..
 * @{
 */

/**
 * @brief Parameters for map merging high-level interface
 * @details Contains all tunables for estimating transformation between n maps
 * and for compositing the global map
 *
 */
struct MapMergingParams {
  double resolution = 0.1;
  double descriptor_radius = resolution * 8.0;
  int outliers_min_neighbours = 100;
  double normal_radius = resolution * 6.0;
  double keypoint_threshold = 0.01;
  bool refine_transform = true;
  double inlier_threshold = resolution * 5.0;
  double max_correspondence_distance = inlier_threshold * 2.0;
  int max_iterations = 100;
  size_t matching_k = 5;
  double transform_epsilon = 1e-2;
  double confidence_threshold = 10.0;
  double output_resolution = 0.05;
  double reg_resolution = 1.5;
  double reg_step_size = 0.1;
  double filter_z_min = -std::numeric_limits<double>::infinity();
  double filter_z_max = std::numeric_limits<double>::infinity();
  std::string reference_frame = "AUTO";

  /**
   * @brief Sources parameters from command line arguments
   * @details Uses PCL's command line parser to initialize the parameters.
   * Format is `--param_name <value>`. param_name is the same as the struct
   * member.
   *
   * @param argc arguments count
   * @param argv program arguments
   *
   * @return parameters with values from command line of default values where
   * not provided.
   */
  static MapMergingParams fromCommandLine(int argc, char **argv);

  /**
   * @brief Sources parameters from ROS node parameters
   * @details Parameter names are the same as the struct
   * member.
   *
   * @param node ROS node to source parameters from
   * @return parameters with values from ROS params of default values where
   * not provided.
   */
  static MapMergingParams fromROSNode(const ros::NodeHandle &node);

};
std::ostream &operator<<(std::ostream &stream, const MapMergingParams &params);

PointCloudPtr composeMapFromKFs(const std::vector<KeyFrameSnapshot::Ptr> &keyframes_snapshot,
                                 double resolution);

PointCloudPtr removeOutliers(const PointCloudConstPtr &input, double radius,
                             int min_neighbors);

}  // namespace elastic_map_merge_3d

#endif  // ELASTIC_MAP_MERGE_3D_ELASTIC_MAP_MERGING_H_