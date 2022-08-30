#include <elastic_map_merge_3d/elastic_map_merging.h>
#include <elastic_map_merge_3d/keyframe_map_assembler.h>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace elastic_map_merge_3d
{

MapMergingParams MapMergingParams::fromCommandLine(int argc, char **argv)
{
  MapMergingParams params;

  using pcl::console::parse_argument;

  parse_argument(argc, argv, "--resolution", params.resolution);
  parse_argument(argc, argv, "--descriptor_radius", params.descriptor_radius);
  parse_argument(argc, argv, "--outliers_min_neighbours",
                 params.outliers_min_neighbours);
  parse_argument(argc, argv, "--normal_radius", params.normal_radius);
  parse_argument(argc, argv, "--inlier_threshold", params.inlier_threshold);
  parse_argument(argc, argv, "--max_correspondence_distance",
                 params.max_correspondence_distance);
  parse_argument(argc, argv, "--max_iterations", params.max_iterations);
  int matching_k = -1;
  parse_argument(argc, argv, "--matching_k", matching_k);
  if (matching_k > 0) {
    params.matching_k = size_t(matching_k);
  }
  parse_argument(argc, argv, "--transform_epsilon", params.transform_epsilon);
  parse_argument(argc, argv, "--confidence_threshold",
                 params.confidence_threshold);
  parse_argument(argc, argv, "--output_resolution", params.output_resolution);
  parse_argument(argc, argv, "--reg_resolution", params.reg_resolution);
  parse_argument(argc, argv, "--reg_step_size", params.reg_step_size);
  parse_argument(argc, argv, "--filter_z_min", params.filter_z_min);
  parse_argument(argc, argv, "--filter_z_max", params.filter_z_max);
  parse_argument(argc, argv, "--reference_frame", params.reference_frame);

  return params;
}

MapMergingParams MapMergingParams::fromROSNode(const ros::NodeHandle &n)
{
  MapMergingParams params;

  n.getParam("resolution", params.resolution);
  n.getParam("descriptor_radius", params.descriptor_radius);
  n.getParam("outliers_min_neighbours",
                 params.outliers_min_neighbours);
  n.getParam("normal_radius", params.normal_radius);

  n.getParam("inlier_threshold", params.inlier_threshold);
  n.getParam("max_correspondence_distance",
                 params.max_correspondence_distance);
  n.getParam("max_iterations", params.max_iterations);
  int matching_k = -1;
  n.getParam("matching_k", matching_k);
  if (matching_k > 0) {
    params.matching_k = size_t(matching_k);
  }
  n.getParam("transform_epsilon", params.transform_epsilon);
  n.getParam("confidence_threshold",
                 params.confidence_threshold);
  n.getParam("output_resolution", params.output_resolution);
  n.getParam("reg_resolution", params.reg_resolution);
  n.getParam("reg_step_size", params.reg_step_size);
  n.getParam("filter_z_min", params.filter_z_min);
  n.getParam("filter_z_max", params.filter_z_max);
  n.getParam("reference_frame", params.reference_frame);

  return params;
}

std::ostream &operator<<(std::ostream &stream, const MapMergingParams &params)
{
  stream << "resolution: " << params.resolution << std::endl;
  stream << "descriptor_radius: " << params.descriptor_radius << std::endl;
  stream << "outliers_min_neighbours: " << params.outliers_min_neighbours
         << std::endl;
  stream << "normal_radius: " << params.normal_radius << std::endl;
  stream << "inlier_threshold: " << params.inlier_threshold << std::endl;
  stream << "max_correspondence_distance: "
         << params.max_correspondence_distance << std::endl;
  stream << "max_iterations: " << params.max_iterations << std::endl;
  stream << "matching_k: " << params.matching_k << std::endl;
  stream << "transform_epsilon: " << params.transform_epsilon << std::endl;
  stream << "confidence_threshold: " << params.confidence_threshold << std::endl;
  stream << "output_resolution: " << params.output_resolution << std::endl;
  stream << "reg_resolution: " << params.reg_resolution << std::endl;
  stream << "reg_step_size: " << params.reg_step_size << std::endl;
  stream << "filter_z_min: " << params.filter_z_min << std::endl;
  stream << "filter_z_max: " << params.filter_z_max << std::endl;
  stream << "reference_frame: " << params.reference_frame << std::endl;

  return stream;
}

PointCloudPtr composeMapFromKFs(const std::vector<KeyFrameSnapshot::Ptr> &keyframe_snapshot,
                                 double resolution)
{
  if (keyframe_snapshot.empty()) {
    return nullptr;
  }

  PointCloudPtr result(new PointCloud);
  PointCloudPtr cloud_aligned(new PointCloud);
  cloud_aligned = assemble_map_from_kfs(keyframe_snapshot, resolution);

  // // voxelize result cloud to required resolution
  // result = downSample(result, resolution);

  return cloud_aligned;
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local
 * neighbors */
PointCloudPtr removeOutliers(const PointCloudConstPtr &input, double radius,
                             int min_neighbors)
{
  pcl::RadiusOutlierRemoval<PointT> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(radius);
  filter.setMinNeighborsInRadius(min_neighbors);

  PointCloudPtr output(new PointCloud);
  filter.filter(*output);

  return output;
}

}  // namespace elastic_map_merge_3d