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

  parse_argument(argc, argv, "--descriptor_radius", params.descriptor_radius);
  parse_argument(argc, argv, "--outliers_min_neighbours",
                 params.outliers_min_neighbours);
  parse_argument(argc, argv, "--output_resolution", params.output_resolution);

  return params;
}

MapMergingParams MapMergingParams::fromROSNode(const ros::NodeHandle &n)
{
  MapMergingParams params;

  n.getParam("descriptor_radius", params.descriptor_radius);
  n.getParam("outliers_min_neighbours",
                 params.outliers_min_neighbours);
  n.getParam("output_resolution", params.output_resolution);

  return params;
}

std::ostream &operator<<(std::ostream &stream, const MapMergingParams &params)
{
  stream << "descriptor_radius: " << params.descriptor_radius << std::endl;
  stream << "outliers_min_neighbours: " << params.outliers_min_neighbours
         << std::endl;
  stream << "output_resolution: " << params.output_resolution << std::endl;

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