#ifndef ELASTIC_MAP_MERGE_3D_FEATURES_H_
#define ELASTIC_MAP_MERGE_3D_FEATURES_H_

#include <elastic_map_merge_3d/typedefs.h>

namespace elastic_map_merge_3d
{
/**
 * @defgroup features Features
 * @brief Low-level feature-extracting functions.
 * @details Low-level functions to extract features from pointclouds and to
 * preprocess pointclouds for feature extraction. These function operate on
 * individual pointcloud.
 * @{
 */

/**
 * @brief Voxelize input pointcloud to reduce number of points.
 *
 * @param input input pointcloud
 * @param resolution required resolution for voxelization
 *
 * @return Voxelized pointcloud
 */
PointCloudPtr downSample(const PointCloudConstPtr &input, double resolution);

}  // namespace elastic_map_merge_3d

#endif  // ELASTIC_MAP_MERGE_3D_FEATURES_H_