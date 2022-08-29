// SPDX-License-Identifier: BSD-2-Clause

#ifndef ELASTIC_MAP_MERGE_3D_KEYFRAME_MAP_ASSEMBLER_HPP
#define ELASTIC_MAP_MERGE_3D_KEYFRAME_MAP_ASSEMBLER_HPP

#include <ros/ros.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <elastic_map_merge_3d/typedefs.h>
#include <elastic_map_merge_3d/keyframe.h>

namespace elastic_map_merge_3d {
  
/**
 * @brief converts KF msg pose info to Eigen::Matrix4f form
 * @param keyframe  keyframe in KF msg form
 * @return converted pose in Eigen::Matrix4f form
 */
Eigen::Matrix4f kf2matrix4f(const KF keyframe);

/**
 * @brief generates a map point cloud
 * @param keyframes   snapshots of keyframes
 */
PointCloudPtr assemble_map_from_kfs(const std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot, double resolution);

}  // namespace elastic_map_merge_3d

#endif  // ELASTIC_MAP_MERGE_3D_KEYFRAME_MAP_ASSEMBLER_HPP
