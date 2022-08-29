// SPDX-License-Identifier: BSD-2-Clause

#include <elastic_map_merge_3d/keyframe_map_assembler.h>

#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>

namespace elastic_map_merge_3d {

Eigen::Matrix4f kf2matrix4f(const KF keyframe) {
    const auto& orientation = keyframe.pose.orientation;
    const auto& position = keyframe.pose.position;

    Eigen::Quaterniond q;
    q.w() = keyframe.pose.orientation.w;
    q.x() = keyframe.pose.orientation.x;
    q.y() = keyframe.pose.orientation.y;
    q.z() = keyframe.pose.orientation.z;

    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.linear() = q.toRotationMatrix();
    isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);

    return isometry.matrix().cast<float>();
}

PointCloudPtr assemble_map_from_kfs(const std::vector<KeyFrameSnapshot::Ptr> keyframes, double resolution) {
  if(keyframes.empty()) {
    std::cerr << "warning: keyframes empty!!" << std::endl;
    return nullptr;
  }

  PointCloudPtr cloud(new PointCloud());

  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();

    for(const auto& src_pt : keyframe->cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  if (resolution <=0.0)
    return cloud; // To get unfiltered point cloud with intensity

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  PointCloudPtr filtered(new PointCloud());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

} // namespace elastic_map_merge_3d
