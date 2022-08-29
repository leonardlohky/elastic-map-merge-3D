// SPDX-License-Identifier: BSD-2-Clause

#ifndef ELASTIC_MAP_MERGE_3D_REGISTRATIONS_H
#define ELASTIC_MAP_MERGE_3D_REGISTRATIONS_H

#include <ros/ros.h>
#include <pcl/registration/registration.h>

#include <elastic_map_merge_3d/typedefs.h>

namespace elastic_map_merge_3d {

/**
 * @brief select a scan matching algorithm according to rosparams
 * @param pnh
 * @return selected scan matching
 */
pcl::Registration<PointT, PointT>::Ptr select_registration_method(ros::NodeHandle& pnh);

}  // namespace elastic_map_merge_3d

#endif  //
