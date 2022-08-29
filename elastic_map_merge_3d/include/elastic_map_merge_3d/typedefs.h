#ifndef ELASTIC_MAP_MERGE_3D_TYPEDEFS_H_
#define ELASTIC_MAP_MERGE_3D_TYPEDEFS_H_

#include <pcl/PCLPointCloud2.h>
#include <pcl/correspondence.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/search/kdtree.h>

#include <elastic_map_merge_3d/KF.h>
#include <elastic_map_merge_3d/Keyframes_Graph.h>

namespace elastic_map_merge_3d
{
// setup some typedefs for working with RGB pointclouds

// basic pointclouds definitions
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::search::KdTree<PointT> TreeT;

// normals as separate pointscloud
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// local descriptors for registration
typedef pcl::PCLPointCloud2 LocalDescriptors;
typedef pcl::PCLPointCloud2::Ptr LocalDescriptorsPtr;
typedef pcl::PCLPointCloud2::ConstPtr LocalDescriptorsConstPtr;

// for ground plane segmentation
typedef pcl::ModelCoefficients::Ptr ModelCoeffPtr;
typedef pcl::PointIndices::Ptr PointIndicesPtr;

// correspondences
using pcl::Correspondences;
using pcl::CorrespondencesPtr;

// color handler for visualisation
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;

}  // elastic_map_merge_3d

#endif  // ELASTIC_MAP_MERGE_3D_TYPEDEFS_H_
