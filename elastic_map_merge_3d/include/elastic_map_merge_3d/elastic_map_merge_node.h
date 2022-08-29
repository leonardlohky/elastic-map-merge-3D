#ifndef ELASTIC_MAP_MERGE_3D_ELASTIC_MAP_MERGE_NODE_H_
#define ELASTIC_MAP_MERGE_3D_ELASTIC_MAP_MERGE_NODE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <elastic_map_merge_3d/typedefs.h>
#include <elastic_map_merge_3d/elastic_map_merging.h>
#include <elastic_map_merge_3d/ros_utils.h>
#include <elastic_map_merge_3d/graph_slam.h>
#include <elastic_map_merge_3d/keyframe.h>
#include <elastic_map_merge_3d/loop_detector.h>
#include <elastic_map_merge_3d/information_matrix_calculator.h>


namespace elastic_map_merge_3d
{
/**
 * @defgroup node ROS node
 * @brief ROS interface.
 * @details Manages maps discovery, transforms estimation and the global
 * map publishing.
 * @{
 */

/**
 * @brief ROS node class.
 * @details Runs robot discovery, transforms estimation and map compositing at
 * predefined rates (from ROS parameters). Does not spin on its own.
 *
 */


struct KeyFramesGraphs {
  std::vector<std::vector<KeyFrame::Ptr>> keyframes;
  std::vector<std::unique_ptr<GraphSLAM>> graphs;
};

class ElasticMapMerge3d
{
private:
  struct KFSubscription {
    // protects keyframes
    std::mutex mutex;
    Keyframes_Graph::ConstPtr keyframes_graph;
    ros::Subscriber kf_sub;
    int keyframes_added_curr;
    std::vector<KeyFrame::Ptr> keyframe_queue;
  };

  struct LocalMap2OdomSubscription {
    // protects map2odom
    std::mutex mutex;
    geometry_msgs::TransformStamped::ConstPtr map2odom_msg;
    ros::Subscriber local_map2odom_sub;
  };

  ros::NodeHandle node_;

  /* node parameters */
  double compositing_rate_;
  double discovery_rate_;
  double loop_detection_rate_;
  std::string robot_kf_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  // compositing & estimation parameters
  MapMergingParams map_merge_params_;

  // robust kernel parameters
  std::string odometry_edge_robust_kernel;
  double odometry_edge_robust_kernel_size;
  std::string floor_edge_robust_kernel;
  double floor_edge_robust_kernel_size;

  // to hold new keyframes from all robots
  std::vector<std::deque<KeyFrame::Ptr>> new_keyframes_vec;

  bool fix_first_node;
  std::string fix_first_node_stddev;
  bool fix_first_node_adaptive;
  int g2o_solver_num_iterations;
  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_plane_node;
  double floor_edge_stddev;
  std::vector<KeyFrame::Ptr> keyframes_database; // global server keyframe database

  // loop detector params
  double distance_thresh;
  double accum_distance_thresh;
  double distance_from_last_edge_thresh;

  double fitness_score_max_range;
  double fitness_score_thresh;
  std::string registration_method;

  // for map cloud generation
  std::atomic_bool graph_updated;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::vector<std::vector<KeyFrameSnapshot::Ptr>> keyframes_snapshot_vec;

  // publishing
  ros::Publisher merged_map_publisher_;
  ros::Publisher markers_publisher;
  // periodical callbacks
  ros::Timer compositing_timer_;
  ros::Timer discovery_timer_;
  ros::Timer loop_detection_timer_;
  // transforms for tf
  std::vector<geometry_msgs::TransformStamped> tf_transforms_;
  tf::TransformBroadcaster tf_publisher_;
  std::thread tf_thread_;             //  tf needs it own thread
  std::atomic_flag tf_current_flag_;  // whether tf_transforms_ are up to date
                                      // with transforms_

  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, KFSubscription*> robots_;
  std::vector<std::string> robots_namespace_order_;
  std::string anchor_robot_ns;

  // owns keyframe graphs -- iterator safe
  std::forward_list<KFSubscription> subscriptions_;
  size_t subscriptions_size_;
  std::mutex subscriptions_mutex_;

  // owns local map2odoms -- iterator safe
  std::forward_list<LocalMap2OdomSubscription> map2odom_subscriptions_;
  size_t map2odom_subscriptions_size_;
  std::mutex map2odom_subscriptions_mutex_;

  std::vector<Eigen::Matrix4f> global_transforms_;
  std::mutex global_transforms_mutex_;

  std::vector<tf::Transform> pairwise_transforms_;
  std::mutex pairwise_transforms_mutex_;

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotKFTopic(const ros::master::TopicInfo& topic);
  void KFUpdate(const Keyframes_Graph::ConstPtr& msg,
                 KFSubscription& subscription);
  void LocalMap2OdomUpdate(const geometry_msgs::TransformStamped::ConstPtr& msg,
                            LocalMap2OdomSubscription& subscription);
  std::vector<tf::Transform> getLocalMap2Odoms();
  void updatePairwiseTransforms(std::vector<Eigen::Matrix4f> trans_update);
  void publishTF();

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;

public:

  ElasticMapMerge3d();
  
  /**
   * @brief Initiates discovery of new robots (maps) under current ROS core.
   * @details When new maps topics are found, there are added for merging. This
   * function is thread-safe
   */
  void discovery();

  /**
   * @brief Composes and publishes the global map based on keyframes
   * transformations
   * @details This function is thread-safe
   */
  void KFMapCompositing();

  /**
   * @brief Composes and publishes the global map based on keyframes
   * transformations
   * @details This function is thread-safe
   */
  bool flushRobotKeyframeQueues();
  
  /**
   * @brief Composes and publishes the global map based on keyframes
   * transformations
   * @details This function is thread-safe
   */
  void loopDetection();

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const;

};

///@} group node

}  // namespace elastic_map_merge_3d

#endif  // ELASTIC_MAP_MERGE_3D_ELASTIC_MAP_MERGE_NODE_H_