#include <elastic_map_merge_3d/elastic_map_merge_node.h>

#include <boost/algorithm/string.hpp>
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>

namespace elastic_map_merge_3d
{

ElasticMapMerge3d::ElasticMapMerge3d() : subscriptions_size_(0)
{
  ros::NodeHandle private_nh("~");
  std::string merged_map_topic;
  std::string markers_topic;
  bool publish_tf = true;

  loop_detector.reset(new LoopDetector(private_nh));
  inf_calclator.reset(new InformationMatrixCalculator(private_nh));
  graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));

  anchor_node = nullptr;
  anchor_edge = nullptr;
  floor_plane_node = nullptr;

  private_nh.param("compositing_rate", compositing_rate_, 0.3);
  private_nh.param("discovery_rate", discovery_rate_, 0.05);
  private_nh.param("loop_detection_rate", loop_detection_rate_, 0.01);
  private_nh.param<std::string>("robot_kf_topic", robot_kf_topic_, "keyframes");
  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic, "kf_map");
  private_nh.param<std::string>("markers_topic", markers_topic, "markers");
  private_nh.param<std::string>("world_frame", world_frame_, "map");
  private_nh.param("publish_tf", publish_tf, true);
  private_nh.param("use_floor_information", use_floor_information, false);

  private_nh.param<bool>("fix_first_node", fix_first_node, false);
  private_nh.param<std::string>("fix_first_node_stddev", fix_first_node_stddev, "1 1 1 1 1 1");
  private_nh.param<bool>("fix_first_node_adaptive", fix_first_node_adaptive, true);
  private_nh.param<int>("g2o_solver_num_iterations", g2o_solver_num_iterations, 1024);
  private_nh.param<std::string>("odometry_edge_robust_kernel", odometry_edge_robust_kernel, "NONE");
  private_nh.param("odometry_edge_robust_kernel_size", odometry_edge_robust_kernel_size, 1.0);
  private_nh.param<std::string>("floor_edge_robust_kernel", floor_edge_robust_kernel, "NONE");
  private_nh.param<double>("floor_edge_robust_kernel_size", floor_edge_robust_kernel_size, 1.0);
  private_nh.param<double>("floor_edge_stddev", floor_edge_stddev, 10.0);

  private_nh.param<double>("distance_thresh", 5.0);
  private_nh.param<double>("accum_distance_thresh", 8.0);
  private_nh.param<double>("min_edge_interval", 5.0);

  private_nh.param<double>("fitness_score_max_range", std::numeric_limits<double>::max());
  private_nh.param<double>("fitness_score_thresh", 0.5);
  private_nh.param<std::string>("registration_method", registration_method, "FAST_VGICP");

  private_nh.param<std::string>("robot_1_init_pos", robot_1_init_pos, "");
  private_nh.param<std::string>("robot_2_init_pos", robot_2_init_pos, "");
  private_nh.param<std::string>("robot_3_init_pos", robot_3_init_pos, "");

  // registration parameters
  map_merge_params_ = MapMergingParams::fromROSNode(private_nh);

  // robot init pose database
  initRobotsStartPoses();

  /* publishing */
  merged_map_publisher_ =
      node_.advertise<PointCloud>(merged_map_topic, 50, true);
  markers_publisher = node_.advertise<visualization_msgs::MarkerArray>(markers_topic, 16);

  /* periodical discovery, estimation, compositing */
  compositing_timer_ =
    node_.createTimer(ros::Duration(1. / compositing_rate_),
                      [this](const ros::TimerEvent&) { KFMapCompositing(); });
  discovery_timer_ =
      node_.createTimer(ros::Duration(1. / discovery_rate_),
                        [this](const ros::TimerEvent&) { discovery(); });
  loop_detection_timer_ = 
      node_.createTimer(ros::Duration(1. / loop_detection_rate_),
                        [this](const ros::TimerEvent&) { loopDetection(); });

  // tf needs to publish at constant frequency to avoid transform lookup fails
  if (publish_tf) {
    tf_thread_ = std::thread([this]() {
      ros::Rate rate(30.0);
      while (node_.ok()) {
        publishTF();
        rate.sleep();
      }
    });
  }

}

void ElasticMapMerge3d::initRobotsStartPoses()
{
  if(!robot_1_init_pos.empty()) {
    Eigen::Isometry3d robot_1_init_pos_mat = Eigen::Isometry3d::Identity();

    std::stringstream sst(robot_1_init_pos);
    for(int i = 0; i < 3; i++) {
        double t_val = 1.0;
        sst >> t_val;

        if(t_val > 0 || t_val < 0) {
          robot_1_init_pos_mat(i, 3) = t_val;
        }

    }

    for(int j = 0; j < 3; j++) {
        double rot_val = 1.0;
        sst >> rot_val;

        if(rot_val > 0 || rot_val < 0) {
          robot_1_init_pos_mat(j, j) = rot_val;
        }

    }

    std::string robot_key = robot_namespace_ + "1";
    robot_init_pos_info.insert(std::make_pair(robot_key, robot_1_init_pos_mat));

  }


  if(!robot_2_init_pos.empty()) {
    Eigen::Isometry3d robot_2_init_pos_mat = Eigen::Isometry3d::Identity();

    std::stringstream sst(robot_2_init_pos);
    for(int i = 0; i < 3; i++) {
        double t_val = 1.0;
        sst >> t_val;

        if(t_val > 0 || t_val < 0) {
          robot_2_init_pos_mat(i, 3) = t_val;
        }

    }

    for(int j = 0; j < 3; j++) {
        double rot_val = 1.0;
        sst >> rot_val;

        if(rot_val > 0 || rot_val < 0) {
          robot_2_init_pos_mat(j, j) = rot_val;
        }

    }

    std::string robot_key = robot_namespace_ + "2";
    robot_init_pos_info.insert(std::make_pair(robot_key, robot_2_init_pos_mat));

  }

  if(!robot_3_init_pos.empty()) {
    Eigen::Isometry3d robot_3_init_pos_mat = Eigen::Isometry3d::Identity();

    std::stringstream sst(robot_3_init_pos);
    for(int i = 0; i < 3; i++) {
        double t_val = 1.0;
        sst >> t_val;

        if(t_val > 0 || t_val < 0) {
          robot_3_init_pos_mat(i, 3) = t_val;
        }

    }

    for(int j = 0; j < 3; j++) {
        double rot_val = 1.0;
        sst >> rot_val;

        if(rot_val > 0 || rot_val < 0) {
          robot_3_init_pos_mat(j, j) = rot_val;
        }

    }

    std::string robot_key = robot_namespace_ + "3";
    robot_init_pos_info.insert(std::make_pair(robot_key, robot_3_init_pos_mat));

  }

  for(const auto& elem : robot_init_pos_info) {
    std::cout << elem.first << " " << elem.second.matrix() << std::endl;
  }

}

/*
 * Dynamic robots discovery
 */
void ElasticMapMerge3d::discovery()
{
  ROS_DEBUG("Robot discovery started.");

  ros::master::V_TopicInfo topic_infos;
  std::string robot_name;
  std::string kf_topic;
  std::string map2odom_topic;

  ros::master::getTopics(topic_infos);

  for (const auto& topic : topic_infos) {
    // we check only map topic
    if (!isRobotKFTopic(topic)) {
      continue;
    }

    robot_name = robotNameFromTopic(topic.name);
    if (robots_.count(robot_name)) {
      // we already know this robot
      continue;
    }

    ROS_INFO("adding robot [%s] to system", robot_name.c_str());
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex_);
      subscriptions_.emplace_front();
      ++subscriptions_size_;

      // update anchor robot namespace
      char delimiter = '/';
      std::string s;
      std::vector<std::string> v;

      std::istringstream iss(robot_name);
      while(std::getline(iss, s, delimiter)) {
        v.push_back(s);
      }
      anchor_robot_ns = v[1];
      ROS_INFO("Anchor robot is: %s ", anchor_robot_ns.c_str());

      robots_namespace_order_.insert(robots_namespace_order_.begin(), anchor_robot_ns);

      // add identity transform matrix for new robot as placeholder
      pairwise_transforms_mutex_.lock();

      Eigen::Matrix4f ident_mat = Eigen::Matrix4f::Identity();
      tf::Transform identity_mat = matrix4d2transform(ident_mat.matrix().cast<double>());
      pairwise_transforms_.push_back(identity_mat);

      pairwise_transforms_mutex_.unlock();
    
    }

    // no locking here. robots_ are used only in this procedure
    KFSubscription& subscription = subscriptions_.front();
    robots_.insert({robot_name, &subscription}); 

    /* subscribe callbacks for local keyframe graphs */
    kf_topic = ros::names::append(robot_name, robot_kf_topic_);
    ROS_INFO("Subscribing to KEYFRAME topic: %s.", kf_topic.c_str());
    subscription.kf_sub = node_.subscribe<Keyframes_Graph>(
        kf_topic, 50, [this, &subscription](const Keyframes_Graph::ConstPtr& msg) {
          KFUpdate(msg, subscription);
        });
    subscription.keyframes_added_curr = 0; // init keyframes added counter
    ROS_INFO("Added KEYFRAME topic: %s.", kf_topic.c_str());

  }

  ROS_DEBUG("Robot discovery finished.");
}

/*
 * Composing maps according to computed transforms
 */
void ElasticMapMerge3d::KFMapCompositing()
{
  ROS_DEBUG("Map compositing from keyframes started.");

  std::vector<KeyFrameSnapshot::Ptr> snapshot;

  keyframes_snapshot_mutex.lock();
  snapshot = keyframes_snapshot;
  keyframes_snapshot_mutex.unlock();

  auto merged_map = composeMapFromKFs(snapshot, map_merge_params_.output_resolution);

  if(!merged_map) {
    return;
  }

  // process the merged map to remove noise outliers
  merged_map = removeOutliers(merged_map, map_merge_params_.descriptor_radius,
                              map_merge_params_.outliers_min_neighbours);

  // publish the cleaned merged map to merged map topic
  std_msgs::Header merged_header;
  merged_header.frame_id = world_frame_;
  merged_header.stamp = ros::Time::now();
  pcl_conversions::toPCL(merged_header, merged_map->header);
  merged_map_publisher_.publish(merged_map);

  ROS_DEBUG("Map compositing finished.");
}

bool ElasticMapMerge3d::flushRobotKeyframeQueues()
{
  ROS_INFO("Flushing robot keyframe queues");

  int map2odom_idx = 0;
  for (auto& subscription : subscriptions_) {
    std::lock_guard<std::mutex> lock2(subscription.mutex);

    int num_processed = 0;
    auto keyframe_queue = subscription.keyframe_queue;
    std::deque<KeyFrame::Ptr> temp_keyframe_queue;

    // get respective global transforms
    pairwise_transforms_mutex_.lock();
    auto tf_trans = pairwise_transforms_[map2odom_idx];
    pairwise_transforms_mutex_.unlock();

    // get init pose transform
    std::string robot_id = robots_namespace_order_[map2odom_idx];
    Eigen::Isometry3d robot_init_trans = Eigen::Isometry3d::Identity();
    for (const auto& it : robot_init_pos_info) {
      if (it.first == robot_id) {
        robot_init_trans = it.second;
      }
    }
    
    Eigen::Isometry3d odom2map;
    tf::transformTFToEigen(tf_trans, odom2map);

    for(int i = 0; i < keyframe_queue.size(); i++){
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      temp_keyframe_queue.push_back(keyframe);

      // add pose node
      Eigen::Isometry3d odom = odom2map * keyframe->odom * robot_init_trans;
      keyframe->node = graph_slam->add_se3_node(odom);

      // add floor plane coeffs info from keyframe into pose graph
      if(use_floor_information) {
        if(!floor_plane_node) {
          floor_plane_node = graph_slam->add_plane_node(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0));
          floor_plane_node->setFixed(true);
        }

        Eigen::Vector4d coeffs = *(keyframe->floor_coeffs);
        Eigen::Matrix3d floor_information = Eigen::Matrix3d::Identity() * (1.0 / floor_edge_stddev);
        auto plane_edge = graph_slam->add_se3_plane_edge(keyframe->node, floor_plane_node, coeffs, floor_information);
        graph_slam->add_robust_kernel(plane_edge, floor_edge_robust_kernel, floor_edge_robust_kernel_size);
      }

      // fix the first node
      if(anchor_node != nullptr) {
        if(fix_first_node) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
          std::stringstream sst(fix_first_node_stddev);
          for(int i = 0; i < 6; i++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(i, i) = 1.0 / stddev;
          }

          anchor_node = graph_slam->add_se3_node(Eigen::Isometry3d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se3_edge(anchor_node, keyframe->node, Eigen::Isometry3d::Identity(), inf);
        }
      }

      if(keyframes_database.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      KeyFrame::Ptr prev_keyframe;
      if(i == 0) {
        // find last known keyframe with matching id in database; iterate from back
        for(int j = keyframes_database.size() - 1; j >= 0; --j) {
          if(keyframes_database[j]->robot_ns == keyframe->robot_ns) {
            prev_keyframe = keyframes_database[j];
            break;
          }
        }
      } else {
        prev_keyframe = keyframe_queue[i - 1];
      }

      Eigen::Isometry3d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, relative_pose);
      auto edge = graph_slam->add_se3_edge(keyframe->node, prev_keyframe->node, relative_pose, information);
      graph_slam->add_robust_kernel(edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size);      

    }

    new_keyframes_vec.push_back(temp_keyframe_queue);
    temp_keyframe_queue.clear();
    subscription.keyframe_queue.clear(); // clean individual robot kf queue

    ++map2odom_idx;

  }
  
  return true;

}

void ElasticMapMerge3d::loopDetection()
{
  // add keyframes and floor coeffs in the queues to the pose graph
  bool keyframe_updated = flushRobotKeyframeQueues();
  if(!keyframe_updated) {
    return;
  }

  ROS_INFO("Loop detection started");

  // loop detection
  std::vector<Eigen::Matrix4f> trans_update;
  for (const auto& new_keyframes : new_keyframes_vec) {
    if(new_keyframes.size() != 0) {
      std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes_database, new_keyframes, *graph_slam);
      if(loops.size() > 1) {
          std::cout << "Loop closure detected! " << std::endl;
      }
      for(const auto& loop : loops) {
        Eigen::Isometry3d relpose(loop->relative_pose.cast<double>());
        Eigen::MatrixXd information_matrix = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, relpose);
        auto edge = graph_slam->add_se3_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
        graph_slam->add_robust_kernel(edge, odometry_edge_robust_kernel, odometry_edge_robust_kernel_size);
      }

      // add new keyframes to database
      std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes_database));

      // move the first node anchor position to the current estimate of the first node pose
      // so the first node moves freely while trying to stay around the origin
      if(anchor_node && fix_first_node_adaptive) {
        Eigen::Isometry3d anchor_target = static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
        anchor_node->setEstimate(anchor_target);
      }

      // get tf
      const auto& keyframe = keyframes_database.back();
      Eigen::Isometry3d trans = keyframe->node->estimate() * keyframe->odom.inverse();
      trans_update.push_back(trans.matrix().cast<float>());

    }
  }

  // optimize the pose graph
  std::cout << "Starting pose graph optimization" << std::endl;
  graph_slam->optimize(g2o_solver_num_iterations);

  std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes_database.size());
  std::transform(keyframes_database.begin(), keyframes_database.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

  keyframes_snapshot_mutex.lock();
  keyframes_snapshot.swap(snapshot);
  keyframes_snapshot_mutex.unlock();
  graph_updated = true;

  updatePairwiseTransforms(trans_update);

  new_keyframes_vec.clear();

  if(markers_publisher.getNumSubscribers()) {
    auto markers = create_marker_array(ros::Time::now());
    markers_publisher.publish(markers);
  }

  ROS_INFO("Loop detection completed");

}

void ElasticMapMerge3d::KFUpdate(const Keyframes_Graph::ConstPtr& msg,
                                 KFSubscription& subscription)
{
  ROS_DEBUG("received keyframe update");
  std::lock_guard<std::mutex> lock(subscription.mutex);

  subscription.keyframes_graph = msg;

  int num_new_keyframes = 0;
  for(int i = subscription.keyframes_added_curr; i < msg->keyframes.size(); i++) {
    ros::Time stamp = msg->keyframes[i].header.stamp;
    std::string robot_ns = msg->keyframes[i].robot_ns;
    Eigen::Isometry3d kf_odom = pose2isometry(msg->keyframes[i].pose);
    double accum_d = msg->keyframes[i].accum_distance;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(msg->keyframes[i].cloud, *cloud);

    // convert each KF to keyframe class object
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, robot_ns, kf_odom, accum_d, cloud));

    // Add floor coeffs
    Eigen::Vector4d coeffs(msg->keyframes[i].floor_coeffs[0], msg->keyframes[i].floor_coeffs[1], msg->keyframes[i].floor_coeffs[2], msg->keyframes[i].floor_coeffs[3]);
    keyframe->floor_coeffs = coeffs;

    subscription.keyframe_queue.push_back(keyframe); // new keyframes to be added to global pose graph later

    ++num_new_keyframes;
  }

  subscription.keyframes_added_curr += num_new_keyframes;
}

void ElasticMapMerge3d::updatePairwiseTransforms(std::vector<Eigen::Matrix4f> trans_update)
{
  ROS_INFO("Updating pairwise transforms to global frame");

  if(!pairwise_transforms_.empty() && !trans_update.empty()) {
    pairwise_transforms_mutex_.lock();
    for(int i = 0; i < trans_update.size(); i++) {
      pairwise_transforms_[i] = matrix4d2transform(trans_update[i].matrix().cast<double>());

    }

    std::cout << "Loop detection transforms udpated to:" << std::endl;
    for(const auto& trans : pairwise_transforms_) {
      Eigen::Isometry3d info;
      tf::transformTFToEigen(trans, info);
      std::cout << info.matrix() << "\n" << std::endl;
    }

    pairwise_transforms_mutex_.unlock();
  } else {
    ROS_INFO("No new updates to global transforms");
  }


  ROS_INFO("Finished updating pairwise transforms to global frame");
}

std::string ElasticMapMerge3d::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool ElasticMapMerge3d::isRobotKFTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_kf_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support only KFs type msg as keyframes */
  bool is_occupancy_grid = topic.datatype == "hdl_graph_slam/Keyframes_Graph";

  /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

void ElasticMapMerge3d::publishTF()
{

  // get pairwise transforms
  pairwise_transforms_mutex_.lock();
  std::vector<tf::Transform> global_transforms = pairwise_transforms_;
  pairwise_transforms_mutex_.unlock();

  if (!global_transforms.empty()) {
    for (int i = 0; i < global_transforms.size(); i++) {
      std::string parent_frame_id = world_frame_; // change parent frame
      std::string child_frame_id = robots_namespace_order_[i] + "/map";
      tf_publisher_.sendTransform(tf::StampedTransform(global_transforms[i], ros::Time::now(), parent_frame_id, child_frame_id));

    }
  }

}

visualization_msgs::MarkerArray ElasticMapMerge3d::create_marker_array(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.5;

    visualization_msgs::Marker& imu_marker = markers.markers[1];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 1;
    imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes_database.size());
    traj_marker.colors.resize(keyframes_database.size());
    for(int i = 0; i < keyframes_database.size(); i++) {
      Eigen::Vector3d pos = keyframes_database[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = pos.z();

      double p = static_cast<double>(i) / keyframes_database.size();
      traj_marker.colors[i].r = 1.0 - p;
      traj_marker.colors[i].g = p;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes_database[i]->acceleration) {
        Eigen::Vector3d pos = keyframes_database[i]->node->estimate().translation();
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = pos.z();

        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
      if(edge_se3) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = v2->estimate().translation();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i * 2].r = 1.0 - p1;
        edge_marker.colors[i * 2].g = p1;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
        edge_marker.colors[i * 2 + 1].g = p2;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i * 2].z += 0.5;
          edge_marker.points[i * 2 + 1].z += 0.5;
        }

        continue;
      }

      g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>(edge);
      if(edge_plane) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_plane->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2(pt1.x(), pt1.y(), 0.0);

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z();
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].b = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].b = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>(edge);
      if(edge_priori_xy) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
        pt2.head<2>() = edge_priori_xy->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>(edge);
      if(edge_priori_xyz) {
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_priori_xyz->vertices()[0]);
        Eigen::Vector3d pt1 = v1->estimate().translation();
        Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = pt2.z();

        edge_marker.colors[i * 2].r = 1.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 1.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    if(!keyframes_database.empty()) {
      Eigen::Vector3d pos = keyframes_database.back()->node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = pos.z();
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;

    return markers;
  }

}  // namespace elastic_map_merge_3d

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elastic_map_merge");
  // this package is still in development -- start with debugging enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  elastic_map_merge_3d::ElasticMapMerge3d map_merge_node;
  // use all threads for spinning
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}