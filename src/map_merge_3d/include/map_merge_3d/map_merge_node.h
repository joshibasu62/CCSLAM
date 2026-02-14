#ifndef MAP_MERGE_MAP_MERGE_NODE_H_
#define MAP_MERGE_MAP_MERGE_NODE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map_merge_3d/map_merging.h>
#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{

class MapMerge3d : public rclcpp::Node
{
private:
  struct MapSubscription {
    std::mutex mutex;
    PointCloudConstPtr map;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub;
  };

  /* node parameters */
  double compositing_rate_;
  double discovery_rate_;
  double estimation_rate_;
  std::string robot_map_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  bool publish_tf_; // Added member variable for this

  // compositing & estimation parameters
  MapMergingParams map_merge_params_;

  // publishing
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_map_publisher_;
  
  // periodical callbacks
  rclcpp::TimerBase::SharedPtr compositing_timer_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr estimation_timer_;
  
  // transforms for tf
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
  std::thread tf_thread_;             
  std::atomic_flag tf_current_flag_; 

  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, MapSubscription*> robots_;
  // owns maps -- iterator safe
  std::forward_list<MapSubscription> subscriptions_;
  size_t subscriptions_size_;
  std::mutex subscriptions_mutex_;
  
  // estimated transforms between maps
  std::vector<Eigen::Matrix4f> transforms_;
  std::mutex transforms_mutex_;

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotMapTopic(const std::string& topic_name, const std::string& topic_type);
  void mapUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                 MapSubscription& subscription);
  void publishTF();

public:
  MapMerge3d();
  ~MapMerge3d(); // Destructor to join threads

  void discovery();
  void mapCompositing();
  void transformsEstimation();
  std::vector<PointCloudConstPtr> getMaps();
  std::vector<Eigen::Matrix4f> getTransforms();
};

}  // namespace map_merge_3d

#endif  // MAP_MERGE_MAP_MERGE_NODE_H_