#include <map_merge_3d/map_merge_node.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rcpputils/asserts.hpp>
// FIX 1: Change .h to .hpp
#include <tf2_eigen/tf2_eigen.hpp> 

namespace map_merge_3d
{

static bool endsWith(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() && 
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

MapMerge3d::MapMerge3d() 
: Node("map_merge_3d"), 
  subscriptions_size_(0)
{
  this->declare_parameter("compositing_rate", 0.3);
  this->declare_parameter("discovery_rate", 0.05);
  this->declare_parameter("estimation_rate", 0.01);
  this->declare_parameter("robot_map_topic", "map");
  this->declare_parameter("robot_namespace", "");
  this->declare_parameter("merged_map_topic", "map");
  this->declare_parameter("world_frame", "world");
  this->declare_parameter("publish_tf", true);

  compositing_rate_ = this->get_parameter("compositing_rate").as_double();
  discovery_rate_ = this->get_parameter("discovery_rate").as_double();
  estimation_rate_ = this->get_parameter("estimation_rate").as_double();
  robot_map_topic_ = this->get_parameter("robot_map_topic").as_string();
  robot_namespace_ = this->get_parameter("robot_namespace").as_string();
  std::string merged_map_topic = this->get_parameter("merged_map_topic").as_string();
  world_frame_ = this->get_parameter("world_frame").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();

  // FIX 2: passing 'this' (raw pointer) instead of shared_from_this()
  // You must update map_merging.h/.cpp as shown below for this to work
  map_merge_params_ = MapMergingParams::fromROSNode(this);

  merged_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_map_topic, 1);

  compositing_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / compositing_rate_),
      [this]() { mapCompositing(); });

  discovery_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / discovery_rate_),
      [this]() { discovery(); });

  estimation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / estimation_rate_),
      [this]() { transformsEstimation(); });

  tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  if (publish_tf_) {
    tf_thread_ = std::thread([this]() {
      rclcpp::Rate rate(30.0);
      while (rclcpp::ok()) {
        publishTF();
        rate.sleep();
      }
    });
  }
}

MapMerge3d::~MapMerge3d() {
    if (tf_thread_.joinable()) {
        tf_thread_.join();
    }
}

void MapMerge3d::discovery()
{
  RCLCPP_DEBUG(this->get_logger(), "Robot discovery started.");

  auto topic_names_and_types = this->get_topic_names_and_types();

  for (const auto& t : topic_names_and_types) {
    std::string name = t.first;
    std::vector<std::string> types = t.second;
    
    if (types.empty()) continue;
    std::string type = types[0]; 

    if (!isRobotMapTopic(name, type)) {
      continue;
    }

    std::string robot_name = robotNameFromTopic(name);
    if (robots_.count(robot_name)) {
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Adding robot [%s] to system", robot_name.c_str());
    
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex_);
      subscriptions_.emplace_front();
      ++subscriptions_size_;
    }

    MapSubscription& subscription = subscriptions_.front();
    robots_.insert({robot_name, &subscription});

    RCLCPP_INFO(this->get_logger(), "Subscribing to MAP topic: %s", name.c_str());
    
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.transient_local(); 

    subscription.map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        name, qos, 
        [this, &subscription](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          mapUpdate(msg, subscription);
        });
  }
  RCLCPP_DEBUG(this->get_logger(), "Robot discovery finished.");
}

void MapMerge3d::mapCompositing()
{
  std::vector<PointCloudConstPtr> clouds = getMaps();
  if (clouds.empty()) return;

  std::vector<Eigen::Matrix4f> transforms = getTransforms();
  
  if (clouds.size() > transforms.size()) {
      clouds.resize(transforms.size());
  }

  PointCloudPtr merged_map = composeMaps(clouds, transforms, map_merge_params_.output_resolution);
  if (!merged_map) return;

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*merged_map, output_msg);
  
  output_msg.header.frame_id = world_frame_;
  output_msg.header.stamp = this->now();

  merged_map_publisher_->publish(output_msg);
}

void MapMerge3d::transformsEstimation()
{
  std::vector<PointCloudConstPtr> clouds = getMaps();
  if (clouds.empty()) return;

  std::vector<Eigen::Matrix4f> transforms = estimateMapsTransforms(clouds, map_merge_params_);

  {
    std::lock_guard<std::mutex> lock(transforms_mutex_);
    transforms_ = transforms;
  }
  tf_current_flag_.clear();
}

void MapMerge3d::mapUpdate(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                           MapSubscription& subscription)
{
  std::lock_guard<std::mutex> lock(subscription.mutex);
  PointCloudPtr cloud(new PointCloud);
  pcl::fromROSMsg(*msg, *cloud);
  subscription.map = cloud;
}

std::vector<PointCloudConstPtr> MapMerge3d::getMaps()
{
  std::vector<PointCloudConstPtr> clouds;
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  clouds.reserve(subscriptions_size_);
  for (auto& subscription : subscriptions_) {
    std::lock_guard<std::mutex> lock2(subscription.mutex);
    if (subscription.map) {
        clouds.emplace_back(subscription.map);
    }
  }
  return clouds;
}

std::vector<Eigen::Matrix4f> MapMerge3d::getTransforms()
{
  std::lock_guard<std::mutex> lock(transforms_mutex_);
  return transforms_;
}

std::string MapMerge3d::robotNameFromTopic(const std::string& topic)
{
  std::string clean_topic = topic;
  if (clean_topic.front() == '/') clean_topic.erase(0, 1);
  
  size_t slash_pos = clean_topic.find('/');
  if (slash_pos != std::string::npos) {
      return clean_topic.substr(0, slash_pos);
  }
  return "";
}

bool MapMerge3d::isRobotMapTopic(const std::string& topic_name, const std::string& topic_type)
{
  if (topic_type != "sensor_msgs/msg/PointCloud2") return false;
  if (topic_name == merged_map_publisher_->get_topic_name()) return false;
  if (!endsWith(topic_name, robot_map_topic_)) return false;
  if (!robot_namespace_.empty() && topic_name.find(robot_namespace_) == std::string::npos) {
      return false;
  }
  return true;
}

static inline std::vector<geometry_msgs::msg::TransformStamped> computeTFTransforms(
    const std::vector<Eigen::Matrix4f>& transforms,
    const std::vector<PointCloudConstPtr>& maps, const std::string& world_frame)
{
  std::vector<geometry_msgs::msg::TransformStamped> result;
  if (transforms.size() != maps.size()) return result;

  result.reserve(transforms.size());
  for (const auto& transform : transforms) {
    Eigen::Affine3d affine;
    affine.matrix() = transform.cast<double>();
    result.emplace_back(tf2::eigenToTransform(affine));
  }
  
  for (size_t i = 0; i < result.size(); ++i) {
    result[i].header.frame_id = world_frame;
    result[i].child_frame_id = maps[i]->header.frame_id;
  }
  return result;
}

void MapMerge3d::publishTF()
{
  if (!tf_current_flag_.test_and_set()) {
    auto transforms = getTransforms();
    auto maps = getMaps();
    if (transforms.size() == maps.size()) {
        tf_transforms_ = computeTFTransforms(transforms, maps, world_frame_);
    }
  }
  
  if (tf_transforms_.empty()) return;

  rclcpp::Time now = this->now();
  for (auto& transform : tf_transforms_) {
    transform.header.stamp = now;
    tf_publisher_->sendTransform(transform);
  }
}

}  // namespace map_merge_3d

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<map_merge_3d::MapMerge3d>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}