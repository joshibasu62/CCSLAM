#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <Eigen/Eigen>

static const std::string kModeName = "Teleoperation";

class VelocityControlMode : public px4_ros2::ModeBase
{
public:
  explicit VelocityControlMode(rclcpp::Node & node)
  : ModeBase(node, kModeName)
  , _node(node)
  {
    // _node.set_logger_level(rclcpp::Logger::Level::Debug);
    // Create trajectory setpoint controller
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    _clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    // Subscribe to cmd_vel topic
    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        _latest_cmd_vel = msg;
        _last_cmd_time = _clock->now();
      });
  }

  void onActivate() override
  {
    _last_cmd_time = _clock->now();
    RCLCPP_DEBUG(node().get_logger(), "Velocity control mode activated!");
  }

  void onDeactivate() override
  {
    RCLCPP_DEBUG(node().get_logger(), "Velocity control mode deactivated!");
  }

  void updateSetpoint(float) override
  {
    const auto current_time = _clock->now();
    Eigen::Vector3f velocity_body;
    std::optional<float> yaw_rate = std::nullopt;
    Eigen::Vector3f velocity_ned;
    if (!_latest_cmd_vel) {
        // No commands received yet, send zero velocity
        Eigen::Vector3f zero_velocity = Eigen::Vector3f::Zero();
        _trajectory_setpoint->update(
            zero_velocity,
            {},
            {},
            std::nullopt
        );
        return;
    }
    if ((current_time - _last_cmd_time).seconds()<=0.25) {
      // Convert Twist (body frame) to NED frame velocities
      // Assuming cmd_vel is in body frame (common for joysticks):
      // linear.x = forward, linear.y = left, linear.z = up
      // angular.z = yaw rate
      float yaw = _vehicle_attitude->yaw(); 
      
      velocity_body.x() = _latest_cmd_vel->linear.x;  // forward
      velocity_body.y() = _latest_cmd_vel->linear.y;  // left
      velocity_body.z() = _latest_cmd_vel->linear.z; 
      
      // float yaw_rate = _latest_cmd_vel->angular.z;
      velocity_ned.x() = velocity_body.x() * std::cos(yaw) - velocity_body.y() * std::sin(yaw);
      velocity_ned.y() = velocity_body.x() * std::sin(yaw) + velocity_body.y() * std::cos(yaw);
      velocity_ned.z() = -velocity_body.z();  // body-up → NED-down
      // Send to PX4
      yaw_rate = -_latest_cmd_vel->angular.z;  
    }
    _trajectory_setpoint->update(
      velocity_ned,
      {},  // No acceleration feedforward
      {},  // No yaw setpoint (use rate instead)
      yaw_rate
    );
    }
  

private:
  void loadParameters();
  rclcpp::Node &_node;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  geometry_msgs::msg::Twist::SharedPtr _latest_cmd_vel;
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<rclcpp::Clock> _clock;
  rclcpp::Time _last_cmd_time;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("velocity_control_node");
  
  VelocityControlMode velocity_mode(*node);
  
  // Register the mode
  velocity_mode.doRegister();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}