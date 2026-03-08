#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <tf2_eigen/tf2_eigen.hpp> 
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mutex>
#include <chrono>
using namespace std::chrono_literals;

class RosOdometry2VehicleOdometry : public rclcpp::Node
{
public:
    RosOdometry2VehicleOdometry() : 
        Node("ros_odometry_to_vehicle_odometry_wo_map"),
        repeat_odom_(false)
    {
        odom_topic_ = this->declare_parameter("odom_topic", std::string("/rtabmap/odom"));
        vehicle_odometry_topic_ = this->declare_parameter("vehicle_odometry_topic", std::string("/fmu/in/vehicle_visual_odometry"));
        repeat_odom_ = this->declare_parameter("repeat_odom", false);
        
        create_pub_sub();
    
        if(repeat_odom_) {
            // Publish pose at steady rate and restamp with now
            auto timer_callback = [this]() -> void {
                px4_msgs::msg::VehicleOdometry msg;
                {
                    std::scoped_lock lock(mutex_); 
                    msg = current_vehicle_odometry_;
                    uint64_t timestamp_usec = this->get_clock()->now().nanoseconds() / 1000;
                    msg.timestamp = timestamp_usec;
                    msg.timestamp_sample = timestamp_usec;
                }
                
                vehicle_odometry_publisher_->publish(msg);
            };
            timer_ = this->create_wall_timer(50ms, timer_callback);
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odometry)
    {
        // NO MORE TF LOOKUPS! We take the pure odometry directly.
        px4_msgs::msg::VehicleOdometry msg;
        msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        
        // Convert ROS nanoseconds to PX4 microseconds
        uint64_t timestamp_usec = rclcpp::Time(odometry->header.stamp).nanoseconds() / 1000;
        msg.timestamp = timestamp_usec;
        msg.timestamp_sample = timestamp_usec; // CRITICAL: EKF2 requires this!

        // 1. Extract raw ENU position and orientation directly from the Odometry message
        Eigen::Quaterniond orientationENU;
        tf2::fromMsg(odometry->pose.pose.orientation, orientationENU);

        Eigen::Vector3d positionENU;
        tf2::fromMsg(odometry->pose.pose.position, positionENU);

        // 2. Convert ENU directly to NED (PX4's coordinate system)
        Eigen::Quaterniond orientationNED = px4_ros_com::frame_transforms::ros_to_px4_orientation(orientationENU);
        Eigen::Vector3d positionNED = px4_ros_com::frame_transforms::transform_static_frame(positionENU, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
        
        // Pass the covariance directly into the static frame transformer
        px4_ros_com::frame_transforms::Covariance6d covNED = px4_ros_com::frame_transforms::transform_static_frame(odometry->pose.covariance, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);

        // 3. Assign Pose to PX4 message
        msg.q[0] = orientationNED.w();
        msg.q[1] = orientationNED.x();
        msg.q[2] = orientationNED.y();
        msg.q[3] = orientationNED.z();
        msg.position[0] = positionNED.x();
        msg.position[1] = positionNED.y();
        msg.position[2] = positionNED.z();

        // Position Variance (Inject 0.05 if zero)
        msg.position_variance[0] = (covNED[0] < 0.001) ? 0.05 : covNED[0];
        msg.position_variance[1] = (covNED[7] < 0.001) ? 0.05 : covNED[7];
        msg.position_variance[2] = (covNED[14] < 0.001) ? 0.05 : covNED[14];

        // Orientation Variance (Inject 0.05 if zero)
        msg.orientation_variance[0] = (covNED[21] < 0.001) ? 0.05 : covNED[21];
        msg.orientation_variance[1] = (covNED[28] < 0.001) ? 0.05 : covNED[28];
        msg.orientation_variance[2] = (covNED[35] < 0.001) ? 0.05 : covNED[35];

        // 4. Twist is published in base frame, convert ENU to NED
        msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
        Eigen::Vector3d linearROS, angularROS;
        tf2::fromMsg(odometry->twist.twist.linear, linearROS);
        tf2::fromMsg(odometry->twist.twist.angular, angularROS);
        
        Eigen::Vector3d linearFRD = px4_ros_com::frame_transforms::transform_static_frame(linearROS, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
        msg.velocity[0] = linearFRD.x();
        msg.velocity[1] = linearFRD.y();
        msg.velocity[2] = linearFRD.z();
        
        Eigen::Vector3d angularFRD = px4_ros_com::frame_transforms::transform_static_frame(angularROS, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
        msg.angular_velocity[0] = angularFRD.x();
        msg.angular_velocity[1] = angularFRD.y();
        msg.angular_velocity[2] = angularFRD.z();
        
        px4_ros_com::frame_transforms::Covariance6d velocityCovNED = px4_ros_com::frame_transforms::transform_static_frame(odometry->twist.covariance, px4_ros_com::frame_transforms::StaticTF::ENU_TO_NED);
        
        // Velocity Variance (Inject 0.05 if zero)
        msg.velocity_variance[0] = (velocityCovNED[0] < 0.001) ? 0.05 : velocityCovNED[0];
        msg.velocity_variance[1] = (velocityCovNED[7] < 0.001) ? 0.05 : velocityCovNED[7];
        msg.velocity_variance[2] = (velocityCovNED[14] < 0.001) ? 0.05 : velocityCovNED[14];

        // 5. Publish
        if(repeat_odom_)
        {
            std::scoped_lock lock(mutex_); 
            current_vehicle_odometry_ = msg;
        }
        else {
            vehicle_odometry_publisher_->publish(msg);
        }
    }

private:
    void create_pub_sub() {
        if(vehicle_odometry_publisher_) vehicle_odometry_publisher_.reset();
        if(subscription_) subscription_.reset();

        vehicle_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            vehicle_odometry_topic_, 10);

        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&RosOdometry2VehicleOdometry::odom_callback, this, std::placeholders::_1));
    }
    
    std::string odom_topic_;
    std::string vehicle_odometry_topic_;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;
    px4_msgs::msg::VehicleOdometry current_vehicle_odometry_;
    std::mutex mutex_;

    bool repeat_odom_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosOdometry2VehicleOdometry>());
    rclcpp::shutdown();
    return 0;
}