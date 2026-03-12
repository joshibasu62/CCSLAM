#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    enum ControlState {
        kWaiting,           // Before arming
        kTakingOff,         // Climbing to target altitude
        kRotating,          // Rotating 270 degrees
        kPositionControl,   // Hovering / holding position
        kVelocityControl    // Following Nav2 cmd_vel
    };

    OffboardControl() :
        Node("offboard_control"),
        use_sim_time_(false)
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&OffboardControl::twist_callback, this, std::placeholders::_1));

        local_pos_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(), std::bind(&OffboardControl::local_pos_callback, this, std::placeholders::_1));

        status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", rclcpp::QoS(10).best_effort(), std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

        ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", rclcpp::QoS(10).best_effort(), std::bind(&OffboardControl::vehicle_cmd_ack_callback, this, std::placeholders::_1));

        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&OffboardControl::joy_callback, this, std::placeholders::_1));

        this->get_parameter("use_sim_time", use_sim_time_);

        offboard_setpoint_counter_ = 0;

        current_goal_.x = 0;
        current_goal_.y = 0;
        current_goal_.z = -1.5;    //  Changed to 1.5m
        current_goal_.heading = 0;

        control_State_ = kWaiting;
        velocity2d_ = true;
        last_request_ = this->get_clock()->now();
        arming_stamp_ = this->get_clock()->now();
        twist_stamp_ = this->get_clock()->now();

        // Rotation parameters
        takeoff_altitude_ = -1.5;          // NED: -1.5 = 1.5m above ground
        rotation_total_ = 270.0 * M_PI / 180.0;  // 270° in radians
        rotation_speed_ = 0.5;             // rad/s — how fast to rotate
        altitude_tolerance_ = 0.15;        // How close to target alt before rotating
        yaw_tolerance_ = 0.05;             // How close to target yaw (radians ≈ 3°)

        auto timer_callback = [this]() -> void {

            if (offboard_setpoint_counter_ == 10) {
                RCLCPP_INFO(get_logger(), "Setting offboard mode...");
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            }
            else if (offboard_setpoint_counter_ == 20) {
                if (!vehicle_status_.pre_flight_checks_pass ||
                    vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    offboard_setpoint_counter_ = 0;
                    RCLCPP_WARN(get_logger(),
                        "Offboard could not be set, waiting for preflight checks...");
                }
                else {
                    this->arm();
                }
            }

            if (offboard_setpoint_counter_ >= 21) {
                update_state();
            }
            else {
                ++offboard_setpoint_counter_;
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();
        };
        timer_ = this->create_wall_timer(50ms, timer_callback);
    }

private:

    //  Normalize angle to [-PI, PI]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void update_state() {
        if ((this->get_clock()->now() - last_request_ > rclcpp::Duration::from_seconds(1.0))) {
            if (!vehicle_status_.pre_flight_checks_pass ||
                vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                RCLCPP_ERROR(get_logger(), "Pre-flight checks failing or offboard disabled!");
            }
            else if ((this->get_clock()->now() - twist_stamp_).seconds() < 1.0) {
                if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
                    (twist_.linear.z < -0.4 && twist_.angular.z < -0.4)) {
                    this->arm();
                }
                else if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
                    twist_.linear.z < -0.4 && twist_.angular.z > 0.4) {
                    this->disarm();
                }
            }
            last_request_ = get_clock()->now();
        }


        // TAKING OFF check if altitude reached
        if (control_State_ == kTakingOff) {
            double alt_error = std::abs(local_pose_.z - takeoff_altitude_);
            if (alt_error < altitude_tolerance_) {
                RCLCPP_INFO(get_logger(), " Reached target altitude (%.2f m)! Starting 270° rotation...",
                    -local_pose_.z);

                // Record starting yaw and compute target yaw
                initial_yaw_ = local_pose_.heading;
                // 270° CCW: target = initial + 270° (in NED, positive yaw is CW, so we use negative)
                // But we'll track accumulated rotation instead
                accumulated_rotation_ = 0.0;
                last_yaw_ = initial_yaw_;
                target_yaw_ = normalizeAngle(initial_yaw_ - rotation_total_);  // CW rotation in NED

                // Set current yaw target incrementally
                current_yaw_target_ = initial_yaw_;

                control_State_ = kRotating;
                rotation_start_time_ = this->get_clock()->now();

                RCLCPP_INFO(get_logger(), "Initial yaw: %.2f°, Target yaw: %.2f° (rotating %.0f°)",
                    initial_yaw_ * 180.0 / M_PI,
                    target_yaw_ * 180.0 / M_PI,
                    rotation_total_ * 180.0 / M_PI);
            }
        }

        // ROTATING check if rotation complete
        if (control_State_ == kRotating) {
            // Track accumulated rotation using yaw difference
            double current_yaw = local_pose_.heading;
            double yaw_diff = normalizeAngle(current_yaw - last_yaw_);
            accumulated_rotation_ += std::abs(yaw_diff);
            last_yaw_ = current_yaw;

            // Update incremental yaw target
            double dt = 0.05;  // 50ms timer
            current_yaw_target_ = normalizeAngle(current_yaw_target_ - rotation_speed_ * dt);

            // Check if we've completed the rotation
            if (accumulated_rotation_ >= rotation_total_ - yaw_tolerance_) {
                RCLCPP_INFO(get_logger(), " Rotation complete! Accumulated: %.1f° (target: %.0f°). Hovering...",
                    accumulated_rotation_ * 180.0 / M_PI,
                    rotation_total_ * 180.0 / M_PI);

                // Lock current position as hover goal
                current_goal_ = local_pose_;
                current_goal_.heading = target_yaw_;
                control_State_ = kPositionControl;
                arming_stamp_ = this->get_clock()->now();  // Reset for Nav2 5s delay

                RCLCPP_INFO(get_logger(), "Ready for Nav2 goals! Position hold at (%.2f, %.2f, %.2f) heading=%.1f°",
                    current_goal_.x, current_goal_.y, current_goal_.z,
                    current_goal_.heading * 180.0 / M_PI);
            }

            // Safety timeout: if rotation takes too long (e.g., 60 seconds)
            if ((this->get_clock()->now() - rotation_start_time_).seconds() > 60.0) {
                RCLCPP_WARN(get_logger(), "⚠️ Rotation timeout! Switching to hover.");
                current_goal_ = local_pose_;
                control_State_ = kPositionControl;
                arming_stamp_ = this->get_clock()->now();
            }
        }
    }

    void arm()
    {
        arming_stamp_ = get_clock()->now();
        current_goal_ = local_pose_;
        current_goal_.z = takeoff_altitude_;  //  Use 1.5m altitude
        RCLCPP_INFO(get_logger(), "Vehicle arming..");
        RCLCPP_INFO(get_logger(), "Take off to %.1f meters... position=(%.2f, %.2f, %.2f)",
            -takeoff_altitude_, current_goal_.x, current_goal_.y, current_goal_.z);
        control_State_ = kTakingOff;  //  Start takeoff state
        publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand::ARMING_ACTION_ARM);
    }

    void disarm()
    {
        RCLCPP_INFO(get_logger(), "Vehicle disarming...");
        publish_vehicle_command(
            VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand::ARMING_ACTION_DISARM);
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        double yaw = local_pose_.heading;
        double cosyaw = cos(yaw);
        double sinyaw = sin(yaw);

        switch (control_State_) {

            case kWaiting:
                // Send zero position before arming
                msg.position[0] = 0.0f;
                msg.position[1] = 0.0f;
                msg.position[2] = 0.0f;
                msg.yaw = 0.0f;
                break;

            case kTakingOff:
                // Position control: climb to target altitude
                msg.position[0] = current_goal_.x;
                msg.position[1] = current_goal_.y;
                msg.position[2] = takeoff_altitude_;
                msg.yaw = current_goal_.heading;  // Maintain current heading
                msg.velocity[0] = NAN;
                msg.velocity[1] = NAN;
                msg.velocity[2] = NAN;
                msg.yawspeed = NAN;
                break;

            case kRotating:
                // Hold position, rotate using incremental yaw target
                msg.position[0] = current_goal_.x;
                msg.position[1] = current_goal_.y;
                msg.position[2] = takeoff_altitude_;
                msg.yaw = current_yaw_target_;  // Incrementally rotating target
                msg.velocity[0] = NAN;
                msg.velocity[1] = NAN;
                msg.velocity[2] = NAN;
                msg.yawspeed = NAN;
                break;

            case kPositionControl:
            {
                // Check for cmd_vel timeout
                double time_since_last_twist = (this->get_clock()->now() - twist_stamp_).seconds();
                (void)time_since_last_twist;  // Used below in velocity control transition

                msg.position[0] = current_goal_.x;
                msg.position[1] = current_goal_.y;
                msg.position[2] = current_goal_.z;
                msg.yaw = current_goal_.heading;
                msg.velocity[0] = NAN;
                msg.velocity[1] = NAN;
                msg.velocity[2] = NAN;
                msg.yawspeed = NAN;
                break;
            }

            case kVelocityControl:
            {
                double time_since_last_twist = (this->get_clock()->now() - twist_stamp_).seconds();

                // Timeout: switch back to position hold
                if (time_since_last_twist > 0.5) {
                    control_State_ = kPositionControl;
                    current_goal_ = local_pose_;
                    if (std::abs(current_goal_.z) < 0.1) {
                        current_goal_.z = takeoff_altitude_;
                    }
                    RCLCPP_INFO(get_logger(), "Cmd_vel timeout! Position hold (%.2f, %.2f, %.2f)",
                        current_goal_.x, current_goal_.y, current_goal_.z);

                    // Publish position hold for this tick
                    msg.position[0] = current_goal_.x;
                    msg.position[1] = current_goal_.y;
                    msg.position[2] = current_goal_.z;
                    msg.yaw = current_goal_.heading;
                    msg.velocity[0] = NAN;
                    msg.velocity[1] = NAN;
                    msg.velocity[2] = NAN;
                    msg.yawspeed = NAN;
                    break;
                }

                // Body-to-NED velocity rotation
                msg.velocity[0] = twist_.linear.x * cosyaw - twist_.linear.y * sinyaw;
                msg.velocity[1] = twist_.linear.x * sinyaw + twist_.linear.y * cosyaw;
                msg.velocity[2] = velocity2d_ ? NAN : -twist_.linear.z;

                msg.position[0] = NAN;
                msg.position[1] = NAN;
                msg.position[2] = velocity2d_ ? current_goal_.z : NAN;

                msg.yaw = NAN;
                msg.yawspeed = -twist_.angular.z;
                break;
            }
        }

        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist_ = *msg;
        twist_stamp_ = this->get_clock()->now();

        // Only allow Nav2 control after takeoff + rotation is complete
        if ((twist_stamp_ - arming_stamp_).seconds() > 5.0 && control_State_ == kPositionControl)
        {
            current_goal_.z = local_pose_.z;
            if (std::abs(current_goal_.z) < 0.1) {
                current_goal_.z = takeoff_altitude_;
            }
            RCLCPP_INFO(get_logger(), "Nav2 /cmd_vel → velocity control (alt=%.2f)", current_goal_.z);
            control_State_ = kVelocityControl;
        }
    }

    void local_pos_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        local_pose_ = *msg;
    }

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        vehicle_status_ = *msg;
    }

    void vehicle_cmd_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
    {
        if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
            RCLCPP_INFO(get_logger(), "Command accepted!");
        }
        else {
            RCLCPP_ERROR(get_logger(), "Command rejected!");
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        velocity2d_ = (msg->buttons[5] != 1);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_pos_subscriber_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscriber_;
    rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    uint64_t offboard_setpoint_counter_;

    geometry_msgs::msg::Twist twist_;
    rclcpp::Time twist_stamp_;
    rclcpp::Time arming_stamp_;

    VehicleLocalPosition local_pose_;
    VehicleLocalPosition current_goal_;
    px4_msgs::msg::VehicleStatus vehicle_status_;

    ControlState control_State_;
    bool velocity2d_;
    bool use_sim_time_;
    rclcpp::Time last_request_;

    //  NEW: Takeoff & rotation parameters
    double takeoff_altitude_;          // NED altitude (negative = up)
    double rotation_total_;            // Total rotation in radians
    double rotation_speed_;            // Rotation speed (rad/s)
    double altitude_tolerance_;        // Altitude error tolerance
    double yaw_tolerance_;             // Yaw error tolerance

    // Rotation tracking
    double initial_yaw_;               // Yaw at start of rotation
    double target_yaw_;                // Final target yaw
    double current_yaw_target_;        // Incrementally updated yaw setpoint
    double accumulated_rotation_;      // How much we've rotated so far
    double last_yaw_;                  // Previous yaw for tracking
    rclcpp::Time rotation_start_time_; // Safety timeout
};

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control (1.5m takeoff + 270° rotation)..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
