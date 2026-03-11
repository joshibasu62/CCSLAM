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

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	enum ControlState {
		kPositionControl,
		kVelocityControl
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
		current_goal_.z = -1.3;
		current_goal_.heading = 0;

		control_State_ = kPositionControl;
		velocity2d_ = true;
		last_request_ = this->get_clock()->now();
		arming_stamp_ = this->get_clock()->now();
		twist_stamp_ = this->get_clock()->now();

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
		timer_ = this->create_wall_timer(50ms, timer_callback);  // ✅ 20Hz for smoother control
	}

private:

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
	}

	void arm()
	{
		arming_stamp_ = get_clock()->now();
		current_goal_ = local_pose_;
		current_goal_.z = current_goal_.z - 1.3;
		RCLCPP_INFO(get_logger(), "Vehicle arming..");
		RCLCPP_INFO(get_logger(), "Take off to 1.3 meters... position=(%f,%f,%f)",
			current_goal_.x, current_goal_.y, current_goal_.z);
		control_State_ = kPositionControl;
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
		// ✅ FIX 3: Use reliable time comparison
		double time_since_last_twist = (this->get_clock()->now() - twist_stamp_).seconds();

		// ✅ FIX 1: Increased timeout from 0.1s to 0.5s
		if (control_State_ == kVelocityControl && time_since_last_twist > 0.5)
		{
			control_State_ = kPositionControl;
			current_goal_ = local_pose_;
			RCLCPP_INFO(get_logger(), "Cmd_vel timeout (%.2fs)! Switching to position hold (x=%f, y=%f, z=%f)",
				time_since_last_twist, current_goal_.x, current_goal_.y, current_goal_.z);
		}

		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		double yaw = local_pose_.heading;
		double cosyaw = cos(yaw);
		double sinyaw = sin(yaw);

		if (control_State_ == kVelocityControl)
		{
			// ✅ FIX 2: Correct body-to-NED velocity rotation
			// Nav2 body frame: x=forward, y=left
			// PX4 NED world frame: [0]=North, [1]=East
			// Rotation: v_world = R(yaw) * v_body
			//   v_north = v_forward * cos(yaw) - v_left * sin(yaw)
			//   v_east  = v_forward * sin(yaw) + v_left * cos(yaw)
			msg.velocity[0] = twist_.linear.x * cosyaw - twist_.linear.y * sinyaw;
			msg.velocity[1] = twist_.linear.x * sinyaw + twist_.linear.y * cosyaw;
			msg.velocity[2] = velocity2d_ ? NAN : -twist_.linear.z;

			msg.position[0] = NAN;
			msg.position[1] = NAN;
			msg.position[2] = velocity2d_ ? current_goal_.z : NAN;  // Hold altitude in 2D mode

			msg.yaw = NAN;
			msg.yawspeed = -twist_.angular.z;  // Nav2 CCW+ → PX4 CW+
		}
		else  // kPositionControl
		{
			msg.velocity[0] = NAN;
			msg.velocity[1] = NAN;
			msg.velocity[2] = NAN;

			msg.position[0] = current_goal_.x;
			msg.position[1] = current_goal_.y;
			msg.position[2] = current_goal_.z;

			msg.yaw = current_goal_.heading;
			msg.yawspeed = NAN;
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

		if ((twist_stamp_ - arming_stamp_).seconds() > 5.0 && control_State_ == kPositionControl)
		{
			// Lock current altitude as target for 2D navigation
			current_goal_.z = local_pose_.z;
			RCLCPP_INFO(get_logger(), "Receiving Nav2 /cmd_vel → Switching to velocity control (hold alt=%f)", current_goal_.z);
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
};

int main(int argc, char *argv[])
{
	std::cout << "Starting fixed offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}