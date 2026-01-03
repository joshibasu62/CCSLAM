#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <iostream>
#include <map>

class TeleopKeyboard : public rclcpp::Node
{
public:
    TeleopKeyboard() : Node("teleop_twist_keyboard")
    {
        // Declare parameter for prefix
        instance_ = this->declare_parameter<std::string>("instance", "");

        // QoS profile
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos.keep_last(10);

        // Create publishers with prefix
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(instance_ + "/cmd_vel", qos);
        active_pub_ = this->create_publisher<std_msgs::msg::Bool>(instance_ + "/teleop/active", qos);

        // Set initial speeds
        speed_ = 0.5;
        turn_ = 1.0;

        // Print help
        std::cout << start << std::endl;
        std::cout << vels(speed_, turn_) << std::endl;

        // Publish active = true at startup
        std_msgs::msg::Bool active_msg;
        active_msg.data = true;
        active_pub_->publish(active_msg);
    }

    void spin()
    {
        struct termios orig_settings;
        tcgetattr(STDIN_FILENO, &orig_settings);
        struct termios raw = orig_settings;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        try
        {
            while (rclcpp::ok())
            {
                char c = getKey();
                handleKey(c);

                // Publish active = true
                std_msgs::msg::Bool active_msg;
                active_msg.data = true;
                active_pub_->publish(active_msg);
            }
        }
        catch (...)
        {
            // On exception, reset
        }

        // Stop robot on shutdown
        geometry_msgs::msg::Twist twist;
        pub_->publish(twist);

        // Publish active = false
        std_msgs::msg::Bool active_msg;
        active_msg.data = false;
        active_pub_->publish(active_msg);

        tcsetattr(STDIN_FILENO, TCSADRAIN, &orig_settings);
        std::cout << "\nTeleop shutdown. Sent teleop/active = False\n";
    }

private:
    char getKey()
    {
        fd_set set;
        struct timeval timeout;

        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
        if (rv == -1)
            return 0;
        else if (rv == 0)
            return 0;
        else
        {
            char buf = 0;
            if (read(STDIN_FILENO, &buf, 1) < 0)
                return 0;
            return buf;
        }
    }

    void handleKey(char key)
    {   
        bool keyPressed = false;
        if (moveBindings_.count(key))
        {
            auto b = moveBindings_[key];
            pitch_ = b[0];
            roll_ = b[1];
            throttle_ = b[2];
            yaw_ = b[3];
            keyPressed = true;
            // printf("Pitch %.2f, roll %.2f, throttle %.2f, yaw %.2f \n", pitch_, roll_, throttle_, yaw_);
        }
        else if (speedBindings_.count(key))
        {
            speed_ *= speedBindings_[key].first;
            turn_ *= speedBindings_[key].second;
            std::cout << vels(speed_, turn_) << std::endl;
            keyPressed = true;
            // pitch_ = roll_ = throttle_ = yaw_ = 0;
        }
        else
        {
            pitch_ = roll_ = throttle_ = yaw_ = 0;

            if (key == 3) // CTRL-C
                rclcpp::shutdown();
        }

        if (keyPressed)
        {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = pitch_ * speed_;
        twist.linear.y = roll_ * speed_;
        twist.linear.z = (throttle_ * speed_);
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = yaw_ * turn_;
        // printf("Twist: linear=(%.2f, %.2f, %.2f) angular=(%.2f, %.2f, %.2f)\n",
        //     twist.linear.x, twist.linear.y, twist.linear.z,
        //     twist.angular.x, twist.angular.y, twist.angular.z);
        pub_->publish(twist);
        }
    }

    std::string vels(double speed, double turn)
    {
        char buf[100];
        snprintf(buf, sizeof(buf), "current: \tspeed %.2f\tturn %.2f ", speed, turn);
        return std::string(buf);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;

    // Movement state
    double speed_, turn_;
    double pitch_ = 0, roll_ = 0, throttle_ = 0, yaw_ = 0;
    std::string instance_;

    // Bindings
    std::map<char, std::array<int, 4>> moveBindings_ = {
        {'w', {1, 0, 0, 0}}, 
        {'s', {-1, 0, 0, 0}}, 
        {'a', {0, -1, 0, 0}},
        {'d', {0, 1, 0, 0}}, 
        {'i', {0, 0, 1, 0}},  
        {'k', {0, 0, -1, 0}},
        {'j', {0, 0, 0, 1}}, 
        {'l', {0, 0, 0, -1}}};

    std::map<char, std::pair<double, double>> speedBindings_ = {
        {'+', {1.1, 1.1}},
        {'-', {0.9, 0.9}}};

    const std::string start =
        "Control drone with keys\n"
        "------------------------------------\n"
        "KEYS:\n"
        "  W/S : Pitch forward/backward\n"
        "  A/D : Roll left/right\n"
        "  J/L : Yaw left/right\n"
        "  I/K : Throttle up/down\n\n"
        "  +/- : Increase/decrease speeds \n"
        "CTRL-C to quit\n";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
