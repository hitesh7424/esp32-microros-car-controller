#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class WheelSpeedController : public rclcpp::Node
{
public:
    WheelSpeedController() : Node("wheel_speed_controller")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/car1/wheel_speeds", 1);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/ctrl1/cmd_vel", 1, std::bind(&WheelSpeedController::cmdVelCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Wheel Speed Controller Node has been started.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg->linear.x > 0 && msg->angular.z == 0)
        {
            moveForward(msg->linear.x * 200);
        }
        else if (msg->linear.x < 0 && msg->angular.z == 0)
        {
            moveReverse(msg->linear.x * 200);
        }
        else if (msg->angular.z > 0 && msg->linear.x == 0)
        {
            turnLeft(msg->angular.z * 150);
        }
        else if (msg->angular.z < 0 && msg->linear.x == 0)
        {
            turnRight(msg->angular.z * 150);
        }
        else if (msg->linear.x == 0 && msg->angular.z == 0)
        {
            stop();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported movement command received.");
        }
    }

    void moveForward(double speed)
    {
        auto wheel_speeds = geometry_msgs::msg::Twist();
        wheel_speeds.linear.x = -speed;
        wheel_speeds.angular.x = -speed;
        wheel_speeds.linear.y = -speed;
        wheel_speeds.angular.y = -speed;
        publishWheelSpeeds(wheel_speeds, "Moving forward");
    }

    void moveReverse(double speed)
    {
        auto wheel_speeds = geometry_msgs::msg::Twist();
        wheel_speeds.linear.x = -speed;
        wheel_speeds.angular.x = -speed;
        wheel_speeds.linear.y = -speed;
        wheel_speeds.angular.y = -speed;
        publishWheelSpeeds(wheel_speeds, "Moving reverse");
    }

    void turnLeft(double speed)
    {
        auto wheel_speeds = geometry_msgs::msg::Twist();
        wheel_speeds.linear.x = -speed;
        wheel_speeds.angular.x = speed;
        wheel_speeds.linear.y = -speed;
        wheel_speeds.angular.y = speed;
        publishWheelSpeeds(wheel_speeds, "Turning left");
    }

    void turnRight(double speed)
    {
        auto wheel_speeds = geometry_msgs::msg::Twist();
        wheel_speeds.linear.x = -speed;
        wheel_speeds.angular.x = speed;
        wheel_speeds.linear.y = -speed;
        wheel_speeds.angular.y = speed;
        publishWheelSpeeds(wheel_speeds, "Turning right");
    }

    void stop()
    {
        auto wheel_speeds = geometry_msgs::msg::Twist();
        wheel_speeds.linear.x = 0.0;
        wheel_speeds.angular.x = 0.0;
        wheel_speeds.linear.y = 0.0;
        wheel_speeds.angular.y = 0.0;
        publishWheelSpeeds(wheel_speeds, "Stopping");
    }

    void publishWheelSpeeds(const geometry_msgs::msg::Twist &wheel_speeds, const std::string &action)
    {
        publisher_->publish(wheel_speeds);
        RCLCPP_INFO(this->get_logger(), "%s: Front -> linear.x=%.2f, angular.x=%.2f; Rear -> linear.y=%.2f, angular.y=%.2f",
                    action.c_str(), wheel_speeds.linear.x, wheel_speeds.angular.x, wheel_speeds.linear.y, wheel_speeds.angular.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelSpeedController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}