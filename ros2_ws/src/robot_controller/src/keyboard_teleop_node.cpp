#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>   

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class KeyboardTeleopPublisher : public rclcpp::Node
{
  public:
    KeyboardTeleopPublisher()
    : Node("keyboardteleop_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&KeyboardTeleopPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      //message.data = "Hello, world! " + std::to_string(count_++);
      message.linear.x = 1.0;
      message.angular.z = 0.5;
      RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f angular.z=%.2f", message.linear.x, message.angular.z);
      publisher_->publish(message);

      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopPublisher>());
  rclcpp::shutdown();
  return 0;
}