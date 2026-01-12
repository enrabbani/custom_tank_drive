#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class BaseController : public rclcpp::Node
{
  public:
    BaseController()
    : Node("base_controller")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&BaseController::topic_callback, this, _1));
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: linear.x=%.2f angular.z=%.2f", msg.linear.x, msg.angular.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseController>());
  rclcpp::shutdown();
  return 0;
}