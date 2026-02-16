#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>   

//Termios
#include <termios.h>
#include <unistd.h>
#include <iostream>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

char getch() {
    char buf = 0;
    struct termios old = {};
    tcgetattr(STDIN_FILENO, &old);
    struct termios newt = old;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    read(STDIN_FILENO, &buf, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &old);
    return buf;
}



class KeyboardTeleopPublisher : public rclcpp::Node
{
  public:
    KeyboardTeleopPublisher()
    : Node("keyboardteleop_publisher")//, count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&KeyboardTeleopPublisher::timer_callback, this));
      keyboard_thread_ = std::thread(&KeyboardTeleopPublisher::keyboard_loop, this);
    }

  private:
    void timer_callback()
    {
      auto cmd = geometry_msgs::msg::Twist();
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd = cmd_;
      }
      RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f angular.z=%.2f", cmd.linear.x, cmd.angular.z);
      publisher_->publish(cmd);

      
    }

    void keyboard_loop()
    {
      while(rclcpp::ok())
      {
        char c = getch();

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_.linear.x = 0;
        cmd_.angular.z = 0;
        switch(c)
        {
          case 'w':
            cmd_.linear.x = 1.0;
            break;
          case 's':
            cmd_.linear.x = -1.0;
            break;
          case 'a':
            cmd_.angular.z = 1.0;
            break;
          case 'd':
            cmd_.angular.z = -1.0;
            break;
          case 'q':
            rclcpp::shutdown();
            break;
        }
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist cmd_;
    std::thread keyboard_thread_;
   // size_t count_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleopPublisher>());
  rclcpp::shutdown();
  return 0;
}