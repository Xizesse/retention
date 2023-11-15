#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher1_ = this->create_publisher<std_msgs::msg::String>("estado", 1);
      timer1_ = this->create_wall_timer(
      200ms, std::bind(&MinimalPublisher::timer_callback1, this)); // 5Hz

      publisher2_ = this->create_publisher<std_msgs::msg::Float64>("range", 1);
      timer2_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback2, this)); // 1 Hz

      publisher3_ = this->create_publisher<std_msgs::msg::Bool>("armed", 1);
      timer3_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback3, this)); // 1Hz
    }

  private:
    void timer_callback1()
    {
      auto message = std_msgs::msg::String();
      message.data = "LAND"; // Replace with your string value
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s' to estado", message.data.c_str());
      publisher1_->publish(message);
    }

    void timer_callback2()
    {
      auto message = std_msgs::msg::Float64();
      message.data = 1.5; // Replace with your float value
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f' to range", message.data);
      publisher2_->publish(message);
    }

    void timer_callback3()
    {
      auto message = std_msgs::msg::Bool();
      message.data = true; // Replace with your bool value
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s' to armed", message.data ? "true" : "false");
      publisher3_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;
    rclcpp::TimerBase::SharedPtr timer3_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher3_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}