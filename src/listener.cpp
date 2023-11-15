
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

class MultiSubscriber : public rclcpp::Node
{
public:
  MultiSubscriber()
  : Node("multi_subscriber")
  {
    subscription1_ = this->create_subscription<std_msgs::msg::String>(
      "topic1", 10, std::bind(&MultiSubscriber::topic1_callback, this, std::placeholders::_1));
    subscription2_ = this->create_subscription<std_msgs::msg::Bool>(
      "topic2", 10, std::bind(&MultiSubscriber::topic2_callback, this, std::placeholders::_1));
    subscription3_ = this->create_subscription<std_msgs::msg::Float32>(
      "topic3", 10, std::bind(&MultiSubscriber::topic3_callback, this, std::placeholders::_1));
  }

private:
  void topic1_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    latest_msg1_ = msg;
    process_messages();
  }

  void topic2_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    latest_msg2_ = msg;
    process_messages();
  }

  void topic3_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_msg3_ = msg;
    process_messages();
  }

 void process_messages()
  {
    if (latest_msg1_ && latest_msg2_ && latest_msg3_)
    {
      RCLCPP_INFO(this->get_logger(), "Received synchronized messages:\nString: '%s'\nBool: '%s'\nFloat: '%f'",
      latest_msg1_->data.c_str(), latest_msg2_->data ? "true" : "false", latest_msg3_->data);
      latest_msg1_.reset();
      latest_msg2_.reset();
      latest_msg3_.reset();
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription2_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription3_;
  std_msgs::msg::String::SharedPtr latest_msg1_;
  std_msgs::msg::Bool::SharedPtr latest_msg2_;
  std_msgs::msg::Float32::SharedPtr latest_msg3_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiSubscriber>());
  rclcpp::shutdown();
  return 0;
}