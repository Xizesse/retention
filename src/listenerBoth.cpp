#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "sensor_msgs/msg/range.hpp"
#include "drone_interfaces/msg/state.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription1_ = this->create_subscription<drone_interfaces::msg::State>(
      "state", 1, std::bind(&MinimalSubscriber::topic1_callback, this, _1));

      subscription2_ = this->create_subscription<sensor_msgs::msg::Range>(
      "range", 1, std::bind(&MinimalSubscriber::topic2_callback, this, _1));

      
    }

  private:
    void topic1_callback(const drone_interfaces::msg::State::SharedPtr msg)
    {
      latest_msg1_ = msg;
      process_messages();
    }

    void topic2_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
      latest_msg2_ = msg;
      process_messages();
    }


    void process_messages()
    {
      if(latest_msg1_ && latest_msg2_ )
      {
        RCLCPP_INFO(this->get_logger(), "Mode: '%d', Armed: '%s' Range: '%f'", latest_msg1_->mode, latest_msg1_->armed? "true" : "false", latest_msg2_->range);
        latest_msg1_.reset();
        latest_msg2_.reset();
      
      }
    }

    rclcpp::Subscription<drone_interfaces::msg::State>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription2_;
    drone_interfaces::msg::State::SharedPtr latest_msg1_;
    sensor_msgs::msg::Range::SharedPtr latest_msg2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}