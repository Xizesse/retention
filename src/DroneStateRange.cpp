#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
//include msgs Range and State from msgs
//#include "retention/msg/Range.hpp"
//#include "retention/msg/State.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "drone_interfaces/msg/state.hpp"
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher1_ = this->create_publisher<sensor_msgs::msg::Range>("range", 1);
      timer1_ = this->create_wall_timer(
      200ms, std::bind(&MinimalPublisher::timer_callback1, this)); // 5Hz

      publisher2_ = this->create_publisher<drone_interfaces::msg::State>("state", 1);
      timer2_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisher::timer_callback2, this)); // 1 Hz
    }

  private:
    void timer_callback1()
    {
      auto message = sensor_msgs::msg::Range();
      message.header.stamp = this->now();
      message.header.frame_id = "frame_id";
      message.range = 1.5; //Valor ao calhas
      //info the range and the time stamp
  
      publisher1_->publish(message);
    }

    void timer_callback2()
    {
      auto message = drone_interfaces::msg::State();
      
      /*
      std_msgs/Header header
      bool connected
      bool armed
      bool guided
      bool manual_input
      string mode
      uint8 system_status
      */

      message.header.stamp = this->now();
      message.header.frame_id = "frame_id";
      message.connected = true;
      message.armed = true;
      message.guided = true;
      message.manual_input = true;
      message.mode = "LAND";
      message.system_status = 1;
      //info the mode and armed and the time stamp
      RCLCPP_INFO(this->get_logger(), "State: '%s' armed : '%s'", message.mode.c_str(), message.armed? "true" : "false" );
      publisher2_->publish(message);
    }

    

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<drone_interfaces::msg::State>::SharedPtr publisher2_;
    
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}