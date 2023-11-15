#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "drone_interfaces/msg/state.hpp"
#include "sensor_msgs/msg/range.hpp"

class ConverteNode : public rclcpp::Node
{
public:
    ConverteNode()
    : Node("converte_node")
    {
        //publishers
        state_publisher_ = this->create_publisher<drone_interfaces::msg::State>("state", 1);
        range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("range", 1);
        //subscritores
        mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "mode", 1, std::bind(&ConverteNode::mode_callback, this, std::placeholders::_1));
        armed_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "armed", 1, std::bind(&ConverteNode::armed_callback, this, std::placeholders::_1));
        distancia_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "distancia", 1, std::bind(&ConverteNode::distancia_callback, this, std::placeholders::_1));
    }

private:
    void mode_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        last_mode_ = msg;
        publish_state();
    }

    void armed_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        last_armed_ = msg;
        publish_state();
    }

    void distancia_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto range_msg = sensor_msgs::msg::Range();
        range_msg.range = msg->data;
        range_publisher_->publish(range_msg);
        RCLCPP_INFO(this->get_logger(), "Published Range: '%f'", range_msg.range);

    }

    void publish_state()
    {
        if (last_mode_ && last_armed_)
        {
            auto state_msg = drone_interfaces::msg::State();

            // std_msgs/Header header
            // bool connected
            // bool armed
            // bool guided
            // bool manual_input
            // string mode
            // uint8 system_status

            state_msg.mode = last_mode_->data;
            state_msg.armed = last_armed_->data;
            state_publisher_->publish(state_msg);

            last_mode_.reset();
            last_armed_.reset();
            RCLCPP_INFO(this->get_logger(), "Published mode: '%s', armed: '%s'", state_msg.mode.c_str(), state_msg.armed ? "true" : "false");
        }
    }


    rclcpp::Publisher<drone_interfaces::msg::State>::SharedPtr state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distancia_subscription_;

    std_msgs::msg::String::SharedPtr last_mode_;
    std_msgs::msg::Bool::SharedPtr last_armed_;
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConverteNode>());
    rclcpp::shutdown();
    return 0;
}