#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


class ModbusDummy : public rclcpp::Node
{
public:
  ModbusDummy()
  : Node("modbus_dummy"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("modbus_dummy", 10);
    timer_ = this->create_wall_timer(
      20ms, std::bind(&ModbusDummy::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "modbus_dummy";
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModbusDummy>());
  rclcpp::shutdown();
  return 0;
}