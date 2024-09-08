#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vec_package/msg/vector.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("Vector_publisher")
    {
      vec.resize(3);
      vec.push_back(0);
      vec.push_back(1);
      vec.push_back(2);
      vec.resize(3);

      publisher_ = this->create_publisher<vec_package::msg::Vector>("Vector_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      vec[0]=++vec[0];
      vec[1]=++vec[0];
      vec[2]=++vec[1];
	vec.shrink_to_fit();
      std::string vecString = std::to_string(vec[0])+", "+std::to_string(vec[1])+", "+std::to_string(vec[2]);
      auto message = vec_package::msg::Vector();
      message.data = vec;
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: "+vecString+" size="<<message.data.size());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vec_package::msg::Vector>::SharedPtr publisher_;
    std::vector<int> vec;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
