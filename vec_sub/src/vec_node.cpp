#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vec_package/msg/vector.hpp"
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("Vector_subscriber")
    {
      vec.resize(3);
      vec.push_back(0);
      vec.push_back(1);
      vec.push_back(2);

      subscription_ = this->create_subscription<vec_package::msg::Vector>(
      "Vector_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const vec_pac
    kage::msg::Vector & msg) const
    {
      //vec = msg.data;
      //RCLCPP_INFO(this->get_logger(), "Receiving: '%d', '%d', '%d'", vec[0], vec[1], vec[2]);
      //RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.data[0] <<", "<<msg.data[1] <<", "<<msg.data[2] << "'");
      
      if(msg.data.size()>0){ 
	std::string vecString;
	
	for(int i=0; i<(int)msg.data.size()-1; i++){
	  vecString += std::to_string(msg.data[i]) +", ";
	}
	vecString += std::to_string(msg.data.back());

	RCLCPP_INFO_STREAM(this->get_logger(), "Received "+std::to_string(msg.data.size())+" members: "+vecString);
      }else{
        RCLCPP_WARN_STREAM(this->get_logger(), "Received empty std::vector<int>");
      }
      
      //RCLCPP_INFO_STREAM(this->get_logger(), "Received info");
    }
    rclcpp::Subscription<vec_package::msg::Vector>::SharedPtr subscription_;
    std::vector<int> vec;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
