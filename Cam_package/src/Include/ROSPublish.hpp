#ifndef ROS2PUB123
#define ROS2PUB123
#include <string>
//#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "vec_package/msg/vector.hpp"

class VectorPublisher : public rclcpp::Node
{
  public:
//Constructors
    VectorPublisher(const string& nodeName, const string& topicName)
    : Node(nodeName)
    {
      publisher_ = this->create_publisher<vec_package::msg::Vector>(topicName, 10);
    }
    VectorPublisher(): VectorPublisher("Vector_publisher", "Vector_topic") {}
//End of Constructors
    void PublishINTVector(const std::vector<int>& msg, bool log=true){
      auto message = vec_package::msg::Vector();
      message.data = msg;
      
      if(message.data.size()>0){ 
        if(log){ //format the data so it can be logged
            string vecString;
            for(int i=0; i<message.data.size()-1; i++){
                vecString += std::to_string(message.data[i]) +", ";
            }
            vecString += std::to_string(message.data.back());

            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: "+vecString);
        } 
      }else{
        RCLCPP_WARN_STREAM(this->get_logger(), "Trying to send empty std::vector<int>");
      }
      
      publisher_->publish(message);
    }
    //Publishes the vector offset by a number
    void PublishOffset(const std::vector<int>& msg, const int refPoint, bool log=true){
      std::vector<int> offmsg; offmsg.reserve(msg.size());
      for(int element:msg){
        offmsg.emplace_back(element-refPoint);
      }
      PublishINTVector(offmsg, log);
    }

  private:
    rclcpp::Publisher<vec_package::msg::Vector>::SharedPtr publisher_;
};
#endif
