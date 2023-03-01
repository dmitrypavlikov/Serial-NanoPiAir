#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/custom.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

int16_t lw_spd = 0; 
int16_t rw_spd = 0;	
int16_t body_angle = 0;  
int16_t hand_angle = 0;  

class LeoSubscriber : public rclcpp::Node
{
  public:
    LeoSubscriber()
    : Node("to_leonardo")
    {
      subscription_ = this->create_subscription<custom_msgs::msg::Custom>(
      "to_leonardo", 10, std::bind(&LeoSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const custom_msgs::msg::Custom & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard something");//, msg.lw_spd.c_str());
      lw_spd = msg.lw_spd;
      rw_spd = msg.rw_spd;
      body_angle = msg.body_angle;
      hand_angle = msg.hand_angle;
    }
    rclcpp::Subscription<custom_msgs::msg::Custom>::SharedPtr subscription_;
};

void sendMail(){
	//Create mail package (+XXXX,+XXXX,XXX,XXX.)
	// ...
	//Send UART mail to DXL_controller
	// ...
	;
}

int main(int argc, char * argv[])
{	
  rclcpp::init(argc, argv);
  
  
  rclcpp::spin(std::make_shared<LeoSubscriber>());
  rclcpp::shutdown();

  
  return 0;
}


