#include <chrono>
#include <cstdio>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/custom.hpp"

using namespace std::chrono_literals;

/*class MailPublisher : public rclcpp::Node
{
  public:
    Publisher()
    : Node("mail_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<custom_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
	private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};*/

//Waiting for custom msg


void recieveMail(){
	//Receive UART mail (+XXXX,+XXXX,XXX,XXX.)
	// ...
	//Convert mail to int variables
	// ...
	;
}

void sendMail(){
	//Create mail package (+XXXX,+XXXX,XXX,XXX.)
	// ...
	//Send UART mail to DXL_controller
	// ...
	;
}

int main()
{


  printf("YASS");
  return 0;
}

