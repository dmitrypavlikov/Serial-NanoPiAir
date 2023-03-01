#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <cstdio>
#include <cstdlib>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string.h>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/custom.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct termios tty;
char send_buffer[20];
std::stringstream buf;
std::string str;

int serial_port = open("/dev/ttyUSB0", O_RDWR);

int16_t lw_spd = 0; 
int16_t rw_spd = 0;	
int16_t body_angle = 0;  
int16_t hand_angle = 0;  

void launch_serial(){
	if (serial_port < 0) {
    	printf("Error %i from open: %s\n", errno, strerror(errno));}
	if(tcgetattr(serial_port, &tty) != 0) {
    	printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));}
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;
	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);
	//cfsetspeed(&tty, B9600); // alternative 
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
	printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));}
	memset(&send_buffer, 'a', sizeof(send_buffer));
}

void sendMail(){
	//Create mail package "buf" (+XXXX,+XXXX,XXX,XXX.)
	buf << lw_spd << ',' << rw_spd << ',' << body_angle << ',' << hand_angle << '.';
	buf >> str;
	for (int i = 0; i < sizeof(send_buffer); i++){
	send_buffer[i] = str[i];
	}
	//Send UART mail to DXL_controller
	write(serial_port, send_buffer, sizeof(send_buffer));
}


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
      sendMail();
    }
    rclcpp::Subscription<custom_msgs::msg::Custom>::SharedPtr subscription_;
};




int main(int argc, char * argv[])
{	
  launch_serial();  // launch serial communication
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeoSubscriber>());
  rclcpp::shutdown();
  return 0;
}


