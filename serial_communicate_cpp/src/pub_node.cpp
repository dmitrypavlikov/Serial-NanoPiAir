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

struct termios tty;		//serial configuration struct
char read_buffer[20];		//char buffer for serial reading bytes
std::stringstream buf;		//string <--> int convertor thing
std::string str;		//string buffer for convertor thing
int num = 0;			//int buffer for convertor things

int serial_port = open("/dev/ttyUSB0", O_RDWR);		//open serial port

int16_t lw_spd = 0; 		//left wheel speed
int16_t rw_spd = 0;		//right wheel speed
int16_t body_angle = 0;		//body angle servo
int16_t hand_angle = 0;		//hand angle servo

void launch_serial(){		//serial configuration things 
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
	memset(&read_buffer, 'a', sizeof(read_buffer));		//pulling empty space in read_buffer with char 'a'
}

void recieveMail(){		//function for recieve mail from DXL_controller
	int counter = 0;	//counter for string splitting
	
	//Receive serial mail from DXL_controller (+XXXX,+XXXX,XXX,XXX.)
	int num_bytes = read(serial_port, &read_buffer, sizeof(read_buffer));
	//Convert mail to int variables
	for (int i = 0; i < sizeof(read_buffer); i++){
		if((read_buffer[i] != ',') and (read_buffer[i] != '.') and (read_buffer[i] != 'a')){
			buf << read_buffer[i];}
		else {
			buf >> num;
			switch(counter){
				case 0:
					lw_spd = num;
					break;
				case 1:
					rw_spd = num;
					break;
				case 2:
					body_angle = num;
					break;
			}
			counter++;
			str = "";
			}
		}
	hand_angle = num;		//pulling last variable
}

class LeoPublisher : public rclcpp::Node	//cpp ros2 pub_node
{
	public:
		LeoPublisher()
		: Node("from_leonardo")
		{
			publisher_ = this->create_publisher<custom_msgs::msg::Custom>("from_leonardo", 10); 
			timer_ = this->create_wall_timer(
			500ms, std::bind(&LeoPublisher::timer_callback, this));
		}
		
	private:
		void timer_callback()
		{
			recieveMail();
			auto message = custom_msgs::msg::Custom();
			message.lw_spd = lw_spd;
			message.rw_spd = rw_spd;
			message.body_angle = body_angle;
			message.hand_angle = hand_angle;
			RCLCPP_INFO(this->get_logger(),"Publishing messages");
			publisher_->publish(message);
		}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<custom_msgs::msg::Custom>::SharedPtr publisher_;
};




int main(int argc, char * argv[])
{	
	launch_serial();  // launch serial communication
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LeoPublisher>());
	rclcpp::shutdown();
 	return 0;
}

