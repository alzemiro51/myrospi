
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include  "geometry_msgs/Twist.h"

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <thread>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

#define IN1 	0x400 	//10
#define IN2 	0x200	//9
#define IN3 	0x800	//11
#define IN4 	0x100	//8

#define abs(x) ((x)<0 ? -(x) : (x))

//
// Set up a memory regions to access GPIO
//
void setup_io()
{
	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}

	/* mmap GPIO */
	gpio_map = mmap(
		NULL,             //Any adddress in our space will do
		BLOCK_SIZE,       //Map length
		PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
		MAP_SHARED,       //Shared with other processes
		mem_fd,           //File to map
		GPIO_BASE         //Offset to GPIO peripheral
	);

	close(mem_fd); //No need to keep mem_fd open after mmap

	if (gpio_map == MAP_FAILED) {
		printf("mmap error %d\n", (size_t)gpio_map);//errno also set!
		exit(-1);
	}

	// Always use volatile pointer!
	gpio = (volatile unsigned *)gpio_map;


} // setup_io

class MotorDriver
{
private:
	ros::NodeHandle n;
	ros::Subscriber rwheel_sub;
	ros::Subscriber lwheel_sub;
	int duty_r, duty_l;

	std::thread rwheel_motor_t;
	std::thread lwheel_motor_t;

public:
	MotorDriver();
	void set_motor_r();
	void set_motor_l();
	void rwheel_callback(const std_msgs::Float32::ConstPtr& msg);
	void lwheel_callback(const std_msgs::Float32::ConstPtr& msg);

};

MotorDriver::MotorDriver() {
	duty_r = 0;
	duty_l = 0;
	rwheel_sub = n.subscribe("rwheel_angular_vel_motor", 1, &MotorDriver::rwheel_callback, this);
	lwheel_sub = n.subscribe("lwheel_angular_vel_motor", 1, &MotorDriver::lwheel_callback, this);
	rwheel_motor_t = std::thread(&MotorDriver::set_motor_r, this);
	lwheel_motor_t = std::thread(&MotorDriver::set_motor_l, this);
}

void MotorDriver::set_motor_r(){
	int gpio_set;
	int gpio_clr;

	while(1){
		if(duty_r == 0)
			gpio_set = 0;
		else if(duty_r > 0)
			gpio_set = IN3;
		else if(duty_r < 0)
			gpio_set = IN4;

		GPIO_SET = gpio_set;
		usleep(abs(duty_r));
		GPIO_CLR = gpio_set;
		usleep(1000-abs(duty_r));
	}
}

void MotorDriver::set_motor_l(){
	int gpio_set;
	int gpio_clr;

	while(1){
		if(duty_r == 0)
			gpio_set = 0;
		else if(duty_l > 0)
			gpio_set = IN1;
		else if(duty_l < 0)
			gpio_set = IN2;

		GPIO_SET = gpio_set;
		usleep(abs(duty_l));
		GPIO_CLR = gpio_set;
		usleep(1000-abs(duty_l));
	}	
}

void MotorDriver::rwheel_callback(const std_msgs::Float32::ConstPtr& msg){

	duty_r = (int)msg->data;

}

void MotorDriver::lwheel_callback(const std_msgs::Float32::ConstPtr& msg){

	duty_l = (int)msg->data;
}

int main(int argc, char **argv)
{
	setup_io();
	// Set GPIO pins 7-11 to output
	for (int g=7; g<=11; g++) {
		INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
		OUT_GPIO(g);
	}
	GPIO_CLR = 0xf00;

	ros::init(argc, argv, "motor_driver");

	MotorDriver md;

	ros::spin();

	return 0;
}
