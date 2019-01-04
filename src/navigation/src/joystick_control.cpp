
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <thread>

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
int get_axis_state(struct js_event *event, short axes[6])
{
	int axis = event->number;

	axes[axis] = event->value;

	return axis;
}

class JoystickControl
{
private:
	ros::NodeHandle n;
	ros::Publisher cmd_vel_pub;

	geometry_msgs::Twist twist;

	std::thread joy_thread;

	float scale_angular, scale_linear;
	int js;
	struct js_event event;
	short axes[6];
	bool publish;

public:
	JoystickControl();
	void read_joystick();
};

JoystickControl::JoystickControl(){
	n.param<float>("/joystick_control/scale_angular", scale_angular, 3.0);
	n.param<float>("/joystick_control/scale_linear", scale_linear, 0.5);

	js = open("/dev/input/js0", O_RDONLY);

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	joy_thread = std::thread(&JoystickControl::read_joystick, this);

	publish = false;
}

void JoystickControl::read_joystick(){
	int axis;
	/* This loop will exit if the controller is unplugged. */
	while (read_event(js, &event) == 0)
	{
		switch (event.type)
		{
			case JS_EVENT_BUTTON:
				if (event.number == 0 && event.value){
					if (publish){
						publish = false;
						twist.angular.z = 0;
						twist.linear.x = 0;
						cmd_vel_pub.publish(twist);
					}
					else
						publish = true;
				}
				//printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
				break;
			case JS_EVENT_AXIS:
				axis = get_axis_state(&event, axes);
				if (axis == 0){
					twist.angular.z = -(float)axes[0]/32767;
					twist.angular.z *= scale_angular;
				}
				if (axis == 2){
					twist.linear.x = -(float)(axes[2]+32767)/65535;
					twist.linear.x *= scale_linear;
				}
				if (axis == 5){
					twist.linear.x = (float)(axes[5]+32767)/65535;
					twist.linear.x *= scale_linear;
				}
				if (publish)
					cmd_vel_pub.publish(twist);
				break;
			default:
				/* Ignore init events. */
				break;
		}
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joystick_control");

	JoystickControl jc;

	ros::spin();

	return 0;
}
