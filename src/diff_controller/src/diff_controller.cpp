
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"


class DiffController
{
private:
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub;
	ros::Publisher rwheel_motor_pub;
	ros::Publisher lwheel_motor_pub;

	geometry_msgs::Twist twist;
	std_msgs::Float32 rmotor_cmd, lmotor_cmd;

	float L, R;
	float vr, vl;
	float motor_cmd_max, motor_cmd_min;
	float motor_max_vel, motor_min_vel;

public:
	DiffController();
	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
	float vel_to_motor(float vel);

};

DiffController::DiffController(){
	n.param<float>("wheel_distance", L, 0.13);
	n.param<float>("wheel_radius", R, 0.03);
	n.param<float>("motor_cmd_max", motor_cmd_max, 1000);
	n.param<float>("motor_cmd_min", motor_cmd_min, 0);
	n.param<float>("motor_max_vel", motor_max_vel, 1.00); 
	n.param<float>("motor_min_vel", motor_min_vel, 0.00);

	cmd_vel_sub = n.subscribe("cmd_vel", 1, &DiffController::cmd_vel_callback, this);
	rwheel_motor_pub = n.advertise<std_msgs::Float32>("rwheel_angular_vel_motor", 10);
	lwheel_motor_pub = n.advertise<std_msgs::Float32>("lwheel_angular_vel_motor", 10);
}

void DiffController::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){

	twist = *msg;
	vr = (2 * twist.linear.x + twist.angular.z * L) / 2;
	vl = (2 * twist.linear.x - twist.angular.z * L) / 2;
	rmotor_cmd.data = vel_to_motor(vr);
	lmotor_cmd.data = vel_to_motor(vl);

	rwheel_motor_pub.publish(rmotor_cmd);
	lwheel_motor_pub.publish(lmotor_cmd);

	ROS_INFO("rmotor_cmd = %f", rmotor_cmd.data);
	ROS_INFO("lmotor_cmd = %f", lmotor_cmd.data);
}

float DiffController::vel_to_motor(float vel){
	float motor_cmd;
	float ratio;

	ratio = (motor_cmd_max - motor_cmd_min) / (motor_max_vel - motor_min_vel);
	motor_cmd = vel * ratio;
	if (motor_cmd > 0)
		if (motor_cmd > motor_cmd_max) motor_cmd = motor_cmd_max;

	if (motor_cmd < 0)
		if (abs(motor_cmd) > motor_cmd_max) motor_cmd = -motor_cmd_max;


	return motor_cmd;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "diff_controller");

	DiffController dc;

	ros::spin();

	return 0;
}
