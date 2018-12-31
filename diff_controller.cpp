
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"


class DiffController
{
private:
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub;

	geometry_msgs::Twist twist;

public:
	DiffController();
	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
};

DiffController::DiffController(){

	cmd_vel_sub = n.subscribe("cmd_vel", 1, &DiffController::cmd_vel_callback, this);
}

void DiffController::cmd_vel_callback(const std_msgs::Twist::ConstPtr& msg){

	twist = *msg;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "diff_controller");

	DiffController dc;

	ros::spin();

	return 0;
}
