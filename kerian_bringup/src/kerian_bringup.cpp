#include "kerian_bringup/kerian.h"

Robot::Kerian  kerian;

void callBack(const geometry_msgs::Twist& msg)
{
	kerian.SetExceptVec(msg.linear.x, msg.angular.z);
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"kerian_bringup");
	ros::NodeHandle n;
	
	
	
	if (!kerian.init())
		ROS_ERROR("Kerian initialized failed ");
	
	ROS_INFO("Kerain initialized successful");
	
	ros::Subscriber sub = n.subscribe("cmd_vel",50,callBack);
	
	ros::Rate loop_rate(50);
	
	while(ros::ok())
	{
		ros::spinOnce();
		
		kerian.spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
