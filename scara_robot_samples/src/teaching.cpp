#include "ros/ros.h"
#include <scara_robot_msgs/Axes.h>
#include "sensor_msgs/JointState.h"
#include <sstream>

sensor_msgs::JointState js0;
int isupdate = 0;

void potentiometer_subscribe(const scara_robot_msgs::Axes::ConstPtr& potentiometer)
{

	js0.header.stamp = ros::Time::now();
	int num=5;

	js0.name.resize(num);
	js0.position.resize(num);

	js0.position[0]=potentiometer->axis1;
	js0.position[1]=potentiometer->axis2;
	js0.position[2]=potentiometer->axis3;
	js0.position[3]=potentiometer->axis4;
	js0.position[4]=potentiometer->axis5;
	js0.name[0] = "arm1_joint";
	js0.name[1] = "arm2_joint";
	js0.name[2] = "arm3_joint";
	js0.name[3] = "arm4_joint";
	js0.name[4] = "arm5r_joint";

	isupdate = 1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_position_teaching");

	ros::NodeHandle n;

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
	ros::Subscriber sub = n.subscribe("/scara_controller/potentiometer", 100, potentiometer_subscribe);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		while(ros::ok() && isupdate==0){
			ros::spinOnce();
			loop_rate.sleep();
		}
		joint_pub.publish(js0);
		isupdate = 0;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


