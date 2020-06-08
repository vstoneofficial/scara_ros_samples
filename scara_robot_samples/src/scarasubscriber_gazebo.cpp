#include "ros/ros.h"
#include <scara_robot_msgs/Axes.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <sstream>

#include "scara_lib.h"

int isupdate = 0;
trajectory_msgs::JointTrajectory tpos;

void subscribe_jointstate(const scara_robot_msgs::Axes::ConstPtr& jointstatearray)
{
	tpos.points[0].positions[0]=jointstatearray->axis1;
	tpos.points[0].positions[1]=jointstatearray->axis2;
	tpos.points[0].positions[2]=jointstatearray->axis3;
	tpos.points[0].positions[3]=jointstatearray->axis4;
	tpos.points[0].positions[4]=jointstatearray->axis5;

	isupdate = 1;
}


void subscribe_position(const geometry_msgs::Point::ConstPtr& position)
{
	short sPos[3];
	double x=position->x,y=position->y,z=position->z;
	ROS_INFO("x:[%f] , y:[%f] , z:[%f]",x,y,z);

	pos_to_rad(x,y,z,0,0,sPos,1,3);

	tpos.points[0].positions[0]=(float) sPos[0]/1800.0f*3.1415926f;
	tpos.points[0].positions[1]=(float) sPos[1]/1800.0f*3.1415926f;
	tpos.points[0].positions[2]=z;

	isupdate = 1;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_joint_state_subscriber_gazebo");

	ros::NodeHandle n;

	ros::Publisher joint_pub = n.advertise<trajectory_msgs::JointTrajectory>("/arm_robot/arm_robot_trajectory_controller/command", 100);
	tpos.joint_names.resize(5);
	tpos.points.resize(1);
	tpos.points[0].positions.resize(5);

	tpos.joint_names[0]="arm1_joint";
	tpos.joint_names[1]="arm2_joint";
	tpos.joint_names[2]="arm3_joint";
	tpos.joint_names[3]="arm4_joint";
	tpos.joint_names[4]="arm5r_joint";

	tpos.points[0].positions[0]=0;
	tpos.points[0].positions[1]=0;
	tpos.points[0].positions[2]=0;
	tpos.points[0].positions[3]=0;
	tpos.points[0].positions[4]=0;


	ros::Subscriber sub = n.subscribe("/scara_controller/command", 10, subscribe_jointstate);
	ros::Subscriber subposition = n.subscribe("/scara_controller/position", 10, subscribe_position);


	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		while(ros::ok() && isupdate==0){
			ros::spinOnce();
			loop_rate.sleep();
		}

		tpos.points[0].time_from_start = ros::Duration(1);
		joint_pub.publish(tpos);

		isupdate = 0;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


