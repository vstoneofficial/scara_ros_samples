#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <scara_robot_msgs/Axes.h>
#include "geometry_msgs/Point.h"
#include <sstream>

#include "scara_lib.h"

scara_robot_msgs::Axes pos;
geometry_msgs::Point geo;
int isupdate = 0;

void subscribe_jointstate(const sensor_msgs::JointState::ConstPtr& jointstate)
{
	ROS_INFO("recv id1:[%f] ,  id2:[%f] ,  id3:[%f] ,  id4:[%f] ,  id5:[%f]",
	jointstate->position[0],jointstate->position[1],jointstate->position[2],jointstate->position[3],jointstate->position[4]);

	pos.axis1 = jointstate->position[0];
	pos.axis2 = jointstate->position[1];
	pos.axis3 = jointstate->position[2];
	pos.axis4 = jointstate->position[3];
	pos.axis5 = jointstate->position[4];


	double x,y,z;
	float pos1 = (pos.axis1/3.1415926f)*1800.0f;
	float pos2 = (pos.axis2/3.1415926f)*1800.0f;
	float pos3 = pos.axis3;
	short pos[5]={(short)pos1,(short)pos2,0,0,0};

	rad_to_pos(&x,&y,&z,0,0,pos,3);
	geo.x = x;
	geo.y = y;
	geo.z = pos3;

	isupdate = 1;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_controller");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	ros::Publisher potentiometer_pub = n.advertise<scara_robot_msgs::Axes>("/scara_controller/potentiometer", 500);
	ros::Publisher readposition_pub = n.advertise<geometry_msgs::Point>("/scara_controller/readposition", 500);

	ros::Subscriber subposition = n.subscribe("/arm_robot/joint_states", 10, subscribe_jointstate);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		while(ros::ok() && isupdate==0){
			ros::spinOnce();
			loop_rate.sleep();
		}
		potentiometer_pub.publish(pos);
		readposition_pub.publish(geo);

		isupdate = 0;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


