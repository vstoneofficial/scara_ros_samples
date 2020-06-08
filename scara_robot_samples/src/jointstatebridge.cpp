#include "ros/ros.h"
#include <scara_robot_msgs/Axes.h>
#include "sensor_msgs/JointState.h"
#include <sstream>

scara_robot_msgs::Axes pos;
int isupdate = 0;

void thru_jointstate(const sensor_msgs::JointState::ConstPtr& jointstate)
{
	ROS_INFO("recv id1:[%f] ,  id2:[%f] ,  id3:[%f] ,  id4:[%f] ,  id5:[%f]",
	jointstate->position[0],jointstate->position[1],jointstate->position[2],jointstate->position[3],jointstate->position[4]);

	pos.axis1 = jointstate->position[0];
	pos.axis2 = jointstate->position[1];
	pos.axis3 = jointstate->position[2];
	pos.axis4 = jointstate->position[3];
	pos.axis5 = jointstate->position[4];

	isupdate = 1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_joint_state_bridge");

	ros::NodeHandle n;

	ros::Publisher joint_pub = n.advertise<scara_robot_msgs::Axes>("/scara_controller/command", 100);

	ros::Subscriber sub = n.subscribe("/joint_states", 10, thru_jointstate);

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		while(ros::ok() && isupdate==0){
			ros::spinOnce();
			loop_rate.sleep();
		}
		joint_pub.publish(pos);

		isupdate = 0;

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


