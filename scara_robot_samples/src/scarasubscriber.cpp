#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <scara_robot_msgs/Axes.h>
#include "geometry_msgs/Point.h"
#include <sstream>

#include "scara_lib.h"

HID_UART_DEVICE dev=0;

#define	TRANSITION_MSEC	(150)

void subscribe_jointstate(const scara_robot_msgs::Axes::ConstPtr& jointstatearray)
{

	short sPos[5];
	int num=5;

	sPos[0] = (short) (jointstatearray->axis1/M_PI *1800);
	sPos[1] = (short) (jointstatearray->axis2/M_PI *1800);
	sPos[2] = HEIGHT_TO_RAD((jointstatearray->axis3*1000));
	sPos[3] = (short) (jointstatearray->axis4/M_PI *1800);
	sPos[4] = WIDTH_TO_RAD((-jointstatearray->axis5*2000)+20,CROW_POS);
	if(dev!=0) RSMove(dev,sPos,TRANSITION_MSEC,1,num);
}



void subscribe_position(const geometry_msgs::Point::ConstPtr& position)
{
	short sPos[3];
	double x=position->x,y=position->y,z=position->z;
	ROS_INFO("x:[%f] , y:[%f] , z:[%f]",x,y,z);

	pos_to_rad(x,y,z,0,0,sPos,1,3);
	if(dev!=0) RSMove(dev,sPos,TRANSITION_MSEC,1,3);

}

void subscribe_power(const std_msgs::Bool::ConstPtr& poweron)
{
	short power;
	if(poweron->data==true){
		ROS_INFO("power ON");
		power=1;
	}
	else{
		ROS_INFO("power off");
		power=0;
	}
	if(dev!=0) RSTorqueOnOff(dev,power,1,5);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scara_controller");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	bool poweron=false;

	ros::Publisher potentiometer_pub = n.advertise<scara_robot_msgs::Axes>("/scara_controller/potentiometer", 500);
	ros::Publisher readposition_pub = n.advertise<geometry_msgs::Point>("/scara_controller/readposition", 500);

	ros::Subscriber subjoint = n.subscribe("/scara_controller/command", 10, subscribe_jointstate);
	ros::Subscriber subposition = n.subscribe("/scara_controller/position", 10, subscribe_position);
	ros::Subscriber subpower = n.subscribe("/scara_controller/poweron", 10, subscribe_power);

	scara_robot_msgs::Axes potentiometer;
	geometry_msgs::Point readposition;
	pn.getParam("poweron",poweron);

	unsigned int numDevice=0;
	HidUart_GetNumDevices(&numDevice,VID,PID);
	printf("%d device(s) found.\n",numDevice);

	if(numDevice>0){
		if(HidUart_Open(&dev,0,VID,PID)==HID_UART_SUCCESS){
			SetTXOpenDrain(dev);
			if(poweron) RSTorqueOnOff(dev,1,1,5);
		}
		else{
			printf("cannot open device\n");
			return 1;
		}
	}

	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		if(dev!=0){
			double x,y,z;					//現在のアームの座標を表す変数
			short pos[5]={0,0,0,0,0};		//現在位置を代入する配列変数

			for(int i=0;i<5;i++) RSGetAngle(dev,i+1,&pos[i]);
			//現在接続されたモータ数だけ現在位置を取得
			potentiometer.axis1 = (double)pos[0]/1800.0 * M_PI;
			potentiometer.axis2 = (double)pos[1]/1800.0 * M_PI;
			potentiometer.axis3 = RAD_TO_HEIGHT(pos[2]) / 1000.0;
			potentiometer.axis4 = (double)pos[3]/1800.0 * M_PI;
			potentiometer.axis5 = -(RAD_TO_WIDTH(pos[4],CROW_POS)-20)/2000;

			rad_to_pos(&x,&y,&z,0,0,pos,3);
			readposition.x = x;
			readposition.y = y;
			readposition.z = z;

			potentiometer_pub.publish(potentiometer);
			readposition_pub.publish(readposition);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	if(dev!=0){
		RSTorqueOnOff(dev,0,1,5);
		HidUart_Close(dev);
	}

	return 0;
}


