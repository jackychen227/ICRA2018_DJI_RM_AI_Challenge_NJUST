/*************************************************************************
    > File Name: joystick_teleop.cpp
    > Author: Qingwu Chen
    > Mail: chenqingwu@njust.edu.cn 
    > Created Time: 2018年02月02日 星期五 21时15分17秒
    > Function Description:
 ************************************************************************/

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float64.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
#include<string>
using namespace std;

float max_linear_vel = 0.2;
float max_angular_vel = 1.5707;

/*joystick index mapping

a satands for axes.
b satands for buttons.

you can remapping the index according to your joystick.

	b_6			|			b_7	
	b_4			|			b_5	
-------------------------------------------------------------------								
	a_5		|	b_8	|		b_0	
a_4		a_4	|	b_9	|	b_1		b_1
	a_5		|	b_12	|	        b_0	
-------------------------------------------------------------------								
		a_1		|		a_3		
	a_0		a_0	|	a_2		a_2	
		a_1		|		a_3		

*/
#define	AXES_0	0
#define	AXES_1	1
#define	AXES_2	2
#define	AXES_3	3
#define	AXES_4	4
#define	AXES_5	5

#define	BUTTONS_0	0
#define	BUTTONS_1	1
#define	BUTTONS_2	2
#define	BUTTONS_3	3
#define	BUTTONS_4	4
#define	BUTTONS_5	5
#define	BUTTONS_6	6
#define	BUTTONS_7	7

string node_msgs = "DJI RoboMaster AI Challenge \nICRA 2018, Brisbane, Australia\nNanjing University of Science and Technology\nQingwu Chen\ninfantry_teleop_joystick";
class TeleopJoy{
	public:
		TeleopJoy();
	private:
		void callBack(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle n;
		ros::Publisher base_pub;
		ros::Publisher cradleHead_yaw_pub;
		ros::Publisher cradleHead_pitch_pub;
		ros::Subscriber sub;
		float speed, turn;
		float x, y, z, omega;
		//float fire_flag;
		geometry_msgs::Twist twist;
		std_msgs::Float64 position_yaw, position_pitch,fire_flag;
};

TeleopJoy::TeleopJoy()
{   
	speed = 2.5;
	turn = 6.28; 
	x = 0.0;
	y = 0.0;
	z = 0.0;
	omega = 0.0;

	n.param("speed",speed,speed);
	n.param("turn",turn,turn);

	
	base_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	cradleHead_yaw_pub = n.advertise<std_msgs::Float64>("yaw_position_controller/command",1);
	cradleHead_pitch_pub = n.advertise<std_msgs::Float64>("pitch_position_controller/command",1);

	sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack, this);
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
	
    
	twist.linear.x = -speed*joy->axes[AXES_1];
	twist.linear.y = -speed*joy->axes[AXES_0];
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = turn*joy->axes[AXES_2];

	//position_yaw.data += joy->axes[AXES_2];
	position_yaw.data = 0;
	position_pitch.data -= 0.5*joy->axes[AXES_3];
	if (joy->buttons[BUTTONS_5])
	{
		position_yaw.data = 0;
		position_pitch.data = 0;
	}
	if (position_yaw.data > 1.5 )	position_yaw.data = 1.5;
	else if(position_yaw.data < -1.5 )	position_yaw.data = -1.5;
	if (position_pitch.data > 1.5 )	position_pitch.data = 1.5;
	else if(position_pitch.data < -1.5 )	position_pitch.data = -1.5;

	fire_flag.data = joy->buttons[BUTTONS_7];

	//printf("x:%f\n",twist.linear.x);
	base_pub.publish(twist);
	cradleHead_yaw_pub.publish(position_yaw);
	cradleHead_pitch_pub.publish(position_pitch);
}

int main(int argc, char** argv){
	ros::init(argc, argv,"infantry_teleop_joystick");
	TeleopJoy teleop_infantry;
	cout<<node_msgs<<endl;
	ros::spin();
}
