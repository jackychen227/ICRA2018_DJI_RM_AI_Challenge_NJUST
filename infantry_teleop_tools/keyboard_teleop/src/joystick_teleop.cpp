#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float64.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

using namespace std;
float max_linear_vel = 0.2;
float max_angular_vel = 1.5707;

/*joystick index mapping

a satands for axes.
b satands for buttons.

you can remapping the index according to your joystick.

	b_6						b_7	
	b_4						b_5	
								
	a_5			b_8			b_0	
a_4		a_4		b_9		b_1		b_1
	a_5			b_12		b_0	
								
		a_1				a_3		
	a_0		a_0		a_2		a_2	
		a_1				a_3		

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

class TeleopJoy{
	public:
		TeleopJoy();
	private:
		void callBack(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle n;
		ros::Publisher base_pub,cradleHead_yaw_pub,cradleHead_pitch_pub;
		ros::Subscriber sub;
		float speed, turn;
		float x, y, z, omega;
		float position_yaw, position_pitch;
		float fire_flag;
};

TeleopJoy::TeleopJoy(){   
	speed = 1.0;
	turn = 1.0; 
	x = 0.0;
	y = 0.0;
	z = 0.0;
	omega = 0.0;
	position_yaw = 0.0;
	position_pitch = 0.0;
	fire_flag = 0.0;

	n.param("speed",speed,speed);
	n.param("turn",turn,turn);
	n.param("position_yaw",position_yaw,position_yaw);
	n.param("position_pitch",position_pitch,position_pitch);
	n.param("fire_flag",fire_flag,fire_flag);
	
	base_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	cradleHead_yaw_pub = n.advertise<std_msgs::Float64>("yaw_position_controller/command",1);
	cradleHead_pitch_pub = n.advertise<std_msgs::Float64>("pitch_position_controller/command",1);

	sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callBack, this);
}

void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy){
	geometry_msgs::Twist twist;
	
	twist.linear.x = speed*joy->axes[AXES_1];
	twist.linear.y = speed*joy->axes[AXES_0];
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = turn*joy->axes[AXES_2];

	//position_yaw += joy->axes[AXES_2];
	position_yaw = 0;
	position_pitch += joy->axes[AXES_3];
	if (joy->buttons[BUTTONS_5])
	{
		position_yaw = 0;
		position_pitch = 0;
	}
	if (position_yaw > 1.5 )	position_yaw = 1.5;
	else if(position_yaw < -1.5 )	position_yaw = -1.5;
	if (position_pitch > 1.5 )	position_pitch = 1.5;
	else if(position_pitch < -1.5 )	position_pitch = -1.5;

	fire_flag = joy->buttons[BUTTONS_7];

	base_pub.publish(twist);
	cradleHead_yaw_pub.publish(position_yaw);
	cradleHead_pitch_pub.publish(position_pitch);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "joystick_teleop");
	TeleopJoy teleop_infantry;
	ros::spin();
}

