/*************************************************************************
    > File Name: teleop_joy.cpp
    > Author: Qingwu Chen
    > Mail: chenqingwu@njust.edu.cn 
    > Created Time: 2017年12月25日 星期一 13时37分23秒
    > Function Description:
 ************************************************************************/

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

using namespace std;
float max_linear_vel = 2;
float max_angular_vel = 4.71239;

class TeleopJoy{
		public:
				TeleopJoy();
		private:
				void callback(const sensor_msgs::Joy::ConstPtr& joy);
				ros::NodeHandle n;
				ros::Publisher pub;
				ros::Subscriber sub;
				int velLinear, velAngular;
};

TeleopJoy::TeleopJoy()
{
		velLinear = 1;
		velAngular = 0;
		n.param("axis_linear",velLinear,velLinear);
		n.param("axis_angualr",velAngular,velAngular);
		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
		sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::callback, this);


}

void TeleopJoy::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
		geometry_msgs::Twist vel;
		vel.angular.z = max_angular_vel*joy->axes[2];
		vel.linear.x = -max_linear_vel*joy->axes[1];
                vel.linear.y = -max_linear_vel*joy->axes[0];
		pub.publish(vel);
}

int main(int argc, char** argv){
  ros::init(argc, argv,"teleop_joy");
  TeleopJoy teleop_infantry;
  ros::spin();
}
