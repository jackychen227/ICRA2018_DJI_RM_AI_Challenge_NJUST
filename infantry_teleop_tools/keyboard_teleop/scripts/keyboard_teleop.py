#!/usr/bin/env python
import roslib; roslib.load_manifest('keyboard_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty

msg = """
DJI RoboMaster AI Challenge 
ICRA 2018, Brisbane, Australia
Nanjing University of Science and Technology
Qingwu Chen
referee: https://github.com/ros-teleop

Keyboard Teleop
---------------------------
Reading from the keyboard  and Publishing to Twist and joint controllers!
Infantry base controller and cradlehead controller
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c
---------------------------
For Holonomic mode (strafing), hold down the shift key:
   Q    W    E                   U    I
   A    S    D                   J    K    L
   Z    X    C
               SPACE : FIRE
T : up (+z)
B : down (-z)
---------------------------
anything else : stop

r/v : increase/decrease max speeds by 10%
t/b : increase/decrease only linear speed by 10%
y/n : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
		'w':(1,0,0,0),
		'x':(1,0,0,-1),
		'a':(0,0,0,1),
		'd':(0,0,0,-1),
		'q':(1,0,0,1),
		'x':(-1,0,0,0),
		'c':(-1,0,0,1),
		'z':(-1,0,0,-1),
		'E':(1,-1,0,0),
		'W':(1,0,0,0),
		'A':(0,1,0,0),
		'D':(0,-1,0,0),
		'Q':(1,1,0,0),
		'X':(-1,0,0,0),
		'C':(-1,-1,0,0),
		'Z':(-1,1,0,0),
		'T':(0,0,1,0),
		'B':(0,0,-1,0),
	       }

speedBindings={
		'r':(1.1,1.1),
		'v':(.9,.9),
		't':(1.1,1),
		'b':(.9,1),
		'y':(1,1.1),
		'n':(1,.9),
	      }

cradleHeadBindings={
		'J':(1,0,0),
		'L':(-1,0,0),
		'I':(0,1,0),
		'K':(0,-1,0),
                '\x20':(0,0,1),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	base_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	cradleHead_yaw_pub = rospy.Publisher('yaw_position_controller/command', Float64, queue_size = 1)
	cradleHead_pitch_pub = rospy.Publisher('pitch_position_controller/command', Float64, queue_size = 1)
	rospy.init_node('infantry_teleop_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	cradleHead_angle_delta = rospy.get_param("~cradleHead_angle_delta", 0.08)
	position_yaw = rospy.get_param("~position_yaw", 0.00)
	position_pitch = rospy.get_param("~position_pitch", 0.00)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0
	fire_flag = 0
	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in cradleHeadBindings.keys():
				position_yaw = position_yaw + cradleHead_angle_delta*cradleHeadBindings[key][0]
				position_pitch = position_pitch + cradleHead_angle_delta*cradleHeadBindings[key][1]
				if (position_yaw > 1.5 ):
					position_yaw = 1.5
				elif(position_yaw < -1.5 ):
					position_yaw = -1.5
				if (position_pitch > 1.5 ):
					position_pitch = 1.5
				elif(position_pitch < -1.5 ):
					position_pitch = -1.5
				fire_flag = cradleHeadBindings[key][2]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				fire_flag = 0
				if (key == "U"):
					position_yaw = 0
					position_pitch = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			base_pub.publish(twist)
			cradleHead_yaw_pub.publish(position_yaw)
			cradleHead_pitch_pub.publish(position_pitch)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		position_yaw = 0
		position_pitch = 0		
		base_pub.publish(twist)
		cradleHead_yaw_pub.publish(position_yaw)
		cradleHead_pitch_pub.publish(position_pitch)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


