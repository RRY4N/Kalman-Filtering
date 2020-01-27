#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

data = 0
state = 0

def publisher_node():	
	cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	twist = Twist()
	rospy.sleep(1.)
	global data
	global state

	desired = 320
	kp = 0.005
	ki = 0.00025
	kd = 0.015
	integral = 0
	lasterror = 0

	i = 0

	statearray = [.61,1.22,2.44,3.05]

	start = rospy.Time.now().to_sec()
	end = start
	while (end - start) < 40:
		end = rospy.Time.now().to_sec()
		if state > (statearray[i]- .01) and state < (statearray[i] + .01):
			twist.linear.x = 0
			twist.angular.z = 0
			cmd_pub.publish(twist)
			i = i+1
			rospy.sleep(2.)
 		else:
			actual = data
			error = desired - actual
			integral = integral + error
			derivative = error - lasterror
			correction = kp*error + ki*integral + kd*derivative
		
			twist.linear.x = 0.1
			if abs(correction) > 0.26:
				twist.linear.x = 0.08
			twist.angular.z = correction
			cmd_pub.publish(twist)
			#print("error: ", error)
			lasterror = error
			rospy.sleep(0.05)
		

	twist.linear.x = 0
	twist.angular.z = 0
	cmd_pub.publish(twist)
	
	pass

def callback(data1):
	global data 
	data = int(data1.data)
	
def callback1(data2):
	global state
	state = float(data2.data)


def main():
    try:
	rospy.init_node('camera')
	color_subscriber = rospy.Subscriber('color_mono',String,callback,queue_size=1)
	state_subscriber = rospy.Subscriber('state',String,callback1,queue_size = 1)
	publisher_node()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    main()

