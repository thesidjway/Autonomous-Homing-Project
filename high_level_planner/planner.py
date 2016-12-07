#planner.py
#Siddharth S Jha
import rospy
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Twist

MF=0.1
prevAngles = Point32(0,0,0)
cmdvel=Twist(0,0,0,0,0,0);

def publish()
	pub.publish(cmdvel)

def planner(data):
	beta13=data.x
	beta21=data.y
	beta32=data.z

	betastar13=prevAngles.x
	betastar21=prevAngles.y
	betastar32=prevAngles.z

    delta13=beta13-betastar13
    delta21=beta21-betastar21
    delta32=beta32-betastar32

    vp1=(delta13+delta21)*MF
    vp2=(delta21+delta32)*MF    
    vp3=(delta13+delta32)*MF

    cmdvel.linear.x=
    cmdvel.linear.y=
    cmdvel.linear.z=0

    cmdvel.angular.x=0
    cmdvel.angular.y=0
    cmdvel.angular.z=
    publish()
 
    

def listener();
    rospy.init_node('planner')
    rospy.Subscriber('angles', Point32, planner)
    rospy.spin()

if __name__ == '__main__':
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    listener()