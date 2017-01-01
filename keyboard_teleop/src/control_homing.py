#!/usr/bin/env python

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone
  
# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# Finally the GUI libraries
from PySide import QtCore, QtGui




# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
statuspub = rospy.Publisher('homingStatus',Int32,queue_size=10)

class KeyMapping(object):
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	Emergency        = QtCore.Qt.Key.Key_Space
	StartReading	 = QtCore.Qt.Key.Key_Z
	StopReading	 	 = QtCore.Qt.Key.Key_X
	StartHoming 	 = QtCore.Qt.Key.Key_C
	StopHoming  	 = QtCore.Qt.Key.Key_V
	ReadManually 	 = QtCore.Qt.Key.Key_B
	StopManualRead 	 = QtCore.Qt.Key.Key_N


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):

	def __init__(self):
		super(KeyboardController,self).__init__()
		self.prevvel=-10
		self.x_vel=0.0
		self.y_vel=0.0
		self.bool_homing=0
		self.bool_reading=0
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0
		self.flyingStatus = 1
		self.sub1 = rospy.Subscriber('vels', Twist, self.velCallback)
		self.sub2 = rospy.Subscriber('flyingStatus', Int32, self.flyingCallback)


# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()
		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()

			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1
				if key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1



	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section

			if self.bool_reading==0:
				if key == KeyMapping.ReadManually:
					self.bool_reading=2	
			if self.bool_reading==2:
				if key == KeyMapping.StopManualRead:
					self.bool_reading=0
			if self.bool_homing == 0:
				if key == KeyMapping.StartHoming:
					self.bool_homing=1
			elif self.bool_homing == 1:
				if key == KeyMapping.StopHoming:
					self.bool_homing=0
			if self.bool_reading == 0:
				if key == KeyMapping.StartReading:
					self.bool_reading=1
			elif self.bool_reading == 1:
				if key == KeyMapping.StopReading:
					self.bool_reading=0

			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1


			if key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

	def flyingCallback(self, msg):
		self.flyingStatus=msg.data
		if self.flyingStatus != self.prevvel:
			self.yaw_velocity=0.0
		self.prevvel=self.yaw_velocity
			
			
	def velCallback(self, msg):
		self.x_vel=msg.linear.x
		self.y_vel=msg.linear.y

		if self.bool_homing==1:
			if (abs(self.x_vel)<0.001 and abs(self.y_vel)<0.001):
				if self.flyingStatus==3:
					if abs(self.yaw_velocity)==0:
						self.yaw_velocity=0.6


 
			self.roll=self.x_vel
			self.pitch=self.y_vel
		else:
			self.roll=0
			self.pitch=0


		if self.bool_reading==0:
			if self.bool_homing==0:
				statuspub.publish(0)
			elif self.bool_homing==1:
				statuspub.publish(2)
		elif self.bool_reading==1:
			statuspub.publish(1)
		elif self.bool_reading==2:
				statuspub.publish(3)


		controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('homing_controller')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()

	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)