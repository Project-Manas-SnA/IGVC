#!/usr/bin/env python
import rospy
from interfacing.msg import Velocity
from evdev import InputDevice, categorize, ecodes, events
import time
autonomous = True

gamepad = InputDevice('/dev/input/event17')

leftwheel = 0
rightwheel = 0

def Controller(pub, v, rate):

	global autonomous
	gamepad = InputDevice('/dev/input/event17')

	for event in gamepad.read_loop():

		if event.code == 04:

			if event.value == 589834:
				autonomous = True
				break

			elif event.value == 589833:
				autonomous = False
				v.leftwheel=v.rightwheel=0;
				break

		elif event.code == 17:

			if event.value == -1:
				v.leftwheel=v.rightwheel=24   # Forward
				v.Check = 0
				print (v)
				break

			if event.value == 1:
				v.leftwheel=v.rightwheel=-24
				v.Check = 0					# Backward
				print (v)
				break

		elif event.code == 16:

			if event.value == -1:
				v.leftwheel=-24
				v.rightwheel=24					#left
				v.Check = 0
				print (v)
				break

			if event.value == 1:
				v.leftwheel=24
				v.rightwheel=-24					#right
				v.Check = 0
				print (v)
				break

		elif event.value == 0:
				v.leftwheel = v.rightwheel = 0
				v.Check = 0
				print (v)
				break
		else:
			v.leftwheel=v.rightwheel = 0
			v.Check=0
			print(v)
			break


	pub.publish(v)
	rate.sleep()


def publishBack(pub, v, rate):

	print("autonomous")

	global leftwheel
	global rightwheel

	v.leftwheel = leftwheel
	v.rightwheel = rightwheel
	v.Check = 1

	pub.publish(v)
	rate.sleep()


def Autonomous(data):
	global leftwheel
	global rightwheel

	leftwheel=data.leftwheel
	rightwheel=data.rightwheel


def talker():

	global autonomous

	pub = rospy.Publisher('Velocity', Velocity, queue_size = 10)

	rospy.init_node('velocity')

	rate =rospy.Rate(10)

	v=Velocity()

	rospy.Subscriber("wheelVelocity", Velocity, Autonomous)

	while not rospy.is_shutdown():

		event = gamepad.read_one()

		if event:
			if event.type == ecodes.EV_MSC:

				if event.code == 04:

					if event.value == 589834:
						v.Check = 1
						autonomous = True

					elif event.value == 589833:
						v.Check=0
						v.leftwheel=v.rightwheel=0
						pub.publish(v)
						autonomous = False

		if autonomous == True:
			publishBack(pub, v, rate)
		elif autonomous == False:
			Controller(pub, v, rate)



if __name__ == "__main__":
	try :
		talker()
	except rospy.ROSInterruptException:
		pass
