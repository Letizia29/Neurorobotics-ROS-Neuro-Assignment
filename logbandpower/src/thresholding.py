#!/usr/bin/env python3
import rospy
import numpy as np
from rosneuro_msgs.msg import NeuroFrame, NeuroEvent
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def callback(data: NeuroFrame):
	global new_data, current_frame
	# Save the new data
	current_frame = data
	new_data = True

def apply_threshold(data: NeuroFrame):
	global ch, thr
	
	### ---------------- DATA ACQUISITION ---------------- ###
	
	# New data 
	eeg_data = np.array(data.eeg.data).reshape((data.eeg.info.nchannels, data.eeg.info.nsamples)) #all channels
	

	### ---------------- THRESHOLD APPLICATION ---------------- ###
	
	# If the threshold is reached:
	if (max(abs(eeg_data[ch,:])) > thr):
		
		# 1 if the threshold was reached
		return 1
		
	# 0 if it wasn't
	return 0


def generate_neuroevent():
	# Use the imported NeuroEvent
	new_msg = NeuroEvent()

	new_msg.header.stamp = rospy.Time.now()
	new_msg.header.frame_id = "thresholding_node"
	new_msg.type = "Custom"
	new_msg.value = 0 # irrelevant

	return new_msg


def main():
	global ch, thr
	new_data = False

	# Initialize the node
	rospy.init_node('thresholding_node')
	# Selected channel and threshold value as parameters of the node
	ch = rospy.get_param('channel', 9) # channel
	thr = rospy.get_param('threshold', 15) # threshold

	# Initialize the publisher 
	pub = rospy.Publisher('/events/bus', NeuroEvent, queue_size=1)
	

	# Setup the Subscriber
	rospy.Subscriber('eeg/bandpower', NeuroFrame, callback)
	

	while not rospy.is_shutdown():
		# Wait until new data arrives
		if new_data:

			# publish a NeuroEvent only if the threshold was reached	
			if apply_threshold(current_frame) == 1:
				# Publish in /events/bus
				event_msg = generate_neuroevent()
				pub.publish(event_msg)
	
			new_data = False
	
		#rate.sleep()
		
if __name__ == '__main__':
	main()
