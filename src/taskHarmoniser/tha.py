#!/usr/bin/env python
import signal
import sys
from TaskHarmoniser import TaskHarmoniser
import time
import threading
import rospy 
import rosmsg
from multitasker.srv import *
from multitasker.msg import *
from rospy_message_converter import json_message_converter

_FINISH = False
scheduler = threading.Thread
switcher = threading.Thread
th = TaskHarmoniser


def updateSP(data):
	global th
	print("\nUPDATE SP\n")
	print "updateSP: ",data.da_id, "SP: \n", data.schedule_params
	th.updateScheduleParams(data.da_id, data.schedule_params)
	print("\nUPDATED SP\n")
	
def initDA(data):
	global th
	print("GOT REQUEST")
	new_id = th.getNextID()
	print("ID: ",new_id)
	json_str = json_message_converter.convert_ros_message_to_json(data.init_params)
	print("S: ",json_str)
	da_name = "DA_"+str(new_id)
	th.initialiseDA(data.application, data.version, new_id, da_name, json_str)
	# rospy.set_param('/'+da_name+'/init_params', stri)
	th.addDA(new_id, da_name, data.application)
	th.updateScheduleParams(new_id, data.init_params)
	return TaskRequestResponse(-1)

def scheduler():
	global _FINISH
	global th
	while True:
		print "\n SCHEDULING \n"
		th.schedule()
		print "\n SCHEDULED \n"
		time.sleep(1)
		if _FINISH:
			th.sendIndicator()
			break
def switcher():
	global th
	global _FINISH
	while True:
		print "\n SWITCHING \n"
		th.switchDA()
		if _FINISH:
			break
def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	global _FINISH
	_FINISH = True
	global scheduler
	global switcher 
	scheduler.join()   
	switcher.join() 
	sys.exit(0)

if __name__== "__main__":
	global th
	rospy.init_node('TH', anonymous=True)
	print("ready")
	signal.signal(signal.SIGINT, signal_handler)
	th = TaskHarmoniser()
	global scheduler
	global switcher
	scheduler = threading.Thread(target = scheduler)
	scheduler.start()
	switcher = threading.Thread(target = switcher)
	switcher.start()
	sub_status = rospy.Subscriber("TH/statuses", Status, updateSP)
	s = rospy.Service("TH/new_task", TaskRequest, initDA)

	rospy.spin()
  
	print("END")
    	