import task_machine.StateMachine
import rospy
import time 
import actionlib

STATE_INTERRUPTED = 0
STATE_IN_PROGRESS = 1

# =======================================================================================

'''
Temporary function to simulate some tiago actions but the loop is time-based
'''
def handle_tiago_fake_action(time_stop):

	i = 0
	while ( i < time_stop ):
		
		print( "\t o  HOLD IS UNAVAILABLE"+"\n")
		i += 1
		time.sleep(1)

	return

# =======================================================================================

def handle_tiago_action(action_client, event_in, event_out):

	while ( action_client.get_state() != ACTION_STATUS_SUCCEEDED ):
		
		print( "\t o  action STATE: " + str(action_client.get_state()) +"\n")
		
		if ( action_client.get_state() == ACTION_STATUS_ABORTED ):
			print( "\t o  action STATE: ABORTED - a valid plan could not be found... \n")
			break
		
		if ( handle_interrupt(event_in, event_out) ):
			return STATE_INTERRUPTED
		
		time.sleep(1)

	print( "\t o  action STATE: " + str(action_client.get_state()) +"\n")
	return STATE_IN_PROGRESS

# =======================================================================================

def handle_interrupt(event_in, event_out):

	if event_in.isSet():
        event_out.set()
        return 1
    else:
    	return 0

# =======================================================================================