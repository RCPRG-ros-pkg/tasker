import rospy 
from multitasker.srv import *
import time
from std_msgs.msg import *
import multiprocessing
import threading

class SigHold(Exception):
    pass

class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop_event = threading.Event()

    def stop(self):
        raise SIGTERM
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []
        self.priority = 0
        self.start_deadline = -1
        srv_name = rospy.get_name()+'/multitasking/get_hold_conditions'
        self.s = rospy.Service(srv_name, HoldConditions, self.getHoldConditions)
        self.current_state = ''
        self.task_state = 0 # initialized_not_running
        self.current_state_thread = 0
        self.sub_hold = rospy.Subscriber("hold_now", String, self.onHold)
        self.sub_res = rospy.Subscriber("resume_now", String, self.onResume)
        self.q = multiprocessing.Queue()

    def onHold(self, param):
        print "HOLD"
        self.task_state = 2 # Holding
        self.q.put(self.task_state)
        self.current_state_thread.terminate()
        time.sleep(0.1)
        while True:
            if not self.current_state_thread.is_alive():
                print "[MAIN]: WORKER is a goner"
                self.current_state_thread.join(timeout=1.0)
                break # watchdog process daemon gets terminated
        self.task_state = 3 # Holded
        self.handlers[self.current_state.upper()][1]("blah")
        data = rospy.wait_for_message('/resume_now', Empty)  
        print data 

    def onResume(self, param):
        print "RESUME"
        self.task_state = 4 # Resuming
        self.q.put(self.task_state)
        time.sleep(0.1)
        while True:
            print "RESUME"  
            time.sleep(0.1)   

        # self.current_state_thread.join()
    def getHoldConditions(self, param):
        #raise SigHold()
        return HoldConditionsResponse(priority=self.priority, 
                                        time_to_hold=self.time_to_hold, 
                                        time_to_finish=self.time_to_finish)
        #self.handlers[self.current_state.upper()][1]()

    def add_state(self, name, handler, hold_handler = None, end_state=0):
        name = name.upper()
        self.handlers[name] = [handler, hold_handler]
        if end_state:
            self.endStates.append(name)

    def set_LaunchRules(self, priority = 0 , start_deadline = -1):
        name = name.upper()
        self.set_priority = priority
        self.set_startDeadline = start_deadline

    def set_start(self, name):
        self.startState = name.upper()

    def set_priority(self, value):
        self.priority = value

    def set_time_to_hold(self, value):
        self.time_to_hold = value

    def set_time_to_finish(self, value):
        self.time_to_finish = value

    def update_multitasker(self, priority, time_to_hold, time_to_finish):
        self.time_to_hold = time_to_hold
        self.time_to_finish = time_to_finish
        self.priority = priority

    def set_startDeadline(self, value):
        self.start_deadline = value
    def run_state_machine(self, cargo, q):
        try:
            while True:
                print "1"
                (newState, cargo) = self.handlers[self.current_state.upper()][0](cargo)
                print "2"
                self.current_state = newState
                print "3" 
                if newState.upper() in self.endStates:
                    print("reached ", newState)
                    break 
                print "4" 
                # x =q.get()
                print "5" 
                # print x 
                print "3" 
                if self.task_state == 2:
                    break
                else:
                    handler = self.handlers[newState.upper()][0]
                time.sleep(3)
                if self.task_state == 2:
                    break
                print "HELLO" 
        finally:
            print "HELLO"

    def run(self, cargo):
        try:
            self.q = multiprocessing.Queue()
            self.current_state = self.startState
            handler = self.handlers[self.startState.upper()][0]
            self.task_state = 1 # Running
        except:
            raise InitializationError("must call .set_start() before .run()")
        if not self.endStates:
            raise  InitializationError("at least one state must be an end_state")
        self.current_state_thread = multiprocessing.Process(target = self.run_state_machine, args = (cargo, self.q))

        self.current_state_thread.start()
        while not rospy.is_shutdown():
            print self.current_state_thread.is_alive()
            time.sleep(0.1)



