import rospy 
from multitasker.srv import *
import time
from std_msgs.msg import *
import multiprocessing
import threading

stop_trigger = False

class SigHoldFSM(Exception):
    pass
class SigHoldSTATE(Exception):
    pass

class FSMThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, target, cargo_in = None, event = None):
        # threading.Thread.__init__(self)
        super(FSMThread, self).__init__()
        self._stop_event = event
        self.cargo_in = cargo_in
        self.target = target
        self._return = None

    def run(self):
        # cargo = self.cargo
        # event = self._stop_event
        print "thread event: ", self._stop_event
        self._return = self.target(cargo_in = self.cargo_in, event = self._stop_event)
            # print self._stop_event.is_set()
            # time.sleep(0.2)
    def stop(self):
        # raise SIGTERM
        print "thread STOPPPING"
        self._stop_event.set()
    def stopped(self):
        return self._stop_event.is_set()
    
    def join(self):
        threading.Thread.join(self)
        return self._return
class PTFProcess(multiprocessing.Process):
    def __init__(self, group=None, target=None,
                 args=(), kwargs={}, Verbose=None):
        print "args: ", args
        print "kwargs: ", kwargs
        kwargs = ['ret', 'args']
        multiprocessing.Process.__init__(self, group, target, args, kwargs, Verbose)
        self._return = None

    def run(self):
        if self._Process__target is not None:
            self._return = self._Process__target(*self._Process__args,
                                                **self._Process__kwargs)  

    def join(self):
        multiprocessing.Process.join(self)
        return self._return
# class PTFThread(threading.Thread):
#     def __init__(self, group=None, target=None, name=None,
#                  args=(), kwargs={}, Verbose=None, stop_event = event):
#         threading.Thread.__init__(self, group, target, name, args, kwargs, Verbose)
#         self._return = None
#         self._stop_event = event

#     def run(self):
#         if self._Thread__target is not None:
#             self._return = self._Thread__target(*self._Thread__args,
#                                                 **self._Thread__kwargs)
#     def stop(self):
#         self._stop_event.set()
#     def stopped(self):
#         return self._stop_event.is_set()
#     def join(self):
#         threading.Thread.join(self)
#         return self._return

def run_blocking(handler, arguments = None, state_event = None):
    pqueue = multiprocessing.Queue() 
    pqueue.put(arguments)
    p = multiprocessing.Process(target=handler, args=(pqueue, ))
    # new_thread = PTFThread(target = handler, args = (arguments,))
    p.daemon = True
    p.start()
    print "new thread is alive: ", p.is_alive() 
    while (p.is_alive()):
        time.sleep(0.1)
        if state_event.is_set():
            # p.terminate()
            print "terminate"
        print "waiting to kill"
    ret = pqueue.get()
    p.join()
    # return_dict.values()
    print "after kill"
    print "VALUES: ", ret
    return ret

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
        self.current_state_m_thread = 0
        self.sub_hold = rospy.Subscriber("hold_now", String, self.onHold)
        self.sub_res = rospy.Subscriber("resume_now", String, self.onResume)
        self.q = multiprocessing.Queue()
        self.fsm_stop_event = threading.Event()
        self.resumeState = None
        self.resumeData = None

    def onHold(self, param):
        print "HOLD: ", param
        self.task_state = 2 # Holding
        # self.q.put(self.task_state)
        self.fsm_stop_event.set()
        time.sleep(0.1)
        while True:
            if self.current_state_m_thread.is_alive():
                print "onHold waiting for fsm thread"
                # self.current_state_m_thread.join(timeout=1.0)
                time.sleep(0.1)
                break # watchdog process daemon gets terminated
        self.task_state = 3 # Holded
        # print "onHold waiting for resume"
        # # self.handlers[self.current_state.upper()][1]("blah")
        # data = rospy.wait_for_message('/resume_now', Empty)  
        # print "onHold GOT resume"
        # print data 
        return

    def onResume(self, param):
        print "RESUME"
        self.task_state = 4 # Resuming

        time.sleep(0.1)  

        # self.current_state_m_thread.join()
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
    def run_state_machine(self, cargo_in, event):
        state_stop_event = threading.Event()
        holdData = None
        newState = "next"
        cargo_out = None
        print "END STATES: ", self.endStates
        try:
            while True:
                print "ev1: ", event.isSet()
                print "RUNNING STATE: ", self.current_state.upper()
                thread = FSMThread(target = self.handlers[self.current_state.upper()][0], cargo_in = cargo_in, event = state_stop_event)
                thread.start()
                while True:
                    if event.isSet():
                        state_stop_event.set()
                        while thread.is_alive():
                            print "FSM waits for state to terminate"
                            time.sleep(1)
                        (newState, cargo_out) = thread.join()
                        print "FSM launched newState to hold task"
                        (self.resumeState, self.resumeData) = self.handlers[newState.upper()][0](cargo_in=cargo_out, event = None)
                        print "FSM FINISHED newState as a hold state"
                        break

                    if (not event.isSet() and not thread.is_alive()):
                        (newState, cargo_out) = thread.join()
                        self.current_state = newState
                        cargo_in = cargo_out
                        print "GOT:"
                        print "newState: ", newState
                        print "cargo_out: ", cargo_out
                        break
                print "ev2: ", event.isSet()
                
                if event.isSet():
                    print "FSM finished"
                    break
                if self.current_state.upper() in self.endStates:
                    print "FSM reached end state: ", self.current_state
                    self.task_state = 5
                    break
        finally:
            print "FSM bye"
            state_stop_event.set()
            while thread.is_alive():
                thread.join()

    def run(self, cargo):
        try:
            self.q = multiprocessing.Queue()
            self.current_state = self.startState
            print "start: ", self.current_state
            # handler = self.handlers[self.startState.upper()][0]
            self.task_state = 1 # Running
        
        # self.current_state_m_thread = multiprocessing.Process(target = self.run_state_machine, args = (cargo, self.q))
            self.current_state_m_thread = FSMThread(target = self.run_state_machine, cargo_in = cargo, event = self.fsm_stop_event)
            self.current_state_m_thread.start()
            while not rospy.is_shutdown():
                if self.task_state == 3:
                    print "FSM killed, joining thread"
                    self.current_state_m_thread.join()
                    self.fsm_stop_event.clear()
                if self.task_state == 4:
                    print "Resuming fsm"
                    self.current_state = self.resumeState
                    cargo = self.resumeData
                    self.fsm_stop_event.clear()
                    self.current_state_m_thread = FSMThread(target = self.run_state_machine, cargo_in = self.resumeData, event = self.fsm_stop_event)
                    self.current_state_m_thread.start()
                    self.task_state = 1    
                if self.task_state == 5:
                    print "FSM finished, terminating whole task" 
                    while self.current_state_m_thread.is_alive():
                        print "trying to kill FSM"
                    self.current_state_m_thread.join()
                    self.fsm_stop_event.clear()
                    break          
                time.sleep(1)
                print "test"
        finally:
                self.fsm_stop_event.set()
                while self.current_state_m_thread.is_alive():
                    print "trying to kill FSM"
                    self.current_state_m_thread.join()                # raise InitializationError("must call .set_start() before .run()")
                # if not self.endStates:
                #     raise  InitializationError("at least one state must be an end_state")


