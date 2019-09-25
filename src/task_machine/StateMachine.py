import rospy 
from multitasker.srv import *
from multitasker.msg import *
from std_srvs.srv import Trigger, TriggerResponse
import time
from std_msgs.msg import *
import multiprocessing
import threading
import datetime
stop_trigger = False

class SigHoldFSM(Exception):
    pass
class SigHoldSTATE(Exception):
    pass

class FSMThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, target, cargo_in = None, event_in = None, event_out = None):
        # threading.Thread.__init__(self)
        super(FSMThread, self).__init__()
        self._stop_event = event_in
        self._stopped_event = event_out
        self.cargo_in = cargo_in
        self.target = target
        self._return = None

    def run(self):
        # cargo = self.cargo
        # event = self._stop_event
        print("\n"+str(datetime.datetime.now().time())+"\n"+ "target: "+ str(self.target)+"\n")
        print("\n"+str(datetime.datetime.now().time())+"\n"+ "cargo_in: "+ str(self.cargo_in)+"\n")
        self._return = self.target(cargo_in = self.cargo_in, event_in = self._stop_event, event_out = self._stopped_event)
            # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ self._stop_event.is_set()+"\n")
            # time.sleep(0.2)
    def stop(self):
        # raise SIGTERM
        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "thread STOPPPING"+"\n")
        self._stop_event.set()
    def stopped(self):
        return self._stop_event.is_set()
    
    def join(self):
        threading.Thread.join(self)
        return self._return

class PTFProcess(multiprocessing.Process):
    def __init__(self, group=None, target=None,
                 args=(), kwargs={}, Verbose=None):
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "args: "+ str(args)+"\n")
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "kwargs: "+ str(kwargs)+"\n")
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
    ret = None
    pqueue = multiprocessing.Queue() 
    pqueue.put(arguments)
    p = multiprocessing.Process(target=handler, args=(pqueue, ))
    # new_thread = PTFThread(target = handler, args = (arguments,))
    p.daemon = True
    p.start()
    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "PID: "+ str( p.pid)+"\n")
    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "new thread is alive: "+ str(p.is_alive()) +"\n")
    while (p.is_alive()):
        time.sleep(0.1)
        if state_event.isSet():
            p.terminate()
            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "terminate"+"\n")
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "waiting for a blocking call to end or a hold signal"+"\n")
    if not state_event.isSet():
        ret = pqueue.get()
    p.join()
    # return_dict.values()
    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "after kill"+"\n")
    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "VALUES: "+ str(ret)+"\n")
    return ret

class StateMachine:
    def __init__(self, file, isInterrupting, da_ID, ptf_update_task, ptf_update_sp):
        self.startFlag = False
        self.isInterrupting = False
        self.da_ID = da_ID
        self.isInterrupting = isInterrupting
        self.ptf_update_task = ptf_update_task
        self.ptf_update_sp = ptf_update_sp
        self.print_log = file
        self.handlers = {}
        self.startState = None
        self.endStates = []
        self.facultativeStates = []
        self.priority = 0
        self.start_deadline = -1
        node_namespace = rospy.get_name() + "/multitasking"
        srv_name = rospy.get_name()+'/multitasking/get_hold_conditions'
        self.s = rospy.Service(srv_name, HoldConditions, self.getHoldConditions)
        self.current_state = ''
        self.task_state = 0 # initialized_not_running
        self.current_state_m_thread = 0
        self.res_srv = None
        self.suspend_srv = None
        # self.sub_hold = rospy.Subscriber(node_namespace+"/hold_now", String, self.onHold)
        # self.sub_res = rospy.Subscriber(node_namespace+"/resume_now", String, self.onResume)
        self.q = multiprocessing.Queue()
        self.fsm_stop_event = threading.Event()
        self.resumeState = None
        self.resumeData = None
        self.pub_state = rospy.Publisher('current_state', TaskState, queue_size=10)
        self.pub_status = rospy.Publisher('TH/statuses', Status, queue_size=10)
        self.time_to_hold = -1 
        self.time_to_finish = -1
        self.onResumeData = None
    
    def updateTask(self, TH_data):

        print "UPDATEING TASK - 2"
        return self.ptf_update_task(self, TH_data)

    def updateStatus(self):
        print "UPDATEING STATUS"
        my_status = Status()
        my_status.da_id = self.da_ID
        my_status.schedule_params = self.ptf_update_sp()
        print "UPDATEING STATUS - 2"
        self.pub_status.publish(my_status) 
        print "UPDATEING STATUS - 3"      

    def onHold(self, param):
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "HOLD: "+ str( param)+"\n")
        self.task_state = 2 # Holding
        # self.q.put(self.task_state)
        self.fsm_stop_event.set()
        time.sleep(0.1)
        while True:
            if self.current_state_m_thread.is_alive():
                self.print_log.write("onHold waiting for fsm thread"+"\n")
                # self.current_state_m_thread.join(timeout=1.0)
                time.sleep(0.1)
                break # watchdog process daemon gets terminated
        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "onHold waiting for resume"+"\n")
        # # self.handlers[self.current_state.upper()][1]("blah")
        # data = rospy.wait_for_message('/resume_now', Empty)  
        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "onHold GOT resume"+"\n")
        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ data +"\n")
        return TriggerResponse()

    def onResume(self, param):
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "RESUME"+"\n")
        self.task_state = 5 # Resuming
        self.onResumeData = param.params
        time.sleep(0.1)  
        return StartTaskResponse()
        # self.current_state_m_thread.join()
    def getHoldConditions(self, param):
        #raise SigHold()
        return HoldConditionsResponse(priority=self.priority, 
                                        time_to_hold=self.time_to_hold, 
                                        time_to_finish=self.time_to_finish)
        #self.handlers[self.current_state.upper()][1]()

    def add_state(self, name, handler, hold_handler = None, end_state=0, facultative = 0):
        name = name.upper()
        self.handlers[name] = [handler, hold_handler]
        if end_state:
            self.endStates.append(name)
        if facultative:
            self.facultativeStates.append(name)

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
    def run_state_machine(self, cargo_in, event_in, event_out=None):
        state_stop_event = threading.Event()
        state_stopped = threading.Event()
        state_stop_event.clear()
        state_stopped.clear()
        holdData = None
        newState = "next"
        cargo_out = None
        FSM_holded = False
        print "cargo: ", cargo_in
        my_state = TaskState()
        my_state.node_name = rospy.get_name()
        my_state.status = 1
        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "END STATES: ", self.endStates+"\n")
        try:
            while True:
                # print "1"
                my_state.state_name = str(self.current_state) 
                my_state.state_input = str(cargo_in) 
                self.pub_state.publish( my_state)

                # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "ev1: ", event.isSet()+"\n")
                self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "RUNNING STATE: "+ str(self.current_state.upper())+"\n")
                # self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "Handler: "+ str(self.handlers[self.current_state.upper()][0])+"\n")
                thread = FSMThread(target = self.handlers[self.current_state.upper()][0], cargo_in = cargo_in, event_in = state_stop_event, event_out = state_stopped)
                self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "Starting: "+ str(self.current_state.upper())+"\n")
                
                thread.start()
                # print "3"
                self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "Started: "+ str(self.current_state.upper())+"\n")
                while True:
                    self.print_log.write( "thread: "+ str(thread.is_alive())+"\n")
                    self.print_log.write( "event: "+ str(event_in.isSet())+"\n")
                    self.print_log.write("state_stop_event: "+ str(state_stopped.isSet())+"\n")
                    # if the state is finished and was not stoppped, set new state basing on the return 
                    # of the previous state
                    if (not thread.stopped() and not thread.is_alive() and not state_stopped.isSet()):
                        self.print_log.write( "NEW STATE "+"\n")
                        (newState, cargo_out) = thread.join()
                        self.current_state = newState
                        cargo_in = cargo_out
                        # self.pub_state.publish( str(self.current_state) + " | data: "+ str(cargo_out))
                        break

                    # if state_stopped.isSet():
                    #     state_stopped.clear()
                    #     while thread.is_alive():
                    #         self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM waits for state to terminate"+"\n")
                    #         time.sleep(1)
                    #     (newState, cargo_out) = thread.join()
                    #     self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM launched newState to hold task: "+str(newState.upper())+"\n")
                    #     (self.resumeState, self.resumeData) = self.handlers[newState.upper()][0](cargo_in=cargo_out)
                    #     self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM FINISHED newState as a hold state"+"\n")
                    #     FSM_holded = True
                    #     break
                    
                        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "GOT:"+"\n")
                        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "newState: ", newState+"\n")
                        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "cargo_out: ", cargo_out+"\n")
                    # print "4"

                    # if the hold event of the FSM was set or the state set its own hold event
                    if event_in.isSet() or state_stop_event.isSet() or state_stopped.isSet():
                        self.print_log.write("Setting state event\n")
                        # set state hold event
                        state_stop_event.set()
                        # update data in current_state topic
                        my_state.status = 2
                        self.pub_state.publish(my_state)
                        # wait for state to finish
                        while thread.is_alive():
                            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM waits for current state to terminate"+"\n")
                            time.sleep(1)
                        # if the state set its own hold event, it wants to run a hold state
                        if state_stopped.isSet():
                            (newState, cargo_out) = thread.join()
                            # set next (hold) state
                            self.current_state = newState
                            if self.current_state.upper() in self.endStates:
                                break
                            self.current_state = newState
                            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM launched newState to hold task: "+str(newState.upper())+"\n")
                            # fill current_state topic data
                            my_state.state_name = self.current_state
                            my_state.state_input = str(cargo_out)
                            self.pub_state.publish(my_state)
                            # run hold state
                            (self.resumeState, self.resumeData) = self.handlers[self.current_state.upper()][0](cargo_in=cargo_out)
                            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM FINISHED newState as a hold state"+"\n")
                            FSM_holded = True 
                            # update data in current_state topic
                            my_state.status = 3
                            self.pub_state.publish(my_state)
                            break   
                        # if the FSM event was set, the state was finished, but the state didn't 
                        # want to finish and forced to move to next state (we hope the next state 
                        # will handle the hold event) 
                        else:
                            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM state is not stopped: "+"\n")
                            (newState, cargo_out) = thread.join()
                            self.current_state = newState
                            cargo_in = cargo_out
                            thread = FSMThread(target = self.handlers[self.current_state.upper()][0], cargo_in = cargo_in, event_in = state_stop_event, event_out = state_stopped)
                            thread.start()
                            # fill current_state topic data
                            my_state.state_name = self.current_state
                            my_state.state_input = str(cargo_out)
                            my_state.status = 0
                            self.pub_state.publish(my_state)

                    
                    time.sleep(0.1)
                self.print_log.write( "ev2: "+ str(event_in.isSet())+"\n")
                # the state set the event and the hold state was performed
                if state_stop_event.isSet() and FSM_holded:
                    # set the task state as holded
                    self.task_state = 3
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM finished"+"\n")
                    break
                # if the next state is the end state, 
                if ((self.current_state.upper() in self.endStates) or (self.isInterrupting == 1 and self.current_state.upper() in self.facultativeStates)):
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM reached end state: "+ str(self.current_state)+"\n")
                    self.task_state = 6
                    my_state.status = 5
                    self.pub_state.publish(my_state)
                    break
        finally:
            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM bye"+"\n")
            state_stop_event.set()
            while thread.is_alive():
                thread.join()

    def startTask(self, data):
        self.isInterrupting = data.isInterrupting
        self.startFlag = True
        return StartTaskResponse()

    def getLaunchConditions(self, req):
        l_conditions = LaunchConditionsResponse()
        l_conditions.start_deadline = 0
        l_conditions.task_priority = 0
        return l_conditions
    def run(self, cargo):
        try:
            node_namespace = rospy.get_name() + "/multitasking"
            cond_name = node_namespace + "/get_launch_conditions"
            start_name = node_namespace + "/startTask"
            conditions_srv = rospy.Service(cond_name, LaunchConditions, self.getLaunchConditions)
            start_srv = rospy.Service(start_name, StartTask, self.startTask)
            print "HAVE SERVICES"
            r = rospy.Rate(5)
            while not rospy.is_shutdown():
                self.updateStatus()
                print "Check START"
                if self.startFlag:
                    print "HAVE START"
                    break 
                r.sleep()
            print "HAVE BRAKED"
            start_srv.shutdown()
            print "HAVE SHUTDOWN"

            self.res_srv = None
            self.suspend_srv = None
            node_namespace = rospy.get_name() + "/multitasking"
            self.suspend_srv = rospy.Service(node_namespace+"/hold_now", Trigger, self.onHold)
            self.q = multiprocessing.Queue()
            self.current_state = self.startState
            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "start: "+ str(self.current_state)+"\n"+"\n")
            # handler = self.handlers[self.startState.upper()][0]
            self.task_state = 1 # Running
            print "AAAAA"
        # self.current_state_m_thread = multiprocessing.Process(target = self.run_state_machine, args = (cargo, self.q))
            self.current_state_m_thread = FSMThread(target = self.run_state_machine, cargo_in = cargo, event_in = self.fsm_stop_event)
            self.current_state_m_thread.start()
            while not rospy.is_shutdown():
                if self.task_state == 3:
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM killed, joining thread, waiting for resume"+"\n")
                    self.current_state_m_thread.join()
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM joined thread"+"\n")
                    self.fsm_stop_event.clear()
                    self.task_state = 4
                if self.task_state == 4:
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "Waiting for resume"+"\n")
                    if self.res_srv == None:
                        resume_srv_name = node_namespace+"/startTask"
                        self.res_srv = rospy.Service(resume_srv_name, StartTask, self.onResume)
                        self.suspend_srv.shutdown()
                if self.task_state == 5:
                    self.res_srv.shutdown()
                    self.res_srv = None
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "Resuming fsm"+"\n")
                    #
                    
                    #
                    print "UPDATEING TASK"
                    (self.current_state, cargo) = self.updateTask(TH_data = self.onResumeData)
                    # self.current_state = self.resumeState
                    print cargo
                    print "CS: ",self.current_state
                    my_state = TaskState()
                    my_state.state_name = str(self.current_state) 
                    my_state.state_input = str(cargo) 
                    my_state.node_name = rospy.get_name()
                    my_state.status = 4
                    self.pub_state.publish( my_state)
                    self.fsm_stop_event.clear()
                    self.current_state_m_thread = FSMThread(target = self.run_state_machine, cargo_in = cargo, event_in = self.fsm_stop_event)
                    self.current_state_m_thread.start()
                    suspend_srv_name = node_namespace+"/hold_now"
                    self.suspend_srv = rospy.Service(suspend_srv_name, Trigger, self.onHold)
                    self.task_state = 1    
                if self.task_state == 6:
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM finished, terminating whole task" +"\n")
                    while self.current_state_m_thread.is_alive():
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "trying to kill FSM"+"\n")
                    self.current_state_m_thread.join()
                    self.fsm_stop_event.clear()
                    break
                time.sleep(10)
                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "test"+"\n")
                self.updateStatus()
        finally:
                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FINAL"+"\n")
                self.fsm_stop_event.set()
                while self.current_state_m_thread.is_alive():
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "trying to kill FSM"+"\n")
                    self.current_state_m_thread.join()                # raise InitializationError("must call .set_start() before .run()")
                # if not self.endStates:
                #     raise  InitializationError("at least one state must be an end_state")


