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

global debug
debug = False

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
        global debug
        # cargo = self.cargo
        # event = self._stop_event
        if debug:
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
        global debug
        if debug:
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
    global debug
    ret = None
    pqueue = multiprocessing.Queue() 
    pqueue.put(arguments)
    p = multiprocessing.Process(target=handler, args=(pqueue, ))
    # new_thread = PTFThread(target = handler, args = (arguments,))
    p.daemon = True
    p.start()
    if debug:
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "PID: "+ str( p.pid)+"\n")
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "new thread is alive: "+ str(p.is_alive()) +"\n")
    while (p.is_alive()):
        time.sleep(0.1)
        if state_event.isSet():
            p.terminate()
            if debug:
                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "terminate"+"\n")
        if debug:
            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "waiting for a blocking call to end or a hold signal"+"\n")
    if not state_event.isSet():
        ret = pqueue.get()
    p.join()
    # return_dict.values()
    if debug:
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "after kill"+"\n")
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "VALUES: "+ str(ret)+"\n")
    return ret

class StateMachine:
    def __init__(self, file, isInterrupting, da_ID, ptf_update_task, ptf_update_sp, ptf_suspend_condition, ptf_cost_condition):
        self.startFlag = False
        self.isInterrupting = False
        self.da_ID = da_ID
        self.isInterrupting = isInterrupting
        self.ptf_update_task = ptf_update_task
        self.ptf_update_sp = ptf_update_sp
        self.ptf_suspend_condition = ptf_suspend_condition
        self.ptf_cost_condition = ptf_cost_condition
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
        self.pub_status = rospy.Publisher('TH/statuses', Status, queue_size=10)
        self.time_to_hold = -1 
        self.time_to_finish = -1
        self.onResumeData = None
        self.exec_fsm_state = 0
    
    def updateTask(self, TH_data):

        return self.ptf_update_task(self, TH_data)

    def updateStatus(self):
        global debug
        my_status = Status()
        # self.print_log.write("UPDATEING STATUS id: "+str(self.da_ID)+"\n")
        my_status.da_id = self.da_ID
        my_status.da_state = self.exec_fsm_state
        my_status.schedule_params = self.ptf_update_sp()
        if debug:
            self.print_log.write("UPDATEING STATUS params of: "+str(self.da_ID)+"\n"+str(my_status.schedule_params)+"\n")
        self.pub_status.publish(my_status) 
        # self.print_log.write("STATUS was sent"+"\n")

    def onHold(self, param):
        global debug
        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "HOLD: "+ str( param)+"\n")
        self.task_state = 2 # Holding
        # self.q.put(self.task_state)
        self.fsm_stop_event.set()
        time.sleep(0.1)
        while True:
            if self.current_state_m_thread.is_alive():
                if debug:
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
        global debug
        if debug:
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
        global debug
        state_stop_event = threading.Event()
        state_stopped = threading.Event()
        state_stop_event.clear()
        state_stopped.clear()
        holdData = None
        newState = "next"
        cargo_out = None
        FSM_holded = False
        my_state = TaskState()
        my_state.node_name = rospy.get_name()
        self.exec_fsm_state = 1
        # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "END STATES: ", self.endStates+"\n")
        try:
            while True:
                # print "1"
                my_state.state_name = str(self.current_state) 
                my_state.state_input = str(cargo_in) 
                
                # self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "ev1: ", event.isSet()+"\n")
                if debug:
                    self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "RUNNING STATE: "+ str(self.current_state.upper())+"\n")
                # self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "Handler: "+ str(self.handlers[self.current_state.upper()][0])+"\n")
                thread = FSMThread(target = self.handlers[self.current_state.upper()][0], cargo_in = cargo_in, event_in = state_stop_event, event_out = state_stopped)
                if debug:
                    self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "Starting: "+ str(self.current_state.upper())+"\n")
                
                thread.start()
                # print "3"
                if debug:
                    self.print_log.write("\n"+"\n"+str(datetime.datetime.now().time())+"\n"+ "Started: "+ str(self.current_state.upper())+"\n")
                while True:
                    if debug:
                        self.print_log.write( "thread: "+ str(thread.is_alive())+"\n")
                        self.print_log.write( "event: "+ str(event_in.isSet())+"\n")
                        self.print_log.write("state_stop_event: "+ str(state_stopped.isSet())+"\n")
                    # if the state is finished and was not stoppped, set new state basing on the return 
                    # of the previous state
                    if (not thread.stopped() and not thread.is_alive() and not state_stopped.isSet()):
                        if debug:
                            self.print_log.write( "NEW STATE "+"\n")
                        (newState, cargo_out) = thread.join()
                        self.current_state = newState
                        cargo_in = cargo_out
                        # str(self.current_state) + " | data: "+ str(cargo_out))
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
                        if debug:
                            self.print_log.write("Setting state event\n")
                        # set state hold event
                        state_stop_event.set()
                        # update data in current_state topic
                        self.exec_fsm_state = 2
                        
                        # wait for state to finish
                        while thread.is_alive():
                            if debug:
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
                            if debug:
                                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM launched newState to hold task: "+str(newState.upper())+"\n")
                            # fill current_state topic data
                            my_state.state_name = self.current_state
                            my_state.state_input = str(cargo_out)
                            
                            # run hold state
                            (self.resumeState, self.resumeData) = self.handlers[self.current_state.upper()][0](cargo_in=cargo_out)
                            if debug:
                                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM FINISHED newState as a hold state"+"\n")
                            FSM_holded = True 
                            # update data in current_state topic
                            self.exec_fsm_state = 3
                            
                            break   
                        # if the FSM event was set, the state was finished, but the state didn't 
                        # want to finish and forced to move to next state (we hope the next state 
                        # will handle the hold event) 
                        else:
                            if debug:
                                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM state is not stopped: "+"\n")
                            (newState, cargo_out) = thread.join()
                            self.current_state = newState
                            cargo_in = cargo_out
                            thread = FSMThread(target = self.handlers[self.current_state.upper()][0], cargo_in = cargo_in, event_in = state_stop_event, event_out = state_stopped)
                            thread.start()
                            # fill current_state topic data
                            my_state.state_name = self.current_state
                            my_state.state_input = str(cargo_out)
                            self.exec_fsm_state = 0
                            

                    
                    time.sleep(0.1)
                self.print_log.write( "ev2: "+ str(event_in.isSet())+"\n")
                # the state set the event and the hold state was performed
                if state_stop_event.isSet() and FSM_holded:
                    # set the task state as holded
                    self.task_state = 3
                    if debug:
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM finished"+"\n")
                    break
                # if the next state is the end state, 
                if ((self.current_state.upper() in self.endStates) or (self.isInterrupting == 1 and self.current_state.upper() in self.facultativeStates)):
                    if debug:
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM reached end state: "+ str(self.current_state)+"\n")
                    self.task_state = 6
                    self.exec_fsm_state = 5
                    
                    break
        finally:
            global debug
            if debug:
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

    def getSuspendConditions(self, req):
        return self.ptf_suspend_condition(req)
    def getCostConditions(self, req):
        return self.ptf_cost_condition(req)

    def run(self, cargo):
        try:
            global debug
            node_namespace = rospy.get_name() + "/multitasking"
            start_name = node_namespace + "/startTask"
            start_srv = rospy.Service(start_name, StartTask, self.startTask)
            cost_cond_name = node_namespace + "/get_cost_on_conditions"
            self.cost_cond_srv = rospy.Service(cost_cond_name, CostConditions, self.getCostConditions)
            r = rospy.Rate(5)
            while not rospy.is_shutdown():
                self.updateStatus()
                if self.startFlag:
                    break 
                r.sleep()
            start_srv.shutdown()

            self.res_srv = None
            self.suspend_srv = None
            node_namespace = rospy.get_name() + "/multitasking"
            self.suspend_srv = rospy.Service(node_namespace+"/hold_now", Trigger, self.onHold)
            susp_cond_name = node_namespace + "/get_suspend_conditions"
            self.susp_cond_srv = rospy.Service(susp_cond_name, SuspendConditions, self.getSuspendConditions)
            self.q = multiprocessing.Queue()
            self.current_state = self.startState
            if debug:
                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "start: "+ str(self.current_state)+"\n"+"\n")
            # handler = self.handlers[self.startState.upper()][0]
            self.task_state = 1 # Running
            print "AAAAA"
            if debug:
                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+rospy.get_name()+ ": execFSM start "+"\n")
        # self.current_state_m_thread = multiprocessing.Process(target = self.run_state_machine, args = (cargo, self.q))
            self.current_state_m_thread = FSMThread(target = self.run_state_machine, cargo_in = cargo, event_in = self.fsm_stop_event)
            self.current_state_m_thread.start()
            if debug:
                self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+rospy.get_name()+ ": execFSM started "+"\n")
            while not rospy.is_shutdown():
                self.updateStatus()
                if debug:
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM UPDATE: "+rospy.get_name()+"\n")
                if self.task_state == 3:
                    if debug:
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM killed, joining thread, waiting for resume"+"\n")
                    self.current_state_m_thread.join()
                    if debug:
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM joined thread"+"\n")
                    self.fsm_stop_event.clear()
                    self.task_state = 4
                if self.task_state == 4:
                    if debug:
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "Waiting for resume"+"\n")
                    if self.res_srv == None:
                        resume_srv_name = node_namespace+"/startTask"
                        self.res_srv = rospy.Service(resume_srv_name, StartTask, self.onResume)
                        self.suspend_srv.shutdown()
                        self.susp_cond_srv.shutdown()
                if self.task_state == 5:
                    self.res_srv.shutdown()
                    self.res_srv = None
                    if debug:
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
                    self.exec_fsm_state = 4
                    self.updateStatus()
                    self.fsm_stop_event.clear()
                    self.current_state_m_thread = FSMThread(target = self.run_state_machine, cargo_in = cargo, event_in = self.fsm_stop_event)
                    self.current_state_m_thread.start()
                    suspend_srv_name = node_namespace+"/hold_now"
                    self.suspend_srv = rospy.Service(suspend_srv_name, Trigger, self.onHold)
                    susp_cond_name = node_namespace + "/get_suspend_conditions"
                    self.susp_cond_srv = rospy.Service(susp_cond_name, SuspendConditions, self.getSuspendConditions)
                    self.task_state = 1    
                if self.task_state == 6:
                    if debug:
                        self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FSM finished, terminating whole task" +"\n")
                    while self.current_state_m_thread.is_alive():
                        if debug:
                            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "trying to kill FSM"+"\n")
                    self.current_state_m_thread.join()
                    self.fsm_stop_event.clear()
                    break
                time.sleep(10)
        finally:
                global debug
                if debug:
                    self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "FINAL"+"\n")
                self.fsm_stop_event.set()
                if self.current_state_m_thread != 0:
                    while self.current_state_m_thread.is_alive():
                        if debug:
                            self.print_log.write("\n"+str(datetime.datetime.now().time())+"\n"+ "trying to kill FSM"+"\n")
                        self.current_state_m_thread.join()                # raise InitializationError("must call .set_start() before .run()")
                if not rospy.is_shutdown():
                    self.updateStatus()
                # if not self.endStates:
                #     raise  InitializationError("at least one state must be an end_state")


