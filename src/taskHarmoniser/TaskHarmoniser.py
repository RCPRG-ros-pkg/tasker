from collections import OrderedDict
from multitasker.msg import *
from multitasker.srv import *
import threading
import time
import subprocess
import rospy
from std_srvs.srv import Trigger, TriggerRequest
import rosnode

class TaskHarmoniser():
    def __init__(self):
        self.switchIndicator = threading.Event()
        self.switchIndicator.clear()
        self.init_da = {'da_id': -1, 'da_name': None, 'da_type': None, 'priority': float('-inf'), 'scheduleParams': ScheduleParams()}
        self.lock = threading.Lock()
        self.queue = {}
        self.OrderedQueue = {}
        self.execField = {}
        self.interruptField = {}
    def initialiseDA(self, application, version, da_id, da_name, init_params):
        package = 'multitasker' 
        executable = application
        # cmd = "rosrun "+ package + " "+executable+ " "+da_name+ " "+str(da_id)+" "+str(init_params)
        rospy.set_param('/'+da_name+'/fsm_condition/startTask', False)
        subprocess.Popen(['rosrun', package, executable, version, da_name, str(da_id), init_params])
    def addDA(self, added, da_name, da_type):
        # type: (int) -> None
        self.lock.acquire()        
        da = {'da_id': added, 'da_name': da_name, 'da_type': da_type, 'priority': float('-inf'), 'scheduleParams': ScheduleParams()}
        self.queue[added] = da
        self.lock.release()
    def updateDA(self, da_id, da_name, da_type, priority, scheduleParams):
        # type: (int, int, ScheduleParams) -> None
        da = {'da_id': da_id, 'da_name': da_name, 'da_type': da_type, 'priority': priority, 'scheduleParams': scheduleParams}
    def updateDA(self, da):
        # type: (dict) -> None
       self.queue[da["da_id"]] = da
    def updatePriority(self, da_id, priority):
        # type: (int, int) -> None
        self.lock.acquire()
        # print ("q: ", self.queue[da_id],"\n p: ",self.queue[da_id]["priority"], "\n new_q: ",priority)
        if self.isExecuting():
            if self.execField["da_id"] == da_id:
                self.execField["priority"] = float(priority)
                self.lock.release()
                return
        if self.isInterrupting():
            if self.interruptField["da_id"] == da_id:
                self.interruptField["priority"] = float(priority)
                self.lock.release()
                return
        self.queue[da_id]["priority"] = float(priority)
        # self.queue[da_id]["scheduleParams"].priority = float(priority)
        # print ("NQ: ", self.queue[da_id])
        self.lock.release()
    def updateScheduleParams(self, da_id, scheduleParams):
        # type: (int, ScheduleParams) -> None
        self.lock.acquire()
        if self.isExecuting():
            if self.execField["da_id"] == da_id:
                self.execField["scheduleParams"] = scheduleParams
                self.lock.release()
                return
        if self.isInterrupting():
            if self.interruptField["da_id"] == da_id:
                self.interruptField["scheduleParams"] = scheduleParams
                self.lock.release()
                return
        self.queue[da_id]["scheduleParams"] = scheduleParams
        self.lock.release()
    def makeInterrupting(self, da_id):
        # type: (int) -> None
        if self.isInterrupting():
            self.updateDA(self.interruptField)
        self.interruptField = self.queue[da_id]
        del self.queue[da_id]
    def makeExecuting(self):
        # type: () -> None
        if self.isExecuting():
            self.updateDA(self.execField)
        self.execField = self.interruptField
        self.interruptField = {}
    def isInterrupting(self):
         # type: () -> bool
        return len(self.interruptField) != 0
    def isExecuting(self):
         # type: () -> bool
        return len(self.execField) != 0
    def sendIndicator(self):
         # type: () -> None
        self.switchIndicator.set()
    def getNextID(self):
         # type: () -> int8
        self.lock.acquire()
        i = 0
        while True:
            for key_id in self.queue.items():
                if self.queue[key_id[0]]["da_id"] == i:
                    i = i + 1
                    # print("queue C")
                    continue
            if self.isExecuting():
                if self.execField["da_id"] == i:
                    i = i + 1
                    # print("E C")
                    continue
            if self.isInterrupting():
                if self.interruptField["da_id"] == i:
                    i = i + 1
                    # print("I C")
                    continue
            # print("break")
            break
        self.lock.release()
        return i
    def updateQueue(self, new_queue):
        # type: (OrderedDict) -> None
        self.OrderedQueue = new_queue
        # print ("NQ: ",new_queue)
        next_da = next(iter(new_queue.items()))[1]
        # print ("NDA: ", next_da)
        # print ("NDA_ID: ", next_da["da_id"])
        if not self.isExecuting():
            print("not executing")
            self.makeInterrupting(next_da["da_id"])
            if not self.switchIndicator.isSet():
                self.switchIndicator.set()
        else:
            # print("executing")
            if next_da["priority"] > self.execField["priority"]:
                if not self.isInterrupting():
                    print("NO INTERRUPTING DA, ", next_da["da_id"], " is interrupting now")
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()

                elif next_da["priority"] > self.interruptField["priority"]:
                    # print("da: ", next_da["da_id"],"priority: ",next_da["priority"], "\n Replaces :", self.interruptField["da_id"], "with priority: ", self.interruptField["priority"])
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()
        def updateIrrField(self, next_da):
        # print ("NDA: ", next_da)
        # print ("NDA_ID: ", next_da["da_id"])
        if not self.isExecuting():
            print("not executing")
            self.makeInterrupting(next_da["da_id"])
            if not self.switchIndicator.isSet():
                self.switchIndicator.set()
        else:
            # print("executing")
            if next_da["priority"] > self.execField["priority"]:
                if not self.isInterrupting():
                    print("NO INTERRUPTING DA, ", next_da["da_id"], " is interrupting now")
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()

                elif next_da["priority"] > self.interruptField["priority"]:
                    # print("da: ", next_da["da_id"],"priority: ",next_da["priority"], "\n Replaces :", self.interruptField["da_id"], "with priority: ", self.interruptField["priority"])
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()
    def schedule(self):

        # print("\nSCHEDULE\n")
        self.lock.acquire()
        if self.isExecuting():
            exec_da_name = "/"+self.execField["da_name"]
            # print("checking  EXEC node: ", exec_da_name)
            if not rosnode.rosnode_ping(exec_da_name, 1):
                print("FINIIIIIISSSSSHHHHHHEEEEDDDD")
                self.execField = {}
            else:
                self.execField["priority"] = self.execField["scheduleParams"].cost
        if self.isInterrupting():
            self.interruptField["priority"] = self.interruptField["scheduleParams"].cost 
            if self.isExecuting():
                self.interruptField["priority"] += self.interruptField["scheduleParams"].cost_per_sec * self.execField["scheduleParams"].completion_time
        # print (self.queue)
        for key_id in self.queue.items():
            # queued_da_name = "/"+self.queue[key_id[0]]["da_name"]

            # if not rosnode.rosnode_ping(queued_da_name, 1):
            #     print("DA terminated")
            #     del self.queue[key_id[0]]
            # else:
            self.queue[key_id[0]]["priority"] = self.queue[key_id[0]]["scheduleParams"].cost 
            if self.isExecuting():
                self.queue[key_id[0]]["priority"] += self.queue[key_id[0]]["scheduleParams"].cost_per_sec * self.execField["scheduleParams"].completion_time
        if len(self.queue) > 0:
            q = OrderedDict(sorted(self.queue.items(), 
                            key=lambda kv: kv[1]['priority'], reverse=True))
            # print ("Q: ",q)
            if self.isExecuting():
                highest_da = next(iter(q.items()))[1]
                if self.execField["priority"] < highest_da["priority"]:
                    print "WAITING FOR CONDITIONS"
                    rospy.wait_for_service('/'+self.execField["da_name"]+'/multitasking/get_suspend_conditions')
                    get_susp_cond = rospy.ServiceProxy('/'+self.execField["da_name"]+'/multitasking/get_suspend_conditions', SuspendConditions)
                    trig = SuspendConditionsRequest()
                    resp = get_susp_cond(highest_da["scheduleParams"].final_resource_state)
                    print "HAVE CONDITIONS"
                    self.execField["priority"] = self.execField["scheduleParams"].cost + resp.cost_per_sec*highest_da["scheduleParams"].completion_time
            self.updateQueue(q)
        ###
        #  print queue and fields
        ###
        if not self.isExecuting():
            print "No EXEC dynamic agent"
        else:
            print "\nEXEC: "
            print "ID: ", self.execField["da_id"]
            print "Priority: ", self.execField["priority"]
            print "SP: \n", self.execField["scheduleParams"], "\n"
        if not self.isInterrupting():
            print "No INTERRUPTING dynamic agent"
        else:
            print "\tINTERRUPT: "
            print "ID: ", self.interruptField["da_id"]
            print "Priority: ", self.interruptField["priority"]
            print "SP: \n", self.interruptField["scheduleParams"], "\n"
        print "\tQUEUE: "
        for key_id in self.queue.items():
            print "ID: ", self.queue[key_id[0]]["da_id"]
            print "Priority: ", self.queue[key_id[0]]["priority"]
            print "SP: \n", self.queue[key_id[0]]["scheduleParams"], "\n"

        self.lock.release()
        # print("\nSCHEDULED\n")
    def schedule_new(self):

        # print("\nSCHEDULE\n")
        self.lock.acquire()
        if self.isExecuting():
            exec_da_name = "/"+self.execField["da_name"]
            # print("checking  EXEC node: ", exec_da_name)
            if not rosnode.rosnode_ping(exec_da_name, 1):
                print("FINIIIIIISSSSSHHHHHHEEEEDDDD")
                self.execField = {}
            else:
                self.execField["priority"] = self.execField["scheduleParams"].cost

        if len(self.queue) > 0:
            DAset_GH = {}
            DAset_T = {}
            DAset_GH = {k: v for k, v in self.queue.items() if v[1]["da_type"] == "tiago_guideHuman"}
            DAset_T = {k: v for k, v in self.queue.items() if v[1]["da_type"] == "tiago_transport"}
            
            q_GH = OrderedDict(sorted(DAset_GH.items(), 
                            key=lambda kv: kv[1]['priority'], reverse=True))
            q_T = OrderedDict(sorted(DAset_T.items(), 
                            key=lambda kv: kv[1]['priority'], reverse=True))
            if len(DAset_GH) > 0:
                cGH = next(iter(q_GH.items()))[1]
            if len(DAset_T) > 0:
                cT = next(iter(q_T.items()))[1]
            if not (len(DAset_GH) > 0 or len(DAset_T) > 0):
                print "No candidate"
                return
            dac = {}
            if self.isExecuting():
                if  self.execField["da_type"] == "tiago_guideHuman":
                    dac = cGH
                elif self.execField["da_type"] == "tiago_transport":
                    if len(DAset_GH) > 0:
                        self.updateIrrField(cGH)
                        return
                    elif len(DAset_T) > 0:
                        dac = cT
                    else:
                        print "No candidate"
                else:
                    print "DA in ExecField has unknown type task"
                if dac == {}:
                    print "UNKNOWN ERROR"
                    return
                if dac["priority"] > self.execField["priority"]:
                    print "WAITING FOR COST from exec"
                    rospy.wait_for_service('/'+self.execField["da_name"]+'/multitasking/get_cost_on_conditions')
                    get_susp_cond = rospy.ServiceProxy('/'+self.execField["da_name"]+'/multitasking/get_cost_on_conditions', SuspendConditions)
                    trig = SuspendConditionsRequest()
                    resp = get_susp_cond(dac["scheduleParams"].final_resource_state)
                    cc_exec = resp.cost_to_resume
                    ccps_exec = resp.cost_per_sec

                    print "WAITING FOR COST from candidate"
                    rospy.wait_for_service('/'+dac["da_name"]+'/multitasking/get_cost_on_conditions')
                    get_susp_cond = rospy.ServiceProxy('/'+dac["da_name"]+'/multitasking/get_cost_on_conditions', SuspendConditions)
                    trig = SuspendConditionsRequest()
                    resp = get_susp_cond(self.execField["scheduleParams"].final_resource_state)
                    cc_dac = resp.cost_to_resume
                    ccps_dac = resp.cost_per_sec

                    c_switch = dac["priority"] + cc_exec + ccps_exec * dac["scheduleParams"].completion_time 
                    c_wait = self.execField["priority"] + cc_dac + ccps_dac * self.execField["scheduleParams"].completion_time 
                    if c_switch > c_wait:
                        self.updateIrrField(dac)
                else:
                    print "candidate priority was less then executing task"
            else:
                if len(DAset_GH) > 0:
                    self.updateIrrField(cGH)
                    return
                elif len(DAset_T) > 0:
                    self.updateIrrField(cT)
                else:
                    print "No candidate"

        if not self.isExecuting():
            print "No EXEC dynamic agent"
        else:
            print "\nEXEC: "
            print "ID: ", self.execField["da_id"]
            print "Priority: ", self.execField["priority"]
            print "SP: \n", self.execField["scheduleParams"], "\n"
        if not self.isInterrupting():
            print "No INTERRUPTING dynamic agent"
        else:
            print "\tINTERRUPT: "
            print "ID: ", self.interruptField["da_id"]
            print "Priority: ", self.interruptField["priority"]
            print "SP: \n", self.interruptField["scheduleParams"], "\n"
        print "\tQUEUE: "
        for key_id in self.queue.items():
            print "ID: ", self.queue[key_id[0]]["da_id"]
            print "Priority: ", self.queue[key_id[0]]["priority"]
            print "SP: \n", self.queue[key_id[0]]["scheduleParams"], "\n"

        self.lock.release()
        # print("\nSCHEDULED\n")

    def switchDA(self):
        self.switchIndicator.wait()
        print("\nSWITCHING\n")
        self.lock.acquire()
        if self.isExecuting():
            commanding = self.execField
            self.lock.release()
            print("\nSWITCHING: waiting for EXEC hold_now\n")
            rospy.wait_for_service('/'+self.execField["da_name"]+'/multitasking/hold_now')
            print("\nSWITCHING: have hold_now\n")
            hold_srv = rospy.ServiceProxy('/'+self.execField["da_name"]+'/multitasking/hold_now', Trigger)
            trig = TriggerRequest()
            resp = hold_srv(trig)
            print("SEND SUSPEND to commanding: ", commanding["da_id"])
            i = 0
            while i < 1:
                i = i+1
                time.sleep(1)
        else:
            self.lock.release()
        self.lock.acquire()
        interrupting = self.interruptField
        self.lock.release()
        print("SEND StartTask to initialised: ", interrupting["da_name"])
        srv_name = "/"+interrupting["da_name"]+"/multitasking/startTask"
        # print (srv_name)
        print("\nSWITCHING: waiting for QUEUED startTask\n")
        rospy.wait_for_service(srv_name)
        start_srv = rospy.ServiceProxy(srv_name, StartTask)
        start_srv(False, "")
        print("\nSWITCHING: waiting for STARTED hold_now\n")
        rospy.wait_for_service('/'+interrupting["da_name"]+'/multitasking/hold_now')
        self.lock.acquire()
        # print("\n Making executing\n")
        self.makeExecuting()
        self.switchIndicator.clear()
        # print("\n Made executing\n")
        self.lock.release()
        print("\nSWITCHED\n")
        
        # self.isInterrupting = isInterrupting
        # self.print_log = file
        # self.handlers = {}
        # self.startState = None
        # self.endStates = []
        # self.facultativeStates = []
        # self.priority = 0
        # self.start_deadline = -1
        # node_namespace = rospy.get_name() + "/multitasking"
        # srv_name = rospy.get_name()+'/multitasking/get_hold_conditions'
        # self.s = rospy.Service(srv_name, HoldConditions, self.getHoldConditions)
        # self.current_state = ''
        # self.task_state = 0 # initialized_not_running
        # self.current_state_m_thread = 0
        # self.res_srv = None
        # self.suspend_srv = None
        # # self.sub_hold = rospy.Subscriber(node_namespace+"/hold_now", String, self.onHold)
        # # self.sub_res = rospy.Subscriber(node_namespace+"/resume_now", String, self.onResume)
        # self.q = multiprocessing.Queue()
        # self.fsm_stop_event = threading.Event()
        # self.resumeState = None
        # self.resumeData = None
        # self.pub_state = rospy.Publisher('current_state', TaskState, queue_size=10)
        # self.time_to_hold = -1 
        # self.time_to_finish = -1
        # self.onResumeData = None