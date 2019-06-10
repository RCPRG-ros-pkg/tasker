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
    def initialiseDA(self, application, da_id, da_name, init_params):
        package = 'multitasker' 
        executable = application
        # cmd = "rosrun "+ package + " "+executable+ " "+da_name+ " "+str(da_id)+" "+str(init_params)
        rospy.set_param('/'+da_name+'/fsm_condition/startTask', False)
        subprocess.Popen(['rosrun', package, executable, da_name, str(da_id), init_params])
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
        self.queue[da_id]["priority"] = float(priority)
        # self.queue[da_id]["scheduleParams"].priority = float(priority)
        # print ("NQ: ", self.queue[da_id])
        self.lock.release()
    def updateScheduleParams(self, da_id, scheduleParams):
        # type: (int, ScheduleParams) -> None
        self.lock.acquire()
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
                    print("queue C")
                    continue
            if self.isExecuting():
                if self.execField["da_id"] == i:
                    i = i + 1
                    print("E C")
                    continue
            if self.isInterrupting():
                if self.interruptField["da_id"] == i:
                    i = i + 1
                    print("I C")
                    continue
            print("break")
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
            print("executing")
            if next_da["priority"] > self.execField["priority"]:
                if not self.isInterrupting():
                    print("NO INTERRUPTING DA, ", next_da["da_id"], " is interrupting now")
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()

                elif next_da["priority"] > self.interruptField["priority"]:
                    print("da: ", next_da["da_id"],"priority: ",next_da["priority"], "\n Replaces :", self.interruptField["da_id"], "with priority: ", self.interruptField["priority"])
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()
    def schedule(self):
        self.lock.acquire()
        if self.isExecuting():
            exec_da_name = "/"+self.execField["da_name"]
            print("checking  EXEC node: ", exec_da_name)
            if not rosnode.rosnode_ping(exec_da_name, 1):
                print("FINIIIIIISSSSSHHHHHHEEEEDDDD")
                self.execField = {}
        print (self.queue)
        for key_id in self.queue.items():
            self.queue[key_id[0]]["priority"] = self.queue[key_id[0]]["scheduleParams"].priority
        if len(self.queue) > 0:
            q = OrderedDict(sorted(self.queue.items(), 
                            key=lambda kv: kv[1]['priority'], reverse=True))
            # print ("Q: ",q)
            self.updateQueue(q)
        self.lock.release()

    def switchDA(self):
        self.switchIndicator.wait()
        print("\nSWITCHING\n")
        self.lock.acquire()
        if self.isExecuting():
            commanding = self.execField
            self.lock.release()
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
        print (srv_name)
        rospy.wait_for_service(srv_name)
        start_srv = rospy.ServiceProxy(srv_name, StartTask)
        start_srv(False, "")
        rospy.wait_for_service('/'+interrupting["da_name"]+'/multitasking/get_hold_conditions')
        self.lock.acquire()
        self.makeExecuting()
        self.switchIndicator.clear()
        self.lock.release()
        
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