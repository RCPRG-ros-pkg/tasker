from collections import OrderedDict
from multitasker.msg import *
import threading
import time

class TaskHarmoniser():
    def __init__(self):
        self.switchIndicator = threading.Event()
        self.switchIndicator.clear()
        self.init_da = {'da_id': 0, 'priority': float('-inf'), 'scheduleParams': ScheduleParams()}
        self.lock = threading.Lock()
        self.queue = {}
        self.OrderedQueue = {}
        self.execField = {}
        self.interruptField = {}
    def addDA(self, added: int):
        self.lock.acquire()
        da = {'da_id': added, 'priority': float('-inf'), 'scheduleParams': ScheduleParams()}
        self.queue[added] = da
        self.lock.release()
    def updateDA(self, da_id: int, priority: int, scheduleParams: ScheduleParams):
        da = {'da_id': da_id, 'priority': priority, 'scheduleParams': scheduleParams}
    def updateDA(self, da: dict):
        self.queue[da["da_id"]] = da
    def updatePriority(self, da_id: int, priority: int):
        self.lock.acquire()
        # print ("q: ", self.queue[da_id],"\n p: ",self.queue[da_id]["priority"], "\n new_q: ",priority)
        self.queue[da_id]["priority"] = float(priority)
        # print ("NQ: ", self.queue[da_id])
        self.lock.release()
    def updateScheduleParams(self, da_id: int, scheduleParams: ScheduleParams):
        self.queue[da_id]["scheduleParams"] = scheduleParams
    def makeInterrupting(self, da_id: int):
        self.interruptField = self.queue[da_id]
        del self.queue[da_id]
    def makeExecuting(self, da_id: int):
        self.execField = self.interruptField
        self.interruptField = self.init_da
    def isExecuting(self):
        return len(self.execField) != 0
    def updateQueue(self, new_queue: OrderedDict):
        self.OrderedQueue = new_queue
        next_da = next(iter(new_queue.items()))[1]
        if not self.isExecuting():
            print("not executing")
            self.makeInterrupting(next_da["da_id"])
            if not self.switchIndicator.isSet():
                self.switchIndicator.set()
        else:
            print("executing")
            if next_da["priority"] > self.execField["priority"]:
                if next_da["priority"] > self.interruptField["priority"]:
                    print("da: ", next_da["da_id"],"priority: ",next_da["priority"], "\n Replaces :", self.interruptField["da_id"], "with priority: ", self.interruptField["priority"])
                    self.updateDA(da=self.interruptField)
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.switchIndicator.set()
    def schedule(self):
        self.lock.acquire()
        for key_id in self.queue.items():
            self.queue[key_id[0]]["priority"] = self.queue[key_id[0]]["scheduleParams"].priority + key_id[0]
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
            print("SEND SUSPEND to commanding: ", commanding["da_id"])
            i = 0
            while i < 1:
                i = i+1
                time.sleep(1)
        else:
            self.lock.release()
        self.lock.acquire()
        interrupting = self.interruptField
        print("SEND StartTask to initialised: ", interrupting["da_id"])
        self.makeExecuting(interrupting["da_id"])
        self.lock.release()
        i = 0
        while i < 3:
            i = i+1
            time.sleep(1)
        self.lock.acquire()
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