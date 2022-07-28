# -*- coding: utf-8 -*-

from collections import OrderedDict

from cv2 import log
from tasker_msgs.msg import *
from tasker_msgs.srv import *
from tasker_comm import TaskerCommunicator #, THACommunicator
from dynamic_agent import DynAgent
import threading
import time
import subprocess
import rospy
from std_srvs.srv import Trigger, TriggerRequest
import rosnode
import rosservice
import RequestTable
from datetime import datetime, timedelta
import random
from TaskerLogger import TaskerLogger
import importlib
from rospkg import RosPack

tl = TaskerLogger(agent_name='Harmoniser', log_level='info')
logger = tl.get_logger()

class TaskHarmoniserAgent(object):
    class TaskTypePriorityMap():
        def __init__(self):
            self.map = []
        def addTaskType(self, task_type, type_priority):
            isinstance(task_type,str)
            isinstance(type_priority, int)
            self.map.append({'task_type':task_type, 'priority':type_priority})
        def getMap(self):
            return self.map

    def __init__(self, task_type_priority_map=TaskTypePriorityMap()):
        self.rospack = RosPack()
        self.switchIndicator = threading.Event()
        self.switchIndicator.clear()
        self.init_da = {'da_id': -1, 'da_name': None, 'da_type': None, 'da_state': None, 'priority': float('-inf'), 'scheduleParams': ScheduleParams()}
        self.lock = threading.Lock()
        self.queue = {}
        # self.tasker_communicator =  THACommunicator(logger=logger)
        self.tasker_communicator =  TaskerCommunicator(logger=logger)
        self.tasker_communicator.set_status(self.updateQueueData, 'harmoniser')
        self.sdhl_pub = rospy.Publisher("/TH/shdl_data", ShdlDataStamped)
        #self.cmd_pub = rospy.Publisher("/TH/cmd", CMD)
        self.OrderedQueue = {}
        self.execField = None
        self.interruptField = None
        self.tha_is_running = True
        self.request_table = RequestTable.RequestTable()
        self.task_type_priority_map = task_type_priority_map
        
        # logger.debug("starting cmd updateQueueDataThread thread")
        # self.thread_status_update = threading.Thread(target=self.updateQueueDataThread, args=(1,))
        # self.thread_status_update.start()
        # logger.debug ("started cmd updateQueueDataThread thread")

        #self.sub_status = rospy.Subscriber("TH/statuses", Status, self.updateQueueDataThread)
        self.DA_processes = {}
        self._switch_priority = None
        self.debug_file = False

    def __del__(self):
        self.tasker_communicator.close(id='harmoniser')
        del self.tasker_communicator
        self.tha_is_running = False
        # self.thread_status_update.join()

    def close(self):
        self.tasker_communicator.close(id='harmoniser')
        self.tha_is_running = False
        # self.thread_status_update.join()

    def getTasks(self):
        return self.request_table.items()

    # def updateQueueDataThread(self, args):

    #     while not rospy.is_shutdown() and self.tha_is_running:
    #         msg = self.tasker_communicator.sub_status()
    #         if msg == None:
    #             logger.debug("\nNone msg\n")
    #             break
    #         self.updateQueueData(msg)

    def updateQueueData(self, data):
        global th
        if data is None:
            return 
        logger.debug("\nUPDATE SP\n")
        logger.debug("updateQueueData: '{0}' state: '{1}'\nSP:'{2}'\n".format(data.da_name,data.da_state,data.schedule_params))

        da = self.request_table.get_requst(data.da_id)
        da.shdl_params = data.schedule_params
        da.state = data.da_state
        self.updateDA(da)

        # self.updateScheduleParams(data.da_id, data.schedule_params)
        # priority = self.computePriority(data.schedule_params)
        # self.updatePriority(data.da_id, priority)
        # self.updateDAState(data.da_id, data.da_state)
        logger.debug("\nUPDATED SP\n")

    def initialiseDA(self, executable, da_type,da_id, shdl_rules, args):
        da_name = "DA_"+str(da_id)
        args.append( 'da_id' )
        args.append( str(da_id) )
        args.append( 'da_name' )
        args.append( da_name )
        args.append( 'da_type' )
        args.append( da_type )
        logger.debug("args:'{0}'".format(args))
        da_module = importlib.import_module('.'+str(da_type), 'TaskER.tasks')
        da_state_name = []
        da = DynAgent( da_name, da_id, da_type, da_module.ptf_csp, da_state_name, self.tasker_communicator )
        
        # da_class = da_module.MyTaskER()

        # run_cmd = 
        # #args = ' '.join(map(str, args))
        # run_cmd.append(executable)
        # run_cmd.extend(args)
        # package = 'multitasker'
        logger.debug("Setting DA thread")
        tasker_module = da_module.MyTaskER(da_state_name,da_name, args)
        logger.debug("DA Set")
        t = threading.Thread(target=da.run, args=(tasker_module,))

        row = {'da_id': da_id, 'thread': t, 'module': tasker_module}
        self.DA_processes[da_id] = row
        logger.debug("Starting DA ")
        t.start()
        logger.debug("DA Started")
        logger.debug("Adding DA ")
        self.addDA(da_id, da_name, da_type, shdl_rules, datetime.now())
    def addDA(self, added, da_name, da_type, shdl_rules, req_time):
        # type: (int) -> None
        self.lock.acquire()    
        rec = RequestTable.TaskerReqest(ID=added, huid=da_name, shdl_rules=shdl_rules, req_time=req_time, plan_args={'plan':'static', 'da_type':da_type}, priority=-1)
        rec.state = ['Init']
        rec.last_cmd_sent =None
        self.set_priority(rec)
        self.request_table.addRecord(rec)    
        # da = {'da_id': added, 'da_name': da_name, 'da_type': da_type, 'da_state': ["Init"], 'last_cmd_sent': None,  'priority': float('-inf'), 'ping_count': 0, 'scheduleParams': ScheduleParams()}
        # self.queue[added] = da
        self.lock.release()
    # def updateDA(self, da_id, da_name, da_type, da_state, priority, scheduleParams):
    #     # type: (int, int, ScheduleParams) -> None
    #     da = {'da_id': da_id, 'da_name': da_name, 'da_type': da_type, 'da_state': da_state, 'priority': priority, 'ping_count': 0, 'scheduleParams': scheduleParams}
    def set_priority(self, da):

        task_type_spec = filter(lambda x: da.plan_args['da_type']==x.get('task_type'),self.task_type_priority_map)[0]
        logger.debug(task_type_spec)
        logger.debug(task_type_spec['priority'])
        da.priority = task_type_spec['priority']

    def updateDA(self, da):
        # type: (dict) -> None
        self.set_priority(da)
        # task_type_spec = filter(lambda x: da.plan_args['da_type']==x.get('task_type'),self.task_type_priority_map)[0]
        # print(task_type_spec)
        # print(task_type_spec['priority'])
        # da.priority = task_type_spec['priority']
        self.request_table.updateRecord(da)
        # self.queue[da["da_id"]] = da
    def removeDA(self, da):
        self.lock.acquire()
        # type: (dict) -> None
        if self.isInterrupting():
            if self.interruptField == da.id:
                self.interruptField = None
        if self.isExecuting(): 
            if self.execField == da.id:
                self.execField = None
        self.request_table.removeRecord_by_id(da.id)
        # self.queue.pop(da["da_id"], None)
        t = self.DA_processes[da.id]['thread']
        logger.debug("Joining Thread of '{0}'".format(da.id))
        t.join()
        logger.debug("Joined Thread of '{0}'".format(da.id))
        mod = self.DA_processes[da.id]['module']
        del mod
        # Wait until process terminates (without using p.wait())
        #
        #
        
        # while p.poll() is None:
        #     # Process hasn't exited yet, let's wait some
        #     print("TH waits for DA: "+da["da_id"]+" termination")
        #     time.sleep(0.5)
        self.lock.release()

    def removeDA_no_lock(self, da):
        # type: (dict) -> None
        if self.isInterrupting():
            if self.interruptField == da.id:
                self.interruptField = None
        if self.isExecuting(): 
            if self.execField == da.id:
                self.execField = None
        self.request_table.removeRecord_by_id(da.id)
        # self.queue.pop(da["da_id"], None)
        t = self.DA_processes[da.id]['thread']
        logger.debug("Joining Thread of '{0}'".format(da.id))
        t.join()
        logger.debug("Joined Thread of '{0}'".format(da.id))
        # Wait until process terminates (without using p.wait())
        #
        #

        # while p.poll() is None:
        #     # Process hasn't exited yet, let's wait some
        #     print("TH waits for DA: "+da["da_name"]+" termination")
        #     time.sleep(0.5)
    # def updatePriority(self, da_id, priority):
    #     # type: (int, int) -> None
    #     self.lock.acquire()
    #     # print ("q: ", self.queue[da_id],"\n p: ",self.queue[da_id]["priority"], "\n new_q: ",priority)
    #     if self.isExecuting():
    #         if self.execField["da_id"] == da_id:
    #             self.execField["priority"] = float(priority)
    #             self.lock.release()
    #             return
    #     if self.isInterrupting():
    #         if self.interruptField["da_id"] == da_id:
    #             self.interruptField["priority"] = float(priority)
    #             self.lock.release()
    #             return
    #     if self.queue.has_key(da_id):
    #         self.queue[da_id]["priority"] = float(priority)
    #     else:
    #         print ("[TH] - tried to update Priority of DA_",da_id," but there is no such DA" )
    #     # self.queue[da_id]["scheduleParams"].priority = float(priority)
    #     # print ("NQ: ", self.queue[da_id])
    #     self.lock.release()

    # def computePriority(self, schedule_params):
    #     priority = -1* schedule_params.cost 
    #     return priority

    # def updateScheduleParams(self, da_id, scheduleParams):
    #     # type: (int, ScheduleParams) -> None
    #     self.lock.acquire()
    #     if self.isExecuting():
    #         if self.execField["da_id"] == da_id:
    #             self.execField["scheduleParams"] = scheduleParams

    #             self.lock.release()
    #             return
    #     if self.isInterrupting():
    #         if self.interruptField["da_id"] == da_id:
    #             self.interruptField["scheduleParams"] = scheduleParams
    #             self.lock.release()
    #             return
    #     if self.queue.has_key(da_id):
    #         self.queue[da_id]["scheduleParams"] = scheduleParams
    #     else:
    #         print ("[TH] - tried to update Schedule Params of DA_",da_id," but there is no such DA" )
    #     self.lock.release()
    # def updateDAState(self, da_id, da_state):
    #     # type: (int, ScheduleParams) -> None
    #     self.lock.acquire()
    #     if self.isExecuting():
    #         if self.execField["da_id"] == da_id:
    #             self.execField["da_state"] = da_state
    #             self.lock.release()
    #             return
    #     if self.isInterrupting():
    #         if self.interruptField["da_id"] == da_id:
    #             self.interruptField["da_state"] = da_state
    #             self.lock.release()
    #             return
    #     if self.queue.has_key(da_id):
    #         self.queue[da_id]["da_state"] = da_state
    #     else:
    #         print ("[TH] - tried to update STATE of DA_",da_id," but there is no such DA" )
    #     self.lock.release()
    def updateDALastCMD(self, da_id, cmd):
        # type: (int, ScheduleParams) -> None
        self.lock.acquire()
        self.request_table.get_requst(da_id)
        if self.request_table.get_requst(da_id) is not None:
            self.request_table.get_requst(da_id).last_cmd_sent = cmd
        # if self.isExecuting():
        #     if self.execField == da_id:
        #         self.execField["last_cmd_sent"] = cmd
        #         self.lock.release()
        #         return
        # if self.isInterrupting():
        #     if self.interruptField["da_name"] == da_name:
        #         self.interruptField["last_cmd_sent"] = cmd
        #         self.lock.release()
        #         return
        # if self.queue.has_key(da_name):
        #     self.queue[da_name]["last_cmd_sent"] = cmd
        else:
            logger.warning ("[TH] - tried to update last CMD of '{0}' but there is no such DA".format(da_id) )
        self.lock.release()
    def getDALastCMD(self, da_id):
        # type: (int, ScheduleParams) -> None
        self.lock.acquire()

        if self.request_table.get_requst(da_id) is not None:
            self.lock.release()
            return self.request_table.get_requst(da_id).last_cmd_sent

        # if self.isExecuting():
        #     if self.execField["da_name"] == da_id:
        #         self.lock.release()
        #         return self.execField["last_cmd_sent"]                
        # if self.isInterrupting():
        #     if self.interruptField["da_name"] == da_id:
        #         self.lock.release()
        #         return self.interruptField["last_cmd_sent"]
        # if self.queue.has_key(da_id):
        #     self.lock.release()
        #     return self.queue[da_id]["last_cmd_sent"]
        else:
            logger.warning ("[TH] - tried to get last CMD of '{0}' but there is no such DA".format(da_id) )
        self.lock.release()
    def makeInterrupting(self, da_id):
        # type: (int) -> None
        if self.isInterrupting():
            self.updateDA(self.request_table.get_requst(self.interruptField))
        self.interruptField = da_id
        # del self.queue[da_id]
    def makeExecuting(self):
        # type: () -> None
        # if self.isExecuting():
        #     # print ("=================================")
        #     # print ("adding DA: ", self.execField)
        #     # print ("=================================")
        #     self.updateDA(self.request_table.get_requst(self.execField))
        #     # print ("=================================")
        #     # print ("queue: ", self.queue)
        #     # print ("=================================")
        self.execField = self.interruptField
        self.interruptField = None
    def isInterrupting(self):
         # type: () -> bool
        return self.interruptField != None
    def isExecuting(self):
         # type: () -> bool
        return self.execField != None
    def sendIndicator(self, switch_priority):
         # type: () -> None
        self._switch_priority = switch_priority
        self.switchIndicator.set()
    # def getInterruptingAndExecuting(self):
    #     return [self.interruptField,self.execField]
    # def getQueue(self):
    #     queue = self.queue
    #     return queue
    def getNextID(self):
         # type: () -> int8
        self.lock.acquire()
        i = self.request_table.getNextID()
        # i = 0
        # while True:
        #     for key_id in self.queue.items():
        #         if self.queue[key_id[0]]["da_id"] == i:
        #             i = i + 1
        #             # print("queue C")
        #             continue
        #     if self.isExecuting():
        #         if self.execField["da_id"] == i:
        #             i = i + 1
        #             # print("E C")
        #             continue
        #     if self.isInterrupting():
        #         if self.interruptField["da_id"] == i:
        #             i = i + 1
        #             # print("I C")
        #             continue
        #     # print("break")
        #     break
        self.lock.release()
        return i
    # def updateQueue(self, new_queue):
    #     # type: (OrderedDict) -> None
    #     self.OrderedQueue = new_queue
    #     # print ("NQ: ",new_queue)
    #     next_da = next(iter(new_queue.items()))[1]
    #     # print ("NDA: ", next_da)
    #     # print ("NDA_ID: ", next_da["da_id"])
    #     if not self.isExecuting():
    #         # print("not executing")
    #         self.makeInterrupting(next_da["da_id"])
    #         if not self.switchIndicator.isSet():
    #             self.sendIndicator("normal")
    #     else:
    #         # print("executing")
    #         if next_da["priority"] > self.execField["priority"]:
    #             if not self.isInterrupting():
    #                 if self.debug ==True:
    #                     print("NO INTERRUPTING DA, ", next_da["da_id"], " is interrupting now")
    #                 self.makeInterrupting(next_da["da_id"])
    #                 if not self.switchIndicator.isSet():
    #                     self.sendIndicator("normal")

    #             elif next_da["priority"] > self.interruptField["priority"]:
    #                 # print("da: ", next_da["da_id"],"priority: ",next_da["priority"], "\n Replaces :", self.interruptField["da_id"], "with priority: ", self.interruptField["priority"])
    #                 self.makeInterrupting(next_da["da_id"])
    #                 if not self.switchIndicator.isSet():
    #                     self.sendIndicator("normal")
    def updateIrrField(self, next_da, switch_priority, cost_file):
    # print ("NDA: ", next_da)
    # print ("NDA_ID: ", next_da["da_id"])
        # debug = False
        logger.debug("\nIrrField:\n")
        logger.debug("\t Name: '{0}'\n".format(str(next_da.huid)))
        self.makeInterrupting(next_da.id)
        if not self.switchIndicator.isSet():
            logger.error("\t setting switch indicator")
            self.sendIndicator(switch_priority)

    def set_DA_signal(self, da_id, signal, data=[]):
        logger.debug ("set_DA_signal: ")
        self.updateDALastCMD(da_id, signal)
        cmd = CMD(recipient_name = da_id, cmd = signal, data = data)
        logger.debug (cmd)
        
        self.tasker_communicator.call_cmd(cmd, cmd.recipient_name)
        #self.cmd_pub.publish(cmd)

    def suspendDA(self, set_exemplary_susp_task = False):
        if set_exemplary_susp_task:
            self.set_DA_signal(da_id=self.execField, signal = "susp", data = ["rosrun", "TaskER", "exemplary_susp_task", "priority", "0"])
        else:
            self.set_DA_signal(da_id=self.execField, signal = "susp", data = [])
        while not rospy.is_shutdown():
            wait_flag = (self.isDAAlive_with_lock(self.execField) \
                        and (self.request_table.get_requst(self.execField).state[0]!="Wait"))
            if wait_flag:
                logger.info("Switch thread waits for exec_da to be WAIT or DEAD")
                rospy.Rate(5).sleep()
            else:
                self.lock.acquire()
                exe_da = self.execField 
                self.execField = None
                self.lock.release()
                break
        self.lock.acquire()
        logger.debug ("EXEDA: '{0}'".format(exe_da))
        self.updateDA(self.request_table.get_requst(exe_da))
        self.lock.release()

    def isDAAlive_no_lock(self, da):
        # print("checking DA: "+da["da_name"])
        if da.state == ['END']:
                # print ("THA -> DA <"+da["da_name"]+"> in END state")
            return False
        p = self.DA_processes[da.id]['thread']
        if not p.is_alive():
                return False        
        return True

    def isDAAlive_with_lock(self, da):
        logger.debug ("isDAAlive_with_lock: getting lock")
        self.lock.acquire()
        logger.debug ("isDAAlive_with_lock: Got lock")
            # print("checking DA: "+da["da_name"])
        if da.state == ['END']:
                # print ("THA -> DA <"+da["da_name"]+"> in END state")
            self.lock.release()
            return False
        p = self.DA_processes[da.id]['thread']
        if not p.is_alive() :
                return False                 
        self.lock.release()
        return True

    # def hasService(self, srv_name):
    #     service_list = rosservice.get_service_list()
    #     if srv_name in service_list:
    #         if self.debug ==True:
    #             print("\n\n\nHAVE SERVICE\n\n\n")
    #         return True
    #     else:
    #         if self.debug ==True:
    #             print("\n\n\nLOST SERVICE\n\n\n")
    #         return False

    # def schedule(self):

    #     # print("\nSCHEDULE\n")
    #     self.lock.acquire()
    #     if self.isExecuting():
    #         exec_da_name = "/"+self.execField["da_name"]
    #         # print("checking  EXEC node: ", exec_da_name)
    #         if not rosnode.rosnode_ping(exec_da_name, 1):
    #             print("FINIIIIIISSSSSHHHHHHEEEEDDDD")
    #             self.execField = {}
    #         else:
    #             self.execField["priority"] = self.execField["scheduleParams"].cost
    #     if self.isInterrupting():
    #         self.interruptField["priority"] = self.interruptField["scheduleParams"].cost 
    #         if self.isExecuting():
    #             self.interruptField["priority"] += self.interruptField["scheduleParams"].cost_per_sec * self.execField["scheduleParams"].completion_time
    #     # print (self.queue)
    #     for key_id in self.queue.items():
    #         # queued_da_name = "/"+self.queue[key_id[0]]["da_name"]

    #         # if not rosnode.rosnode_ping(queued_da_name, 1):
    #         #     print("DA terminated")
    #         #     del self.queue[key_id[0]]
    #         # else:
    #         self.queue[key_id[0]]["priority"] = self.queue[key_id[0]]["scheduleParams"].cost 
    #         if self.isExecuting():
    #             self.queue[key_id[0]]["priority"] += self.queue[key_id[0]]["scheduleParams"].cost_per_sec * self.execField["scheduleParams"].completion_time
    #     if len(self.queue) > 0:
    #         q = OrderedDict(sorted(self.queue.items(), 
    #                         key=lambda kv: kv[1]['priority'], reverse=True))
    #         # print ("Q: ",q)
    #         if self.isExecuting():
    #             highest_da = next(iter(q.items()))[1]
    #             if self.execField["priority"] < highest_da["priority"]:
    #                 print ("WAITING FOR CONDITIONS")
    #                 # rospy.wait_for_service('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions', timeout=2)
    #                 # get_susp_cond = rospy.ServiceProxy('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions', SuspendConditions)
    #                 trig = SuspendConditionsRequest()

    #                 resp = self.tasker_communicator.call_sus_cond(highest_da["scheduleParams"].final_resource_state)
    #                 # resp = get_susp_cond(highest_da["scheduleParams"].final_resource_state)
    #                 print ("HAVE CONDITIONS")
    #                 self.execField["priority"] = self.execField["scheduleParams"].cost + resp.cost_per_sec*highest_da["scheduleParams"].completion_time
    #         self.updateQueue(q)
    #     ###
    #     #  print queue and fields
    #     ###
    #     if not self.isExecuting():
    #         print ("No EXEC dynamic agent")
    #     else:
    #         if self.debug ==True:
    #             print ("\nEXEC: ")
    #             print ("ID: ", self.execField["da_id"])
    #             print ("Priority: ", self.execField["priority"])
    #             print ("SP: \n", self.execField["scheduleParams"], "\n")
    #     if not self.isInterrupting():
    #         if self.debug ==True:
    #             print ("No INTERRUPTING dynamic agent")
    #     else:
    #         if self.debug ==True:
    #             print ("\tINTERRUPT: ")
    #             print ("ID: ", self.interruptField["da_id"])
    #             print ("Priority: ", self.interruptField["priority"])
    #             print ("SP: \n", self.interruptField["scheduleParams"], "\n")
    #     if self.debug ==True:
    #         print ("\tQUEUE: ")
    #         for key_id in self.queue.items():
    #             print ("ID: ", self.queue[key_id[0]]["da_id"])
    #             print ("Priority: ", self.queue[key_id[0]]["priority"])
    #             print ("SP: \n", self.queue[key_id[0]]["scheduleParams"], "\n")

    #     self.lock.release()
    #     # print("\nSCHEDULED\n")

    def plan(self, plan_args):
        return None, timedelta(minutes=random.randint(2,10))

    def call_planner(self):
        for req in self.request_table.items():
            plan, burst_time = self.plan(req[1].plan_args)
            req[1].set_burst_time(burst_time)
    
    def schedule_smit(self, cost_file):
        for task in self.request_table.items():
            if task[1].state == ['END']:
                self.removeDA(task[1])
                # if self.queue.has_key(data.da_id):
                #     self.removeDA(self.queue[data.da_id])
                # else:
                #     logger.info("This DA: <'{0}'> is not found in THA table, probaly is already removed".format(str(data.da_name)))
        #
        # plan to get burst times for tke tasks
        #
        self.call_planner()
        self.lock.acquire()
        self.request_table.evaluate_all_rules()
        priority_schedule = self.request_table.schedule_with_priority()
        candidate = None
        for task in priority_schedule:
            if task.within(datetime.now()):
                candidate = task.jobID
                if self.request_table.get_requst(candidate).priority != -1:
                    break
                else:
                    candidate = None
        if candidate == None:
            self.lock.release()
            return
        logger.debug("Got candidate '{0}'".format(str(candidate)))
        if candidate != self.execField and candidate != self.interruptField:
            logger.debug("setting candidate '{0}'".format(str(candidate)))
            self.updateIrrField(self.request_table.get_requst(candidate), switch_priority='normal', cost_file=cost_file)
        self.lock.release()
            



    def filterDA_GH(self, DA):
        logger.debug ("IN FILTER: '{0}'".format(DA))
        if DA[1]["da_state"] == 'END':
            return False
        if DA[1]["da_type"] == "guide_human_tasker" and DA[1]["priority"] != float('-inf'):
            return True
        else:
            return False
    def filterDA_HF(self, DA):
        logger.debug ("IN FILTER: '{0}'".format(DA))
        if DA[1]["da_state"] == 'END':
            return False
        if DA[1]["da_type"] == "human_fell_tasker" and DA[1]["priority"] != float('-inf'):
            return True
        else:
            return False

    def filterDA_BJ(self, DA):
        logger.debug ("IN FILTER: '{0}'".format(DA))
        if DA[1]["da_state"] == 'END':
            return False
        if DA[1]["da_type"] == "bring_jar_tasker" and DA[1]["priority"] != float('-inf'):
            return True
        else:
            return False

    def filterDA_MT(self, DA):
        logger.debug ("IN FILTER: '{0}'".format(DA))
        if DA[1]["da_state"] == 'END':
            return False
        if DA[1]["da_type"] == "move_to_tasker" and DA[1]["priority"] != float('-inf'):
            return True
        else:
            return False

    def filterDA_BG(self, DA):
        logger.debug ("IN FILTER: '{0}'".format(DA))
        if DA[1]["da_state"] == 'END':
            return False
        if DA[1]["da_type"] == "bring_goods_tasker" and DA[1]["priority"] != float('-inf'):
            return True
        else:
            return False

    def schedule_new(self, cost_file):
        # print("\nSCHEDULE\n")
        self.lock.acquire()
        if len(self.queue) > 0:
            ordered_queue = OrderedDict(sorted(self.queue.items(), 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            logger.debug ("Queue before END check:\n'{0}'".format(ordered_queue))
        if self.isExecuting():
            if not self.isDAAlive_no_lock(self.execField):
                logger.debug ("THA-> removes DA: '{0}'".format(self.execField["da_name"]))
                self.removeDA_no_lock(self.execField)
        if self.isInterrupting():
            if not self.isDAAlive_no_lock(self.interruptField):
                logger.debug ("THA-> removes DA: '{0}'".format(self.interruptField["da_name"]))
                self.removeDA_no_lock(self.interruptField)
        for da in self.queue.items():
            if not self.isDAAlive_no_lock(da[1]):
                logger.debug ("THA-> removes DA: '{0}'".format(da[1]["da_name"]))
                self.removeDA_no_lock(da[1])

        # if self.isExecuting():
        #     exec_da_name = "/"+self.execField["da_name"]
        #     # print("checking  EXEC node: '{0}'".format(exec_da_name))
        #     if self.execField["da_state"] == 'END':
        #         self.removeDA_no_lock(self.execField)
        #         self.execField = {}
        #     if not rosnode.rosnode_ping(exec_da_name, 1):
        #         print("FINIIIIIISSSSSHHHHHHEEEEDDDD")
        #         self.execField = {}
        # if self.isInterrupting():
        #     irr_da_name = "/"+self.interruptField["da_name"]
        #     # print("checking  EXEC node: ", exec_da_name)
        #     if self.interruptField["da_state"] == 'END':
        #         self.removeDA_no_lock(self.interruptField)
        #         self.interruptField = {}
        #     if not rosnode.rosnode_ping(irr_da_name, 1):
        #         print("FINIIIIIISSSSSHHHHHHEEEEDDDD")
        #         self.interruptField = {}

        # for da in self.queue.items():
        #     if da[1]["da_state"] == 'END':
        #         self.removeDA_no_lock(da[1])
        #     if not rosnode.rosnode_ping("/"+da[1]["da_name"], 1):
        #         rospy.sleep(3)
        #         if not rosnode.rosnode_ping("/"+da[1]["da_name"], 1):
        #             print ("REMOVED:")
        #             print da[1]["da_name"]
        #             self.removeDA_no_lock(da[1])
        #         print ("NEXT DA")
        # print ("OUT OUT")
        if len(self.queue) > 0:
            ordered_queue = OrderedDict(sorted(self.queue.items(), 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            logger.debug ("OQ:\n'{0}'".format(ordered_queue))
            dac = next(iter(ordered_queue.items()))[1]
            DAset_GH = {}
            DAset_HF = {}
            DAset_BJ = {}
            DAset_MT = {}
            DAset_BG = {}
            cGH = {}
            cHF = {}
            cBJ = {}
            cMT = {}
            cBG = {}
            # print ("Q:")
            # print self.queue
            DAset_GH = filter(self.filterDA_GH, self.queue.items())
            DAset_HF = filter(self.filterDA_HF, self.queue.items())
            DAset_BJ = filter(self.filterDA_BJ, self.queue.items())
            DAset_MT = filter(self.filterDA_MT, self.queue.items())
            DAset_BG = filter(self.filterDA_BG, self.queue.items())
            # print ("DAset_GH:")
            # print DAset_GH
            # print ("DAset_T:")
            # print DAset_T
            # DAset_GH = {k: v for k, v in self.queue.tems() if "tiago_guideHuman" in v[1]["da_type"]}
            # DAset_T = {k: v for k, v in self.queue.iteritems() if "tiago_transport" in v[1]["da_type"]}
            q_GH = OrderedDict(sorted(DAset_GH, 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            q_HF = OrderedDict(sorted(DAset_HF, 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            q_BJ = OrderedDict(sorted(DAset_BJ, 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            q_MT = OrderedDict(sorted(DAset_MT, 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            q_BG = OrderedDict(sorted(DAset_BG, 
                            key=lambda kv: kv[1]["priority"], reverse=True))
            if self.debug_file == True:
                cost_file.write("\n"+"Q:\n")
                cost_file.write(str(self.queue)+"\n")

            if len(DAset_HF) > 0:
                logger.debug ("Have HF")
                # print ("q_GH")
                # print q_GH
                cHF = next(iter(q_HF.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cHF:"+"\n")
                    cost_file.write(str(cHF)+"\n")
                dac = cHF
                if self.isExecuting():
                    if not self.filterDA_HF([None,self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac,switch_priority,cost_file)
                        self.lock.release()
                        return
                # print ("cGH")
                # print cGH
            elif len(DAset_GH) > 0:
                logger.debug ("Have GH")
                cGH = next(iter(q_GH.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cGH:"+"\n")
                    cost_file.write(str(cGH)+"\n")
                dac = cGH 
                if self.isExecuting():
                    if self.filterDA_HF([None,self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_GH([None,self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac,switch_priority,cost_file)
                        self.lock.release()
                        return    
            elif len(DAset_BJ) > 0:
                logger.debug ("Have BJ")
                cBJ = next(iter(q_BJ.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cBJ:"+"\n")
                    cost_file.write(str(cBJ)+"\n")
                dac = cBJ 
                if self.isExecuting():
                    if self.filterDA_HF([None,self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_BJ([None,self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac,switch_priority,cost_file)
                        self.lock.release()
                        return    

            elif len(DAset_MT) > 0:
                logger.debug ("Have MT")
                cMT = next(iter(q_MT.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cMT:"+"\n")
                    cost_file.write(str(cMT)+"\n")
                dac = cMT 
                if self.isExecuting():
                    if self.filterDA_HF([None,self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_MT([None,self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac,switch_priority,cost_file)
                        self.lock.release()
                        return    

            elif len(DAset_BG) > 0:
                logger.debug ("Have BG")
                cBG = next(iter(q_BG.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cBG:"+"\n")
                    cost_file.write(str(cBG)+"\n")
                dac = cBG 
                if self.isExecuting():
                    if self.filterDA_HF([None,self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_BG([None,self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac,switch_priority,cost_file)
                        self.lock.release()
                        return    
            else:
                if self.isExecuting():
                    if not self.execField["da_type"] == dac["da_type"]:
                        print ("Executing a task of a type that has higher priority" )
                        self.lock.release()
                        return 
            # if not (len(DAset_GH) > 0 or len(DAset_HF) > 0):
            #     cost_file.write("\n"+"No candidate"+"\n")
            #     print ("No candidate")
            #     self.lock.release()
            #     return

            logger.debug ("dac: ", dac)
            if self.isExecuting() and not self.isInterrupting():
                logger.debug ("COMPARISION of TASKS of same type")
                logger.debug ("dac priority: '{0}'".format(dac["priority"]))
                logger.debug ("exe priority: '{0}'".format(self.execField["priority"]))
                if dac["priority"] > self.execField["priority"]:
                    logger.debug ("REQUESTING reqParam services")
                    # Exec cost for suspension behaviour and task continue starting from DAC final_resource_state -- cc_exec
                    #       , incremental (per sec) cost while waiting for start -- ccps_exec

                    # while (not rospy.is_shutdown()) and not self.hasService('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions'):
                    #     print ("Schedule thread waits for service: "+ '/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions')
                    #     rospy.sleep(0.1)
                    # rospy.wait_for_service('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions', timeout=2)
                    # get_susp_cond = rospy.ServiceProxy('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions', SuspendConditions)
                    # trig = SuspendConditionsRequest()
                    # resp = get_susp_cond(dac["scheduleParams"].final_resource_state)

                    resp = self.tasker_communicator.call_sus_cond(highest_da["scheduleParams"].final_resource_state, highest_da["da_name"])
                    cc_exec = resp.cost_to_resume
                    ccps_exec = resp.cost_per_sec
                    # DAC cost while switched after EXEC finishes -- cc_dac, incremental (per sec) cost while waiting for start -- ccps_dac
            #         ccps_exec = resp.cost_per_sec

                    # while (not rospy.is_shutdown()) and not self.hasService('/'+dac["da_name"]+'/TaskER/get_cost_on_conditions'):
                    #     print ("Schedule thread waits for service: "+ '/'+dac["da_name"]+'/TaskER/get_cost_on_conditions')
                    #     rospy.sleep(0.1)
                    # rospy.wait_for_service('/'+dac["da_name"]+'/TaskER/get_cost_on_conditions', timeout=2)
                    # get_cost_cond = rospy.ServiceProxy('/'+dac["da_name"]+'/TaskER/get_cost_on_conditions', CostConditions)
                    # trig = CostConditionsRequest()
                    # resp = get_cost_cond(self.execField["scheduleParams"].final_resource_state)

                    resp = self.tasker_communicator.call_cost_cond(self.execField["scheduleParams"].final_resource_state, self.execField["da_name"])


                    cc_dac = resp.cost_to_complete
                    ccps_dac = dac["scheduleParams"].cost_per_sec
                    # Calculation of costs to switch and not switch
                    c_switch = dac["scheduleParams"].cost + cc_exec + ccps_exec * dac["scheduleParams"].completion_time 
                    c_wait = self.execField["scheduleParams"].cost + cc_dac + ccps_dac * self.execField["scheduleParams"].completion_time 
                    # send schedule data for visualisation
                    shdl_data = ShdlDataStamped()
                    shdl_data.header.stamp = rospy.Time.now()
                    shdl_data.data.dac_cost = dac["scheduleParams"].cost
                    shdl_data.data.exec_cost = self.execField["scheduleParams"].cost
                    shdl_data.data.dac_cc = cc_dac
                    shdl_data.data.exec_cc = cc_exec
                    shdl_data.data.exec_ccps = ccps_exec
                    shdl_data.data.dac_ccps = ccps_dac
                    shdl_data.data.switch_cost = c_switch
                    shdl_data.data.wait_cost = c_wait
                    shdl_data.data.dac_id = dac["da_id"]
                    shdl_data.data.exec_id = self.execField["da_id"]
                    self.sdhl_pub.publish(shdl_data)
                    logger.debug ("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                    logger.debug ("SHDL_DATA: ")
                    logger.debug (shdl_data.data)
                    logger.debug ("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                    if (c_switch < (c_wait - c_wait*0.1)):
                        switch_priority = "normal"
                        logger.debug ("#################################################")
                        logger.debug ("SWITCH")
                        logger.debug ("#################################################")
                        self.updateIrrField(dac,switch_priority,cost_file)
                    else:
                        logger.debug ("candidate priority was less then executing task")
                    switch_priority = "normal"
                    self.updateIrrField(dac,switch_priority,cost_file)
            #     if  (self.execField["da_type"] == "tiago_humanFell") and cHF!={}:
            #         dac = cHF
            #         if debug == True:
            #             cost_file.write("\n"+"dac:"+"\n")
            #             cost_file.write(str(dac)+"\n")
            #     elif (self.execField["da_type"] == "tiago_humanFell") and cHF=={}:
            #         print ("Exec: hF, no hF in queue")
            #         if debug == True:
            #             cost_file.write("\n"+"Exec: hF, no hF in queue"+"\n")
            #     elif self.execField["da_type"] == "tiago_guideHuman":
            #         if len(DAset_HF) > 0:
            #             cost_file.write("\n"+"GH executing, there is HF in Q"+"\n")
            #             self.updateIrrField(cHF,cost_file)
            #             self.lock.release()
            #             return
            #         elif len(DAset_GH) > 0:
            #             dac = cGH
            #             if debug == True:
            #                 cost_file.write("\n"+"dac:"+"\n")
            #                 cost_file.write(str(dac)+"\n")
            #         else:
            #             cost_file.write("\n"+"No candidate"+"\n")
            #             print ("No candidate")
            #     else:
            #         cost_file.write("\n"+"DA in ExecField has unknown type task"+"\n")
            #     if dac == {}:
            #         cost_file.write("\n"+"No candidate"+"\n")
            #         print ("No candidate")
            #     else:
            #         cost_file.write("\n"+"Exec cost:"+"\n")
            #         cost_file.write("\t"+str(self.execField["scheduleParams"].cost)+"\n")
            #         cost_file.write("DAC cost:"+"\n")
            #         cost_file.write("\t"+str(dac["da_name"])+": "+str(dac["scheduleParams"].cost)+"\n")
            #         # if dac["scheduleParams"].cost < self.execField["scheduleParams"].cost:
            #         cost_file.write("\n"+"Have candidate"+"\n")
            #         cost_file.write("CHECK pair combinations"+"\n")
            #         print ("WAITING FOR SUSPEND COST from exec")
            #         exec_da_name = "/"+self.execField["da_name"]
            #         if not rosnode.rosnode_ping(exec_da_name, 1):
            #             print("EXEC FINIIIIIISSSSSHHHHHHEEEEDDDD")
            #             self.execField = {}
            #             self.lock.release()
            #             return
            #         rospy.wait_for_service('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions')
            #         get_susp_cond = rospy.ServiceProxy('/'+self.execField["da_name"]+'/TaskER/get_suspend_conditions', SuspendConditions)
            #         trig = SuspendConditionsRequest()
            #         resp = get_susp_cond(dac["scheduleParams"].final_resource_state)
            #         cost_file.write("\n"+"EXEC:"+"\n")
            #         cc_exec = resp.cost_to_resume
            #         cost_file.write("\tcc:"+"\n")
            #         cost_file.write(str(cc_exec)+"\n")
            #         ccps_exec = resp.cost_per_sec
            #         cost_file.write("\tccps:"+"\n")
            #         cost_file.write(str(ccps_exec)+"\n")

            #         print ("WAITING FOR COST from candidate")
            #         dac_name = "/"+dac["da_name"]
            #         if not rosnode.rosnode_ping(dac_name, 1):
            #             rospy.sleep(3)
            #             if not rosnode.rosnode_ping(dac_name, 1):
            #                 print ("REMOVED:")
            #                 print dac["da_name"]
            #                 self.removeDA(dac)
            #                 self.lock.release()
            #                 return
            #         rospy.wait_for_service('/'+dac["da_name"]+'/TaskER/get_cost_on_conditions')
            #         get_cost_cond = rospy.ServiceProxy('/'+dac["da_name"]+'/TaskER/get_cost_on_conditions', CostConditions)
            #         trig = CostConditionsRequest()
            #         resp = get_cost_cond(self.execField["scheduleParams"].final_resource_state)
            #         cc_dac = resp.cost_to_complete
            #         cost_file.write("\n"+"DAC:"+"\n")
            #         cost_file.write("\tcc:"+"\n")
            #         cost_file.write(str(cc_dac)+"\n")
            #         ccps_dac = dac["scheduleParams"].cost_per_sec
            #         cost_file.write("\tccps:"+"\n")
            #         cost_file.write(str(ccps_dac)+"\n")
            #         cost_file.write("\n"+"COMBINATION:"+"\n")
            #         c_switch = dac["scheduleParams"].cost + cc_exec + ccps_exec * dac["scheduleParams"].completion_time 
            #         c_wait = self.execField["scheduleParams"].cost + cc_dac + ccps_dac * self.execField["scheduleParams"].completion_time 
            #         cost_file.write("\tc_switch:"+"\n")
            #         cost_file.write(str(c_switch)+"\n")
            #         cost_file.write("\tc_wait:"+"\n")
            #         cost_file.write(str(c_wait)+"\n")

            #         shdl_data = ShdlDataStamped()
            #         shdl_data.header.stamp = rospy.Time.now()
            #         shdl_data.data.dac_cost = dac["scheduleParams"].cost
            #         shdl_data.data.exec_cost = self.execField["scheduleParams"].cost
            #         shdl_data.data.dac_cc = cc_dac
            #         shdl_data.data.exec_cc = cc_exec
            #         shdl_data.data.exec_ccps = ccps_exec
            #         shdl_data.data.dac_ccps = ccps_dac
            #         shdl_data.data.switch_cost = c_switch
            #         shdl_data.data.wait_cost = c_wait
            #         shdl_data.data.dac_id = dac["da_id"]
            #         shdl_data.data.exec_id = self.execField["da_id"]
            #         self.sdhl_pub.publish(shdl_data)


            #         if (c_switch < (c_wait - c_wait*0.1)):
            #             cost_file.write("\n"+"SWITCH SWITCH SWITCH SWITCH "+"\n")
            #             self.updateIrrField(dac,cost_file)
            #         # else:
            #         #     cost_file.write("\n"+"DAC < EXEC cost:"+"\n")
            #         #     print ("candidate priority was less then executing task")
            elif not self.isExecuting() and not self.isInterrupting():
                logger.debug ("No EXEC, NO IRR dynamic agent")
                # if len(DAset_HF) > 0:
                #     print ("HF len > 0")
                #     self.updateIrrField(cHF,cost_file)
                # elif len(DAset_GH) > 0:
                #     print ("GH len > 0")
                #     self.updateIrrField(cGH,cost_file)
                if dac != None:
                    switch_priority = "normal"
                    self.updateIrrField(dac,switch_priority,cost_file)
                else:
                    logger.info ("No candidate")
            else:
                logger.info ("Processing switch")

        if not self.isExecuting():
            logger.info ("No EXEC dynamic agent")
        else:
            logger.debug ("\nEXEC: ")
            logger.debug ("ID: '{0}'".format(self.execField["da_id"]))
            logger.debug ("Cost: '{0}'".format(self.execField["scheduleParams"].cost))
            logger.debug ("SP: \n'{0}'".format(self.execField["scheduleParams"], "\n"))
        if not self.isInterrupting():
            logger.debug ("No INTERRUPTING dynamic agent")
        else:
            logger.debug ("\tINTERRUPT: ")
            logger.debug ("ID: '{0}'".format(self.interruptField["da_id"]))
            logger.debug ("Cost: '{0}'".format(self.interruptField["scheduleParams"].cost))
            logger.debug ("SP: \n'{0}'".format(self.interruptField["scheduleParams"], "\n"))
        logger.debug ("\tQUEUE: ")
        for key_id in self.queue.items():
            logger.debug ("ID: '{0}'".format(self.queue[key_id[0]]["da_id"]))
            logger.debug ("Cost: '{0}'".format(self.queue[key_id[0]]["scheduleParams"].cost))
            logger.debug ("SP: \n'{0}'".format(self.queue[key_id[0]]["scheduleParams"], "\n"))

        self.lock.release()
        # print("\nSCHEDULED\n")

    def switchDA(self):
        r = rospy.Rate(1)
        self.switchIndicator.wait()
        if not self.tha_is_running:
            return
        logger.error("\t GOT switch indicator")
        if self.isInterrupting():
            logger.error("\nSWITCHING\n")
            self.lock.acquire()
            logger.error("\nSWITCHING LOCKED\n")
            if self.isExecuting():
                commanding_da = self.request_table.get_requst(self.execField)
                self.lock.release()
                max_sleep_counter = 6
                sleep_counter = 0
                while self.getDALastCMD(commanding_da.id) in ['start','resume'] \
                        and commanding_da.state[0] in ["Wait", "Initialise", "UpdateTask"]:
                    logger.debug ("[Switch] -- while last cmd")
                    sleep_counter = sleep_counter + 1
                    # while commanding DA stays in ["Wait", "Initialise", "UpdateTask"] longer then max_sleep_counter after ['start','resume'] signal,
                    # terminate the commanding DA
                    if max_sleep_counter == sleep_counter:
                        self.set_DA_signal(da_id=commanding_da.id, signal = "terminate", data = ["priority", self._switch_priority,
                                                            #THA może zarządać odpalenia konkretnej sekwencji wstrzymania np. ("rosrun", "TaskER", "exemplary_susp_task"),
                                                            ])
                        while not rospy.is_shutdown():
                            logger.debug ("[Switch] -- while sleep counter")
                            wait_flag = self.isDAAlive_with_lock(commanding_da)
                            if wait_flag:
                                logger.error("Waiting for DA: '{0}' to terminate after long processing of ['start','resume'] command, and new interruption is comming".format(commanding_da.id))
                                rospy.Rate(5).sleep()
                            else:
                                # Exec DA is terminated, remove it from Exec field
                                self.lock.acquire()
                                self.execField = None
                                self.lock.release()
                                break
                        break
                    r.sleep()
                    
                    
                logger.error("\nisDAAlive_with_lock\n")
                if self.isDAAlive_with_lock(commanding_da):
                    logger.error("SEND SUSPEND to commanding: '{0}'".format(commanding_da.id))
                    self.set_DA_signal(da_id=commanding_da.id, signal = "susp", data = ["priority", self._switch_priority,
                                                            #THA może zarządać odpalenia konkretnej sekwencji wstrzymania np. ("rosrun", "TaskER", "exemplary_susp_task"),
                                                            ])

                     # 10hz
                    # wait until exec DA terminates or swithes to wait state
                    while not rospy.is_shutdown():
                        wait_flag = (self.isDAAlive_with_lock(commanding_da) \
                                and (commanding_da.state[0]!="Wait"))
                        if wait_flag:
                            logger.error("Switch thread waits for exec_da to be WAIT or DEAD")
                            r.sleep()
                        else:
                            if not self.isDAAlive_with_lock(commanding_da):
                                # Exec DA is terminated, remove it from Exec field
                                self.lock.acquire()
                                self.execField = None
                                self.lock.release()
                            break
            else:
                logger.error("\nRELEASE\n")
                self.lock.release()
            logger.error("\nSWITCHING getting lock\n")
            self.lock.acquire()
            logger.error("\nSWITCHING got lock\n")
            interrupting = self.interruptField
            self.lock.release()
            logger.error("SEND StartTask to initialised: '{0}'".format(interrupting))
            # srv_name = "/"+interrupting["da_name"]+"/TaskER/startTask"
            # print (srv_name)
            # print("\nSWITCHING: waiting for QUEUED startTask\n")
            # rospy.wait_for_service(srv_name, timeout=2)
            logger.error ("\nSWITCHING: waiting for QUEUED to be in Initialise or Wait. It is in <"'{0}' "> state.".format(
                        self.request_table.get_requst(interrupting).state[0]))
            while not self.request_table.get_requst(interrupting).state[0] in ["Wait", "Initialise"]:
                if rospy.is_shutdown():
                    return 
                logger.error ("\nSWITCHING: waiting for QUEUED to be in Initialise or Wait. It is in <"'{0}' "> state.".format(
                        self.request_table.get_requst(interrupting).state[0]))
                r.sleep()

            if self.request_table.get_requst(interrupting).state[0]=="Wait":
                self.set_DA_signal(da_id=interrupting, signal = "resume", data = [])
            elif self.request_table.get_requst(interrupting).state[0]=="Initialise":
                self.set_DA_signal(da_id=interrupting, signal = "start", data = [])

            # print("\nSWITCHING: waiting for STARTED hold_now\n")

            # rospy.wait_for_service(srv_name, timeout=2)
            logger.error ("\nSWITCHING: waiting for STARTED '{0}'to be in UpdateTask or ExecFSM. It is in <"'{1}' "> state.".format(
                            interrupting, self.request_table.get_requst(interrupting).state[0]))
            while not self.request_table.get_requst(interrupting).state[0] in ["UpdateTask","ExecFSM"]:
                if rospy.is_shutdown():
                    return 
                logger.error ("\nSWITCHING: waiting for STARTED '{0}'to be in UpdateTask or ExecFSM. It is in <"'{1}' "> state.".format(
                            interrupting, self.request_table.get_requst(interrupting).state[0]))
                rospy.sleep(0.5)

            # rospy.wait_for_service('/'+interrupting["da_name"]+'/TaskER/hold_now', timeout=2)
            self.lock.acquire()
            # print("\n Making executing\n")
            self.makeExecuting()
            self.switchIndicator.clear()
            # print("\n Made executing\n")
            self.lock.release()
            logger.error("\nSWITCHED\n")
        else:
            logger.error ("[TH] -- Killing switch thread")
        # self.isInterrupting = isInterrupting
        # self.print_log = file
        # self.handlers = {}
        # self.startState = None
        # self.endStates = []
        # self.facultativeStates = []
        # self.priority = 0
        # self.start_deadline = -1
        # node_namespace = rospy.get_name() + "/TaskER"
        # srv_name = rospy.get_name()+'/TaskER/get_hold_conditions'
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
    def shutdown(self):
        self.tha_is_running =False
        self.switchIndicator.set()
