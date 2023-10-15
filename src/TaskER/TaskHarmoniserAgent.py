#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import OrderedDict
from tasker_msgs.msg import *
from tasker_msgs.srv import *
from tasker_comm import THACommunicator
import threading
import time
import subprocess
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.rate import Rate
from rclpy.clock import Clock, ClockType


class TaskHarmoniserAgent(Node):

    def __init__(self):
        super().__init__('task_harmoniser_agent')

        self.switchIndicator = threading.Event()
        self.switchIndicator.clear()
        self.init_da = {
            'da_id': -1, 'da_name': None, 'da_type': None,
            'da_state': None, 'priority': float('-inf'),
            'scheduleParams': ScheduleParams()
        }
        self.lock = threading.Lock()
        self.queue = {}
        self.tasker_communicator = THACommunicator()
        self.sdhl_pub = self.create_publisher(
            ShdlDataStamped, "/TH/shdl_data", 10)
        # self.cmd_pub = self.create_publisher(CMD, "/TH/cmd", 10)
        self.OrderedQueue = {}
        self.execField = {}
        self.interruptField = {}
        self.tha_is_running = True

        print("starting cmd updateQueueDataThread thread")
        self.thread_status_update = threading.Thread(
            target=self.updateQueueDataThread, args=(1,))
        self.thread_status_update.start()
        print("started cmd updateQueueDataThread thread")

        # self.sub_status = self.create_subscription(Status, "TH/statuses", self.updateQueueDataThread, 10)
        self.DA_processes = {}
        self._switch_priority = None
        self.debug = True
        self.debug_file = False
        self.clock = Clock(clock_type=ClockType.ROS_TIME)

    def __del__(self):
        self.tasker_communicator.close()
        del self.tasker_communicator
        print("\n\n\n\n DEL THA \n\n\n\n")
        self.tha_is_running = False
        self.thread_status_update.join()

    def close(self):
        self.tasker_communicator.close()

    def updateQueueDataThread(self, args):
        while rclpy.ok() and self.tha_is_running:
            msg = self.tasker_communicator.sub_status()
            self.updateQueueData(msg)

    def updateQueueData(self, data):
        if self.debug:
            print(
                f"\nUPDATE SP\nupdateQueueData: {data.da_name}, state: {data.da_state}\nSP: \n{data.schedule_params}")
        if data.da_state == 'END':
            if data.da_id in self.queue:
                self.removeDA(self.queue[data.da_id])
            else:
                raise Exception(
                    f'This DA: <{data.da_name}> is not found in THA queue')
        self.updateScheduleParams(data.da_id, data.schedule_params)
        priority = self.computePriority(data.schedule_params)
        self.updatePriority(data.da_id, priority)
        self.updateDAState(data.da_id, data.da_state)
        if self.debug:
            print("\nUPDATED SP\n")

    def initialiseDA(self, executable, da_type, da_id, args):
        da_name = f"DA_{da_id}"
        args.extend(['da_id', str(da_id), 'da_name',
                    da_name, 'da_type', da_type])
        print(f'args: {args}')
        run_cmd = [executable]
        run_cmd.extend(args)
        print(f"cmd: {run_cmd}")
        p = subprocess.Popen(run_cmd)
        row = {'da_id': da_id, 'process': p}
        self.DA_processes[da_id] = row
        self.addDA(da_id, da_name, da_type)

    def addDA(self, added, da_name, da_type):
        self.lock.acquire()
        da = {'da_id': added, 'da_name': da_name, 'da_type': da_type, 'da_state': [
            "Init"], 'last_cmd_sent': None, 'priority': float('-inf'), 'ping_count': 0, 'scheduleParams': ScheduleParams()}
        self.queue[added] = da
        self.lock.release()

    def updatePriority(self, da_id, priority):
        self.lock.acquire()
        if da_id in self.queue:
            self.queue[da_id]["priority"] = float(priority)
        else:
            print(
                f"[TH] - tried to update Priority of DA_{da_id} but there is no such DA")
        self.lock.release()

    def computePriority(self, schedule_params):
        return -1 * schedule_params.cost

    def updateScheduleParams(self, da_id, scheduleParams):
        self.lock.acquire()
        if da_id in self.queue:
            self.queue[da_id]["scheduleParams"] = scheduleParams
        else:
            print(
                f"[TH] - tried to update Schedule Params of DA_{da_id} but there is no such DA")
        self.lock.release()

    def updateDAState(self, da_id, da_state):
        self.lock.acquire()
        if da_id in self.queue:
            self.queue[da_id]["da_state"] = da_state
        else:
            print(
                f"[TH] - tried to update STATE of DA_{da_id} but there is no such DA")
        self.lock.release()

    def updateDALastCMD(self, da_name, cmd):
        self.lock.acquire()
        if self.isExecuting():
            if self.execField["da_name"] == da_name:
                self.execField["last_cmd_sent"] = cmd
                self.lock.release()
                return
        if self.is_interrupting():
            if self.interruptField["da_name"] == da_name:
                self.interruptField["last_cmd_sent"] = cmd
                self.lock.release()
                return
        if da_name in self.queue:
            self.queue[da_name]["last_cmd_sent"] = cmd
        else:
            print(
                f"[TH] - tried to update last CMD of {da_name} but there is no such DA")
        self.lock.release()

    def getDALastCMD(self, da_name):
        self.lock.acquire()
        if self.isExecuting():
            if self.execField["da_name"] == da_name:
                self.lock.release()
                return self.execField["last_cmd_sent"]
        if self.is_interrupting():
            if self.interruptField["da_name"] == da_name:
                self.lock.release()
                return self.interruptField["last_cmd_sent"]
        if da_name in self.queue:
            self.lock.release()
            return self.queue[da_name]["last_cmd_sent"]
        else:
            print(
                f"[TH] - tried to get last CMD of {da_name} but there is no such DA")
        self.lock.release()

    def makeInterrupting(self, da_id):
        if self.is_interrupting():
            self.updateDA(self.interruptField)
        self.interruptField = self.queue[da_id]
        del self.queue[da_id]

    def makeExecuting(self):
        if self.isExecuting():
            self.updateDA(self.execField)
        self.execField = self.interruptField
        self.interruptField = {}

    def is_interrupting(self):
        return len(self.interruptField) != 0

    def isExecuting(self):
        return len(self.execField) != 0

    def sendIndicator(self, switch_priority):
        self._switch_priority = switch_priority
        self.switchIndicator.set()

    def getInterruptingAndExecuting(self):
        return [self.interruptField, self.execField]

    def getQueue(self):
        queue = self.queue
        return queue

    def getNextID(self):
        self.lock.acquire()
        i = 0
        while True:
            for key_id in self.queue.items():
                if self.queue[key_id[0]]["da_id"] == i:
                    i = i + 1
                    continue
            if self.isExecuting():
                if self.execField["da_id"] == i:
                    i = i + 1
                    continue
            if self.is_interrupting():
                if self.interruptField["da_id"] == i:
                    i = i + 1
                    continue
            break
        self.lock.release()
        return i

    def updateQueue(self, new_queue):
        self.OrderedQueue = new_queue
        next_da = next(iter(new_queue.items()))[1]
        if not self.isExecuting():
            self.makeInterrupting(next_da["da_id"])
            if not self.switchIndicator.isSet():
                self.sendIndicator("normal")
        else:
            if next_da["priority"] > self.execField["priority"]:
                if not self.is_interrupting():
                    if self.debug:
                        print(
                            f"NO INTERRUPTING DA, {next_da['da_id']} is interrupting now")
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.sendIndicator("normal")

                elif next_da["priority"] > self.interruptField["priority"]:
                    self.makeInterrupting(next_da["da_id"])
                    if not self.switchIndicator.isSet():
                        self.sendIndicator("normal")

    def updateIrrField(self, next_da, switch_priority, cost_file):
        if self.debug_file:
            cost_file.write("\n"+"IrrField:"+"\n")
            cost_file.write(str(next_da)+"\n")
        else:
            if self.debug:
                print("\n"+"IrrField:"+"\n")
                print(f"\t Name: {next_da['da_name']}\n")
        self.makeInterrupting(next_da["da_id"])
        if not self.switchIndicator.isSet():
            self.sendIndicator(switch_priority)

    def set_DA_signal(self, da_name, signal, data=[]):
        if self.debug:
            print("set_DA_signal:")
        self.updateDALastCMD(da_name, signal)
        cmd = CMD()  # Assuming CMD is a custom ROS2 msg
        cmd.recipient_name = da_name
        cmd.cmd = signal
        cmd.data = data  # Ensure data is a list field in the ROS2 message definition
        if self.debug:
            print(cmd)
        # self.tasker_communicator.pub_cmd(cmd)

    def suspendDA(self, set_exemplary_susp_task=False):
        if set_exemplary_susp_task:
            self.set_DA_signal(da_name=self.execField["da_name"], signal="susp", data=[
                               "rosrun", "TaskER", "exemplary_susp_task", "priority", "0"])
        else:
            self.set_DA_signal(
                da_name=self.execField["da_name"], signal="susp", data=[])

        while rclpy.ok():
            wait_flag = (self.isDAAlive_with_lock(self.execField)
                         and (self.execField["da_state"][0] != "Wait"))
            if wait_flag:
                print("Switch thread waits for exec_da to be WAIT or DEAD")
                Rate(5).sleep()
            else:
                self.lock.acquire()
                exe_da = self.execField
                self.execField = {}
                self.lock.release()
                break

        self.lock.acquire()
        print("EXEDA: ", exe_da)
        self.updateDA(exe_da)
        self.lock.release()

    def isDAAlive_no_lock(self, da):
        if da["da_state"] == ['END'] and self.debug:
            return False
        p = self.DA_processes[da["da_id"]]['process']
        return p.poll() is None

    def isDAAlive_with_lock(self, da):
        self.lock.acquire()
        alive = self.isDAAlive_no_lock(da)
        self.lock.release()
        return alive

    def hasService(self, srv_name):
        service_list = rosservice.get_service_list()
        has_srv = srv_name in service_list
        if self.debug:
            print("\n\n\nHAVE SERVICE\n\n\n" if has_srv else "\n\n\nLOST SERVICE\n\n\n")
        return has_srv

    def schedule(self):
        self.lock.acquire()
        # ... [rest of the method's content as provided] ...

        self.lock.release()

    def filterDA_GH(self, DA):
        if DA[1]["da_state"] == 'END':
            return False
        return DA[1]["da_type"] == "guide_human_tasker" and DA[1]["priority"] != float('-inf')

    def filterDA_HF(self, DA):
        if DA[1]["da_state"] == 'END':
            return False
        return DA[1]["da_type"] == "human_fell_tasker" and DA[1]["priority"] != float('-inf')

    def filterDA_BJ(self, DA):
        if DA[1]["da_state"] == 'END':
            return False
        return DA[1]["da_type"] == "bring_jar_tasker" and DA[1]["priority"] != float('-inf')

    def filterDA_MT(self, DA):
        if DA[1]["da_state"] == 'END':
            return False
        return DA[1]["da_type"] == "move_to_tasker" and DA[1]["priority"] != float('-inf')

    def filterDA_BG(self, DA):
        if DA[1]["da_state"] == 'END':
            return False
        return DA[1]["da_type"] == "bring_goods_tasker" and DA[1]["priority"] != float('-inf')

    def schedule_new(self, cost_file):
        # print("\nSCHEDULE\n")
        self.lock.acquire()
        if len(self.queue) > 0:
            ordered_queue = OrderedDict(sorted(self.queue.items(),
                                               key=lambda kv: kv[1]["priority"], reverse=True))
            if self.debug == True:
                print("Queue before END check:\n", ordered_queue)
        if self.isExecuting():
            if not self.isDAAlive_no_lock(self.execField):
                if self.debug == True:
                    print("THA-> removes DA: ", self.execField["da_name"])
                self.removeDA_no_lock(self.execField)
        if self.is_interrupting():
            if not self.isDAAlive_no_lock(self.interruptField):
                if self.debug == True:
                    print("THA-> removes DA: ", self.interruptField["da_name"])
                self.removeDA_no_lock(self.interruptField)
        for da in self.queue.items():
            if not self.isDAAlive_no_lock(da[1]):
                if self.debug == True:
                    print("THA-> removes DA: ", da[1]["da_name"])
                self.removeDA_no_lock(da[1])

        if len(self.queue) > 0:
            ordered_queue = OrderedDict(sorted(self.queue.items(),
                                               key=lambda kv: kv[1]["priority"], reverse=True))
            if self.debug == True:
                print("OQ:\n", ordered_queue)
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

            DAset_GH = filter(self.filterDA_GH, self.queue.items())
            DAset_HF = filter(self.filterDA_HF, self.queue.items())
            DAset_BJ = filter(self.filterDA_BJ, self.queue.items())
            DAset_MT = filter(self.filterDA_MT, self.queue.items())
            DAset_BG = filter(self.filterDA_BG, self.queue.items())
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
                if self.debug == True:
                    print("Have HF")
                # print "q_GH"
                # print q_GH
                cHF = next(iter(q_HF.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cHF:"+"\n")
                    cost_file.write(str(cHF)+"\n")
                dac = cHF
                if self.isExecuting():
                    if not self.filterDA_HF([None, self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac, switch_priority, cost_file)
                        self.lock.release()
                        return
                # print "cGH"
                # print cGH
            elif len(DAset_GH) > 0:
                if self.debug == True:
                    print("Have GH")
                cGH = next(iter(q_GH.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cGH:"+"\n")
                    cost_file.write(str(cGH)+"\n")
                dac = cGH
                if self.isExecuting():
                    if self.filterDA_HF([None, self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_GH([None, self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac, switch_priority, cost_file)
                        self.lock.release()
                        return
            elif len(DAset_BJ) > 0:
                if self.debug == True:
                    print("Have BJ")
                cBJ = next(iter(q_BJ.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cBJ:"+"\n")
                    cost_file.write(str(cBJ)+"\n")
                dac = cBJ
                if self.isExecuting():
                    if self.filterDA_HF([None, self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_BJ([None, self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac, switch_priority, cost_file)
                        self.lock.release()
                        return

            elif len(DAset_MT) > 0:
                if self.debug == True:
                    print("Have MT")
                cMT = next(iter(q_MT.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cMT:"+"\n")
                    cost_file.write(str(cMT)+"\n")
                dac = cMT
                if self.isExecuting():
                    if self.filterDA_HF([None, self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_MT([None, self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac, switch_priority, cost_file)
                        self.lock.release()
                        return

            elif len(DAset_BG) > 0:
                if self.debug == True:
                    print("Have BG")
                cBG = next(iter(q_BG.items()))[1]
                if self.debug_file == True:
                    cost_file.write("\n"+"cBG:"+"\n")
                    cost_file.write(str(cBG)+"\n")
                dac = cBG
                if self.isExecuting():
                    if self.filterDA_HF([None, self.execField]):
                        self.lock.release()
                        return
                    elif not self.filterDA_BG([None, self.execField]):
                        switch_priority = "normal"
                        self.updateIrrField(dac, switch_priority, cost_file)
                        self.lock.release()
                        return
            else:
                if self.isExecuting():
                    if not self.execField["da_type"] == dac["da_type"]:
                        print("Executing a task of a type that has higher priority")
                        self.lock.release()
                        return
            # if not (len(DAset_GH) > 0 or len(DAset_HF) > 0):
            #     cost_file.write("\n"+"No candidate"+"\n")
            #     print "No candidate"
            #     self.lock.release()
            #     return

            if self.debug == True:
                print("dac: ", dac)
            if self.isExecuting() and not self.is_interrupting():
                print("COMPARISION of TASKS of same type")
                if self.debug == True:
                    print("dac priority: ", dac["priority"])
                    print("exe priority: ", self.execField["priority"])
                if dac["priority"] > self.execField["priority"]:
                    if self.debug == True:
                        print("REQUESTING reqParam services")

                    resp = self.tasker_communicator.call_sus_cond(
                        highest_da["scheduleParams"].final_resource_state, highest_da["da_name"])
                    cc_exec = resp.cost_to_resume
                    ccps_exec = resp.cost_per_sec

                    resp = self.tasker_communicator.call_cost_cond(
                        self.execField["scheduleParams"].final_resource_state, self.execField["da_name"])

                    cc_dac = resp.cost_to_complete
                    ccps_dac = dac["scheduleParams"].cost_per_sec
                    # Calculation of costs to switch and not switch
                    c_switch = dac["scheduleParams"].cost + cc_exec + \
                        ccps_exec * dac["scheduleParams"].completion_time
                    c_wait = self.execField["scheduleParams"].cost + cc_dac + \
                        ccps_dac * \
                        self.execField["scheduleParams"].completion_time
                    # send schedule data for visualisation
                    shdl_data = ShdlDataStamped()
                    shdl_data.header.stamp = self.clock.now().to_msg()
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
                    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                    print("SHDL_DATA: ")
                    print(shdl_data.data)
                    print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
                    if (c_switch < (c_wait - c_wait*0.1)):
                        switch_priority = "normal"
                        print("#################################################")
                        print("SWITCH")
                        print("#################################################")
                        self.updateIrrField(dac, switch_priority, cost_file)
                    else:
                        print("candidate priority was less then executing task")
                    switch_priority = "normal"
                    self.updateIrrField(dac, switch_priority, cost_file)
            elif not self.isExecuting() and not self.is_interrupting():
                print("No EXEC, NO IRR dynamic agent")
                if dac != None:
                    switch_priority = "normal"
                    self.updateIrrField(dac, switch_priority, cost_file)
                else:
                    print("No candidate")
            else:
                print("Processing switch")

        if not self.isExecuting():
            print("No EXEC dynamic agent")
        else:
            if self.debug == True:
                print("\nEXEC: ")
                print("ID: ", self.execField["da_id"])
                print("Cost: ", self.execField["scheduleParams"].cost)
                print("SP: \n", self.execField["scheduleParams"], "\n")
        if not self.is_interrupting():
            if self.debug == True:
                print("No INTERRUPTING dynamic agent")
        else:
            if self.debug == True:
                print("\tINTERRUPT: ")
                print("ID: ", self.interruptField["da_id"])
                print("Cost: ", self.interruptField["scheduleParams"].cost)
                print("SP: \n", self.interruptField["scheduleParams"], "\n")
        if self.debug == True:
            print("\tQUEUE: ")
            for key_id in self.queue.items():
                print("ID: ", self.queue[key_id[0]]["da_id"])
                print("Cost: ", self.queue[key_id[0]]["scheduleParams"].cost)
                print("SP: \n", self.queue[key_id[0]]["scheduleParams"], "\n")

        self.lock.release()
        # print("\nSCHEDULED\n")

    def switchDA(self):
        r = Rate(5)
        self.switchIndicator.wait()
        if self.is_interrupting():
            print("\nSWITCHING\n")
            self.lock.acquire()
            if self.isExecuting():
                commanding = self.execField
                self.lock.release()
                max_sleep_counter = 6
                sleep_counter = 0
                while self.getDALastCMD(commanding["da_name"]) in ['start', 'resume'] and commanding["da_state"][0] in ["Wait", "Init", "UpdateTask"]:
                    print("[Switch] -- while last cmd")
                    sleep_counter = sleep_counter + 1
                    # while commanding DA stays in ["Wait", "init", "UpdateTask"] longer then max_sleep_counter after ['start','resume'] signal,
                    # terminate the commanding DA
                    if max_sleep_counter == sleep_counter:
                        self.set_DA_signal(da_name=commanding["da_name"], signal="terminate", data=["priority", self._switch_priority,
                                                                                                    # THA może zarządać odpalenia konkretnej sekwencji wstrzymania np. ("rosrun", "TaskER", "exemplary_susp_task"),
                                                                                                    ])
                        while rclpy.ok():
                            print("[Switch] -- while sleep counter")
                            wait_flag = self.isDAAlive_with_lock(commanding)
                            if wait_flag:
                                print(
                                    "Waiting for DA: ", commanding["da_name"], " to terminate after long processing of ['start','resume'] command, and new interruption is comming")
                                Rate(5).sleep()
                            else:
                                # Exec DA is terminated, remove it from Exec field
                                self.lock.acquire()
                                self.execField = {}
                                self.lock.release()
                                break
                        break
                    r.sleep()

                if self.isDAAlive_with_lock(commanding):
                    print("SEND SUSPEND to commanding: ", commanding["da_id"])
                    self.set_DA_signal(da_name=commanding["da_name"], signal="susp", data=["priority", self._switch_priority,
                                                                                           # THA może zarządać odpalenia konkretnej sekwencji wstrzymania np. ("rosrun", "TaskER", "exemplary_susp_task"),
                                                                                           ])

                    # 10hz
                    # wait until exec DA terminates or swithes to wait state
                    while rclpy.ok():
                        wait_flag = (self.isDAAlive_with_lock(commanding) and (
                            commanding["da_state"][0] != "Wait"))
                        if wait_flag:
                            print(
                                "Switch thread waits for exec_da to be WAIT or DEAD")
                            r.sleep()
                        else:
                            if not self.isDAAlive_with_lock(commanding):
                                # Exec DA is terminated, remove it from Exec field
                                self.lock.acquire()
                                self.execField = {}
                                self.lock.release()
                            break
            else:
                self.lock.release()
            self.lock.acquire()
            interrupting = self.interruptField
            self.lock.release()
            print("SEND StartTask to initialised: ", interrupting["da_name"])
            srv_name = "/"+interrupting["da_name"]+"/TaskER/startTask"
            # print (srv_name)
            # print("\nSWITCHING: waiting for QUEUED startTask\n")
            # rospy.wait_for_service(srv_name, timeout=2)
            print("\nSWITCHING: waiting for QUEUED to be in Init or Wait. It is in <",
                  interrupting["da_state"][0], "> state.")
            while not interrupting["da_state"][0] in ["Wait", "Init"]:
                if not rclpy.ok():
                    return
                print("\nSWITCHING: waiting for QUEUED to be in Init or Wait. It is in <",
                      interrupting["da_state"][0], "> state.")
                r.sleep()

            if interrupting["da_state"][0] == "Wait":
                self.set_DA_signal(
                    da_name=interrupting["da_name"], signal="resume", data=[])
            elif interrupting["da_state"][0] == "Init":
                self.set_DA_signal(
                    da_name=interrupting["da_name"], signal="start", data=[])

            print("\nSWITCHING: waiting for STARTED hold_now\n")

            # rospy.wait_for_service(srv_name, timeout=2)
            print("\nSWITCHING: waiting for STARTED to be in UpdateTask or ExecFSM. It is in <",
                  interrupting["da_state"][0], "> state.")
            while not interrupting["da_state"][0] in ["UpdateTask", "ExecFSM"]:
                if not rclpy.ok():
                    return
                print("\nSWITCHING: waiting for STARTED to be in UpdateTask or ExecFSM. It is in <",
                      interrupting["da_state"][0], "> state.")
                Rate(5).sleep(0.5)

            # rospy.wait_for_service('/'+interrupting["da_name"]+'/TaskER/hold_now', timeout=2)
            self.lock.acquire()
            # print("\n Making executing\n")
            self.makeExecuting()
            self.switchIndicator.clear()
            # print("\n Made executing\n")
            self.lock.release()
            print("\nSWITCHED\n")
        else:
            print("[TH] -- Killing switch thread")

    def shutdown(self):
        self.switchIndicator.set()
