#!/usr/bin/python2

from RequestTable import ScheduleRules, ScheduleRule, TaskerReqest
import TaskHarmoniserAgent
from tasker_comm import DACommunicator
from tasker_msgs.msg import Status, ScheduleParams

from datetime import datetime, timedelta
import random
import time
task_type_priority_mapping = TaskHarmoniserAgent.TaskHarmoniserAgent.TaskTypePriorityMap()
task_type_priority_mapping.addTaskType('transport', 1)
task_type_priority_mapping.addTaskType('fallAssistance', 5)
task_type_priority_mapping.addTaskType('guideHuman', 2)
da_name = 'da1'
tasker_communicator =  DACommunicator(da_name, None, None)

tha = TaskHarmoniserAgent.TaskHarmoniserAgent(task_type_priority_mapping.getMap())
for i in range(20):
    task_type_spec = random.choice(task_type_priority_mapping.getMap())
    print(task_type_spec)
    da_id = tha.getNextID()

    sr = ScheduleRules()
    sr.addRule(ScheduleRule(rule_type='at', \
                        rule_value=datetime.now()+timedelta(minutes=random.randint(1,20))))
    da_name='da'+str(da_id)
    da_type=task_type_spec['task_type']
    print('da_type: ', da_type)
    tha.addDA(da_id, da_name=da_name, da_type=da_type,shdl_rules=sr, req_time=datetime.now())

    #
    # Status tructure
    #
    # int32 da_id
    # string da_name
    # string type
    # tasker_msgs/ScheduleParams schedule_params
    # # task state: initialise, exec, susp, wait, ut
    # string[] da_state
    # raw_input("Press Enter to continue...")
    tasker_communicator.pub_status(Status(da_id=da_id, da_name=da_name, type=da_type,\
                         schedule_params=ScheduleParams(), da_state=['Init']))
    time.sleep(0.5)

    # job = TaskerReqest(ID=str(i),huid=str(i), plan_args='', req_time=datetime.now(), shdl_rules=sr, priority=random.randint(1,5))
    # job.set_burst_time(timedelta(minutes=random.randint(1,10)))
    # job.evaluate_rules()
    # rt.addRecord(job)
# rt.schedule_with_priority()

# shdl_rules=ScheduleRules()
# shdl_rules.addRule
# tha.addDA(da_id, da_name=da_name, da_type='transport',shdl_rules=ScheduleRules, req_time)
print(tha.request_table.get_request(da_id))
f = open("cost_file.txt", "w")
tha.schedule_smit(cost_file=f)
print("RUN: ", tha.interruptField)
f.close()
tha.close()
print('Finished')