# -*- coding: utf-8 -*-
# 
from RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest

from datetime import datetime, timedelta
import random
if __name__ == '__main__':
 
    # List of given jobs. Each job has an identifier, a deadline, and
    # profit associated with it
    rt = RequestTable()

    for i in range(1,10):
        sr = ScheduleRules()
        sign = [-1,1][random.randrange(2)]
        sr.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.now()+timedelta(minutes=10+random.randint(1,20)*sign)))
        job = TaskerReqest(ID=str(i),huid=str(i), plan_args='', req_time=datetime.now(), shdl_rules=sr, priority=random.randint(1,5))
        job.set_burst_time(timedelta(minutes=random.randint(1,10)))
        job.evaluate_rules()
        rt.addRecord(job)
    # out.scheduled -- is an ordered list of PriorityShdl.TimeSlot, first is the time slot of the earliest accepted task
    # out.rejected -- is a list of IDs of the rejected tasks
    # profit -- is a sum of priorities of tasks that were accepted by the scheduling algorithm
    out, profit = rt.schedule_with_priority()
    accepted = out.scheduled
    rejected = out.rejected
    print('count of the accepted tasks', len(accepted))
    print('start time of the earliest accepted task', accepted[0].start)
    print('ID of the earliest accepted task', accepted[0].jobID)
    print('Start time of rejected task with ID:', rt.get_request(rejected[0]).get_id(), "-->",rt.get_request(rejected[0]).start_time)

    print('profit', profit)
    # access to all task requests in the table is available
    print ("ITEMS:", rt.items())

