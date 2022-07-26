# -*- coding: utf-8 -*-
# 
from RequestTable import RequestTable, ScheduleRules, ScheduleRule, TaskerReqest

from datetime import datetime, timedelta
import random
if __name__ == '__main__':
 
    # List of given jobs. Each job has an identifier, a deadline, and
    # profit associated with it
    rt = RequestTable()

    sr1 = ScheduleRules()
    sr1.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=9)))
    sr2 = ScheduleRules()
    sr2.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=2)))
    sr3 = ScheduleRules()
    sr3.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=5)))
    sr4 = ScheduleRules()
    sr4.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=10)))
    sr5 = ScheduleRules()
    sr5.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=4)))
    sr6 = ScheduleRules()
    sr6.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=2)))
    sr7 = ScheduleRules()
    sr7.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=5)))
    sr8 = ScheduleRules()
    sr8.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=7)))
    sr9 = ScheduleRules()
    sr9.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=4)))
    sr10 = ScheduleRules()
    sr10.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.combine(datetime.today(),datetime.min.time())+timedelta(minutes=3)))
    
    sr = [
        sr1, sr2, sr3, sr4, sr5, sr6, sr7, sr8, sr9, sr10]
    #     tr1 = TaskerReqest(ID='1', huid='', plan_args='', req_time=datetime.now(), shdl_rules=sr1, \
    #                         priority=15), 

    #     TaskerReqest(ID='2', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr2, \
    #                         priority=2), 
    #     TaskerReqest(ID='3', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr3, \
    #                         priority=18), 
    #     TaskerReqest(ID='4', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr4, \
    #                         priority=1), 
    #     TaskerReqest(ID='5', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr5, \
    #                         priority=25),
    #     TaskerReqest(ID='6', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr6, \
    #                         priority=20), 
    #     TaskerReqest(ID='7', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr7, \
    #                         priority=8), 
    #     TaskerReqest(ID='8', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr8, \
    #                         priority=10), 
    #     TaskerReqest(ID='9', huid='', plan_args='', req_time=datetime.now(),shdl_rules=sr9, \
    #                         priority=12), 
    #     TaskerReqest(ID='10',huid='', plan_args='', req_time=datetime.now(), shdl_rules=sr10, \
    #                         priority=5)
    # ]
    for i in range(1,10):
        sr = ScheduleRules()
        sign = [-1,1][random.randrange(2)]
        sr.addRule(ScheduleRule(rule_type='at', \
                            rule_value=datetime.now()+timedelta(minutes=10+random.randint(1,20)*sign)))
        print ()
        job = TaskerReqest(ID=str(i),huid=str(i), plan_args='', req_time=datetime.now(), shdl_rules=sr, priority=random.randint(1,5))
        job.set_burst_time(timedelta(minutes=random.randint(1,10)))
        job.evaluate_rules()
        rt.addRecord(job)
    rt.schedule_with_priority()
    print ("ITEMS:", rt.items())
    # # stores the maximum deadline that can be associated with a job
    # T = 15
 
    # # schedule jobs and calculate the maximum profit
    # scheduleJobs(jobs, T)
