# -*- coding: utf-8 -*-

import copy
from datetime import datetime, timedelta
# from time import timedelta

import pandas as pd
# Gantt visualisation
import plotly.express as px
import plotly.offline
import tasker_msgs
from tasker_msgs.msg import ScheduleParams
from ppretty import ppretty
import PriorityShdl
import numpy as np

class ScheduleRule():
    def __init__(self, rule_type, rule_value):
        self.rule_setup_datetime = datetime.now()
        if not rule_type  in ['after','before','at']:
            raise Exception('Unknown rule type:' + rule_type )
        assert isinstance(rule_value, datetime)
        self.rule_type = rule_type
        self.rule_value = rule_value

    def get_type(self):
        return self.rule_type

    def get_value(self):
        return self.rule_value


class ScheduleRules():
    def __init__(self, msg=None):
        self.list = {'at':None,'before':None, 'after':None}
        if msg is not None:
            self.addRuleWithMSG(msg)

    def addRule(self, rule):
        assert isinstance(rule, ScheduleRule)
        self.list[rule.get_type()] = rule.get_value()
    def get_shdl_rules_types(self):
        types = []
        for item in self.list:
            if self.list[item] is not None:
                types.append(item)
        return types
    def get_value_from_type(self, key):
        return copy.copy(self.list[key])

    def addRuleWithMSG(self, msg):
        assert isinstance(msg , tasker_msgs.msg.ScheduleRules)
        if msg.rule_type == 'at':
            in_value = datetime.strptime(msg.rule_value, '%d/%m/%Y, %H:%M:%S.%f')
            self.addRule(ScheduleRule('at', in_value))
        else:
            print ("Rule type not supported")
            raise Exception("Rule type not supported")
    
    def to_string(self):
        return str(self.list)

class TaskerReqest():
    def __init__(self, ID, huid, plan_args, shdl_rules, priority, req_time):
        assert isinstance(req_time, datetime)
        assert isinstance(shdl_rules, ScheduleRules)
        assert isinstance(priority, int)

        self.id = ID
        self.huid = huid
        self.plan_args = plan_args
        self.shdl_rules = shdl_rules
        self.shdl_params = ScheduleParams()
        self.last_cmd_sent = None
        self.priority = priority
        self.req_time = req_time
        self.deadline = None
        self.start_time = None
        self.burst_time = None
        self.plan = None
        self.execution_error_count = 0
        self.replan_count = 0
        self.state = None
        self.switch_priority = None

    def __str__(self):
        s = ppretty(self)
        return s


    def __getitem__(self, key):
        return self[key]

    def get_gantt_data(self):
        gantt_data = dict(Task=self.huid, Start=self.start_time, Finish=self.deadline, Priority=self.priority)
        return gantt_data

    def evaluate_rules(self):
        # print ("RULE TYPES: ",self.shdl_rules.get_shdl_rules_types() )
        # print ("rself.burst_time: ", self.burst_time)
        if 'at' in self.shdl_rules.get_shdl_rules_types():
            print ("deadline: ", self.shdl_rules.get_value_from_type('at'))
            self.deadline = self.shdl_rules.get_value_from_type('at')
            self.start_time  = self.deadline - self.burst_time

    def get_id(self):
        return copy.copy(self.id)

    def get_value_by_key(self, key):
        value = getattr(self, key)
        return value

    def get_plan_args(self):
        return copy.copy(self.plan_args)

    def get_shdl_rules_types(self):
        types = []
        for item in self.shdl_rules:
            types.append(item.get_type())
        return types

    def set_burst_time(self, burst_time):
        isinstance(burst_time, timedelta)
        self.burst_time = burst_time

    def set_start_time(self, start_time):
        self.start_time = start_time

    def set_deadline(self, deadline):
        self.deadline = deadline

    def to_string(self):
        return str("|ID: "+str(self.id)+"|huid: "+str(self.huid)+"|plan_args: "+str(self.plan_args)+
                    "\n\t|shdl_rules: "+self.shdl_rules.to_string()+"|priority: "+str(self.priority)+
                    "\n\t|deadline: "+str(self.deadline)+"|start_time: "+str(self.start_time)+"|burst_time: "+str(self.burst_time)+"|req_time: "+str(self.req_time))
    
    def set_plan(self, plan):
        self.plan = copy.copy(plan)

    def get_execution_error_count(self):
        return self.execution_error_count

    def get_replan_count(self):
        return self.replan_count

    def increase_replan_count(self):
        self.replan_count = self.replan_count + 1

    def reset_replan_count(self):
        self.replan_count = 0

    def reset_excution_error_count(self):
        self.excution_error_count = 0

    def increase_execution_error_count(self):
        self.execution_error_count = self.execution_error_count + 1

    def to_priority_job(self):
        self.evaluate_rules()
        return PriorityShdl.Job(taskId=self.id, deadline=self.deadline, \
                        profit=self.priority, burstTime=self.burst_time)

class RequestTable():
    def __init__(self):
        self.dictionary = {}
        self.last_id = 0
        self.free_id = [0]
        self.used_ids = []
        self.gantt_plot = None
        self.gantt_app = None
        self.gantt_view = None
        self.gantt_data = []
        self.shdl_result = None
        # self.initialiseGantt()
        # self.updateGantt()

    # def __del__(self):
    #     sys.exit(self.gantt_app.exec_())

    def updateGantt(self, with_rejected=True):
        all_data_frame = []
        rejected_data_frame = []
        accepted_data_frame = []
        print (self.shdl_result)
        for record in self.dictionary:
            is_rejected = ''
            if self.shdl_result != None:
                if self.dictionary[record].id in self.shdl_result.rejected:
                    is_rejected = 'Rejected'
                    data = self.dictionary[record].get_gantt_data()
                    data['Rejected']=is_rejected
                    rejected_data_frame.append(data)
                else:
                    is_rejected = ''
                    data = self.dictionary[record].get_gantt_data()
                    data['Rejected']=is_rejected
                    accepted_data_frame.append(data)
                all_data_frame.append(data)

            if with_rejected:
                show_data = all_data_frame
            else:
                show_data = accepted_data_frame

            print ("DATA: ", show_data)
        self.gantt_data = pd.DataFrame(show_data)
        if len(self.gantt_data) > 0:
            self.gantt_plot = px.timeline(self.gantt_data, x_start="Start", x_end="Finish", y="Task", height=600, width=800, color="Priority", text="Rejected")
            # self.gantt_plot.update_traces(textposition='outside')
            if with_rejected:
                plotly.offline.plot(self.gantt_plot, filename='/tmp/tasker_chart_with_rejected.html', auto_open=False)
            else:
                plotly.offline.plot(self.gantt_plot, filename='/tmp/tasker_chart_no_rejected.html', auto_open=False)
            # print ("PLOT:")
            # print self.gantt_plot
            # self.gantt_plot.update_yaxes(autorange="reversed")
            # file_path = os.path.abspath("/tmp/name.html")
            # self.gantt_view.load(QUrl.fromLocalFile(file_path))

    # def initialiseGantt(self):
        # self.gantt_app = QApplication(sys.argv)
        # self.gantt_view = QWebView()
        # self.gantt_view.show()

    # def showGantt(self):
    #     if len(self.gantt_data) > 0:
    #         self.gantt_plot.update_yaxes(autorange="reversed")
    #         # self.gantt_plot.show()
    #         self.gantt_view.show()

    def copyTableAsList(self):
        result = []
        for record in self.dictionary:
            result.append(self.dictionary[record])
        return result

    def isEmpty(self):
        return len(self.dictionary.items()) == 0

    def getNextID(self):
        if len(self.free_id) > 0:
            next_id = min(self.free_id)
            self.free_id.remove(next_id)
            return next_id
        else:
            self.last_id = self.last_id+1
            return self.last_id

    def addRecord(self, record):
        isinstance(record, TaskerReqest)
        self.used_ids.append(record.get_id())
        self.dictionary[record.get_id()] = record
        self.updateGantt()

    def updateRecord(self, record):
        isinstance(record, TaskerReqest)
        if record.get_id() not in self.used_ids:
            raise Exception('RequestTable -> trying to update a record that does not exist in the table:' + record.to_string())
        self.dictionary[record.get_id()] = record
        self.updateGantt()

    def removeRecord(self, record_id):
        isinstance(record_id, int)
        self.removeRecord(self.get_request_by_id(record_id))

    def removeRecord(self, record):
        isinstance(record, TaskerReqest)
        self.used_ids.remove(record.get_id())
        self.free_id.append(record.get_id())
        del self.dictionary[record.get_id()]


    def get_requst(self, record_id):
        if record_id not in self.used_ids:
            return None
        return self.dictionary[record_id]

    def get_request_copy(self, record_id):
        return copy.copy(self.get_requst(record_id))

    def get_highest_by_key_higher_than(self, key, reference=None):
        if len(self.used_ids) == 0:
            raise Exception('There is no records in the table. Cannot get record with a highest value')
        local_dict = copy.copy(self.dictionary)
        if reference is not None:
            ref_value = local_dict[reference].get_value_by_key(key)
            del local_dict[reference]
            if len(local_dict.items()) == 0:
                print ("No records in the table beside the executing request")
                return None
        max_value = local_dict.items()[0][1].get_value_by_key(key)
        max_id = local_dict.items()[0][0]
        for record in local_dict.items():
            i_id = record[0]
            i_value = record[1].get_value_by_key(key)
            if i_value > max_value:
                max_value = i_value
                max_id = i_id
        if reference is not None:
            if ref_value < max_value:
                return self.get_request_by_id(max_id)
            else:
                return None
        else:
            return self.get_request_by_id(max_id)

    def get_highest_by_key_from_list(self, key, request_list):
        if len(request_list) == 0:
            raise Exception('The list is empty')
        max_value = request_list[0].get_value_by_key(key)
        max_id = request_list[0].get_id()
        for req in request_list:
            i_value = req.get_value_by_key(key)
            i_id = req.get_id()
            if i_value > max_value:
                max_value = i_value
                max_id = i_id
        return self.get_request_by_id(max_id)

    def get_minimal_by_key_from_list(self, key, request_list):
        if len(request_list) == 0:
            raise Exception('The list is empty')
        min_value = request_list[0].get_value_by_key(key)
        min_id = request_list[0].get_id()
        for req in request_list:
            i_value = req.get_value_by_key(key)
            i_id = req.get_id()
            if i_value < min_value:
                min_value = i_value
                min_id = i_id
        return self.get_request_by_id(min_id)

    def get_minimal_by_key(self, key):
        if len(self.used_ids) == 0:
            raise Exception('There is no records in the table. Cannot get record with a minimal value')
        min_value = self.dictionary[self.used_ids[0]]
        min_id = self.used_ids[0]
        for record in self.dictionary.items():
            i_value = record[1].get_value_by_key(key)
            i_id = record[0]
            if i_value < min_value:
                min_value = i_value
                min_id = i_id
        return self.get_request_by_id(min_id)

    def delayed_requests(self):
        result = []
        for record in self.dictionary.items():
            # print ("rec time", record[1].get_value_by_key('start_time') )
            if record[1].get_value_by_key('start_time') == None:
                # print ("Record: <",record[1].get_id(),"> has not start_time set")
                return []
            if record[1].get_value_by_key('start_time') < datetime.now():
                result.append(record[1])
        return result
    
    def conflicts(self, request, delay = timedelta()):
        isinstance(delay, timedelta)
        # print ("conflicts")
        conflicts = []
        local_dir = copy.copy(self.dictionary)
        del local_dir[request.get_id()]
        for record in local_dir.items():
            if self.get_conflict_duration(request,record[1], delay) > 0:
                    conflicts.append(record[1])
        return conflicts

    def get_conflict_duration(self, req1, req2, delay = timedelta()):
        isinstance(delay, timedelta)
        # print ("in_conflict")
        a_start = req1.get_value_by_key('start_time')
        a_deadline = req1.get_value_by_key('deadline')
        b_start = req2.get_value_by_key('start_time')
        b_deadline = req2.get_value_by_key('deadline')
        print ("START:")
        print ("---------1---------\t\t---------2---------" )
        print (a_start, "\t", b_start)
        print ("DEADLINE:")
        print ("---------1---------\t\t---------2---------" )
        print (a_deadline, "\t", b_deadline)
        if req1.get_value_by_key('start_time')<req2.get_value_by_key('deadline'):
            if (req1.get_value_by_key('deadline')+ delay)>req2.get_value_by_key('start_time'):
                return (req1.get_value_by_key('deadline')+ delay) - req2.get_value_by_key('start_time')
        return 0

    def get_priority_conflicts(self, req, delay=timedelta()):
        isinstance(delay, timedelta)
        # print ("has_priority_conflict")
        conflicts = self.conflicts(req)
        print ("CONFLICTS: ", conflicts)
        priority_conflicts = []
        for conflict in conflicts:
            if conflict.priority >= req.get_value_by_key('priority'):
                print ("Has priority conflict")
                print ("Has priority conflict")
                print ("Has priority conflict")
                print ("Has priority conflict")
                priority_conflicts.append(conflict)
        return priority_conflicts

    # calc sum of delay of all tasks affected by a 'req' delay
    def calc_delay_task_cost(self, req, delay=timedelta()):
        isinstance(delay, timedelta)
        priority_conflicts = self.get_priority_conflicts(req, delay)
        cost = 0
        for conflict in priority_conflicts:
            conflict_duration = self.get_conflict_duration(req, conflict, delay)
            cost += conflict_duration + self.calc_delay_task_cost(conflict, conflict_duration)
        return cost

    # return the earliest job ID to be done
    def schedule_with_priority(self):
        priority_scheduler = PriorityShdl.PriorityScheduler()
        for req in self.dictionary.items():
            priority_scheduler.addJob(req[1].to_priority_job())
        self.shdl_result = priority_scheduler.scheduleJobs()

        self.shdl_result.scheduled.sort(key=lambda x: x.start, reverse=False)
        print (self.shdl_result.rejected)
        self.updateGantt(with_rejected=True)
        self.updateGantt(with_rejected=False)
        return self.shdl_result.scheduled[0].jobID

