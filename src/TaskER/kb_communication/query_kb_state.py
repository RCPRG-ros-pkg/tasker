#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from diagnostic_msgs.msg import *
from itertools import chain, combinations, permutations, product
from db_connection import dbConnector
from recalc_kb_fun_values import recalc_kb_fun_values
import json
import pymongo

def get_goals(da_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/state/goals', timeout=10)
    try:
        # print "Calling Service"
        predicate_name = ""
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/state/goals', GetAttributeService)
        resp = query_proxy(predicate_name)
        return resp.attributes
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def update_goal(new_goal, da_type, da_id):
    goals = get_goals(da_id)
    if da_type == "clean":
        for goal in goals:
            v = goal.values[1]
            location = v.value
            if location not in new_goal: 
                rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
                try:
                    query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)
                    resp = query_proxy(3, goal)
                    # print "Response is:", resp
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
    elif da_type == "rescue":
        for goal in goals:
            if goal.attribute_name == "visited" or goal.attribute_name == "at-location":
                v = goal.values[1]
                location = v.value
                if location not in new_goal: 
                    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
                    try:
                        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)
                        resp = query_proxy(3, goal)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
            elif goal.attribute_name == "rescued":
                v = goal.values[0]
                location = v.value
                if location in new_goal:
                    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
                    try:
                        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)
                        resp = query_proxy(3, goal)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

                    goal.is_negative = False
                    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
                    try:
                        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)
                        resp = query_proxy(1, goal)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
    else:
        for goal in goals:
            v = goal.values[1]
            location = v.value
            if location != new_goal: 
                rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
                try:
                    query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)
                    resp = query_proxy(3, goal)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e


def query_kb(ki, da_id):
    query = []
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/query_state', KnowledgeQueryService)
        resp = query_proxy([ki])
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return ki


def query_kb_functions(ki, da_id):
    query = []
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/state/functions_values', KnowledgeQueryService)
        resp = query_proxy([ki])
        resp_fk = resp.false_knowledge
        if len(resp_fk) > 0:
            function_value = resp_fk[0].function_value
            ki.function_value = function_value
            return ki
        else:
            return ki
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    return ki

def list_functions_updates(da_id):
    fd_list = fetch_functions_details(da_id)
    ki_list = []
    obj_inst = fetch_objects_instances(da_id)
    for fd in fd_list:
        tp_objects = []
        types = fetch_types(da_id)
        type_objects_dict = dict.fromkeys(types, 0)
        f_name = fd.name
        f_tp = fd.typed_parameters
        tp_objects = []
        for tp in f_tp:
            tp_objects.append(tp.value)
            type_objects_dict[tp.value] += 1

        for key, value in type_objects_dict.items():
            if value > 0:
                instances = obj_inst[key]

                permuted_instances = list(powerset(instances, value))

                for pi in permuted_instances:
                    ki = KnowledgeItem()
                    ki.knowledge_type = KnowledgeItem.FUNCTION
                    ki.attribute_name = fd.name
                    for i in pi:
                        ki.values.append(diagnostic_msgs.msg.KeyValue(key, i))
                    ki_list.append(query_kb_functions(ki, da_id))
    return ki_list


def list_predicates_updates(da_id):
    pd_list = fetch_predicates_details(da_id)
    ki_list = []
    obj_inst = fetch_objects_instances(da_id)
    for pd in pd_list:
        tp_objects = []
        types = fetch_types(da_id)
        type_objects_dict = dict.fromkeys(types, 0)
        p_name = pd.name
        p_tp = pd.typed_parameters
        tp_objects = []
        for tp in p_tp:
            tp_objects.append(tp.value)
            type_objects_dict[tp.value] += 1        
        permuted_instances_dict = {}
        permuted_instances_list = []
        permuted_instances = []
        for key, value in type_objects_dict.items():
            if value > 0:
                instances = obj_inst[key]
                # if a predicate is declared in domain file but there are no instances in problem file
                if len(instances) == 0:
                    break                    
                permuted_instances_dict[key] = (powerset(instances, value))
        
        for key, value in permuted_instances_dict.items():
            it = 0
            for v in value:
                if (type(v) is list or type(v) is tuple) and len(v) == 1:
                    if type(v) == "tuple":
                        value = list(v)
                    v = v[0]
                    value[it] = v
                it += 1

        for tpo in list(set(tp_objects)):
            for key, value in permuted_instances_dict.items():
                if key == tpo:
                    permuted_instances_list.append(value)

        if len(permuted_instances_list) == 1:
            permuted_instances_list = permuted_instances_list[0]
            for i in range(len(permuted_instances_list)):
                if type(permuted_instances_list[i]) == tuple:
                    permuted_instances_list[i] = list(permuted_instances_list[i])
        
        if len(permuted_instances_list) > 0 and len(permuted_instances_dict.keys()) > 1:
            if len(permuted_instances_list) == 1:
                for element in product(permuted_instances_list[0]):
                    permuted_instances.append(element)
            elif len(permuted_instances_list) == 2:
                for element in product(permuted_instances_list[0], permuted_instances_list[1]):
                    permuted_instances.append(element)
            elif len(permuted_instances_list) == 3:
                for element in product(permuted_instances_list[0], permuted_instances_list[1], permuted_instances_list[2]):
                    permuted_instances.append(element)
            elif len(permuted_instances_list) == 4:
                for element in product(permuted_instances_list[0], permuted_instances_list[1], permuted_instances_list[2], permuted_instances_list[3]):
                    permuted_instances.append(element)
        else:
            for element in permuted_instances_list:
                permuted_instances.append(tuple(element))
        
        permuted_instances = list(set(permuted_instances))
        for pi in permuted_instances:
            ki = KnowledgeItem()
            ki.knowledge_type = KnowledgeItem.FACT
            ki.attribute_name = pd.name
            if len(permuted_instances_dict.keys()) > 1:
                for i in pi:
                    # finding object type of instance i
                    for key, value in permuted_instances_dict.items():
                        tuples_equal = False
                        for v in value:
                            if tuple(i) == tuple(v):
                                tuples_equal = True
                                break
                        if tuples_equal:
                            if type(i) is tuple:
                                for ins in i:
                                    ki.values.append(diagnostic_msgs.msg.KeyValue(key, ins))
                            else:
                                ki.values.append(diagnostic_msgs.msg.KeyValue(key, i))
                            break
            else:
                for key, value in permuted_instances_dict.items():
                    tuples_equal = False
                    for v in value:
                        if tuple(pi) == tuple(v):
                            tuples_equal = True
                            break
                    if tuples_equal:
                        for i in tuple(pi):
                            ki.values.append(diagnostic_msgs.msg.KeyValue(key, i))
                        break
            ki_list.append([ki, query_kb(ki, da_id)])
    return ki_list

        
def fetch_functions_details(da_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/domain/functions', GetDomainAttributeService)
        function_details = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch function service call failed: %s"%e
    return function_details.items

def fetch_predicates_details(da_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/domain/predicates', GetDomainAttributeService)
        predicates_details = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch predicates service call failed: %s"%e
    return predicates_details.items

def fetch_types(da_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/domain/types', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/domain/types', GetDomainTypeService)
        types = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch predicates service call failed: %s"%e
    return types.types

def fetch_objects_instances(da_id):
    types = fetch_types(da_id)
    type_instances_dict = {}
    for typ in types:
        rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/state/instances', timeout=10)
        try:
            params = {'type_name' :typ,
                    'include_constants' :'false',
                    'include_subtypes' :'false'
                    } 
            query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/state/instances', GetInstanceService)
            instances = query_proxy(typ, False, False)
            type_instances_dict[typ] = instances.instances
        except rospy.ServiceException, e:
            print "Fetch predicates service call failed: %s"%e
    return type_instances_dict

def update_kb(da_id, da_type, executable_tasks, args, cfg_actions, tf):
    # Updating created KB with values calculated based on fetched robot state
    print("Updating created KB with values calculated based on fetched robot state")
    goal = ''
    goals = []
    if da_type in executable_tasks:
        if da_type == "clean":
            distances = recalc_kb_fun_values(da_type, args, cfg_actions, tf, executable_tasks)
            for i in range(len(args)):
                if args[i] == "m1":
                    goals.append(args[i+1])
                if args[i] == "m2":
                    goals.append(args[i+1])
                if args[i] == "m3":
                    goals.append(args[i+1])
            for i in range(len(goals)):
                # distance Knowledge item
                ki = KnowledgeItem()
                ki.knowledge_type = KnowledgeItem.FUNCTION
                ki.attribute_name = "distance"
                ki.values.append(diagnostic_msgs.msg.KeyValue("location", "stpt"))
                ki.values.append(diagnostic_msgs.msg.KeyValue("location", goals[i]))
                ki.function_value = float(distances[i])

                # Updating/Creating KB MongoDB with updated KB state
                # print("Updating/Creating KB MongoDB")
                # update_kb_snapshot(self.task_id)
                print("Updating MongoDB with calculated function value")
                update_kb_db(da_id, ki)
                print("Updating KB with current DB state")
                update_kb_from_db(da_id)
                print("Successfully updated KB")
        else:
            distance = recalc_kb_fun_values(da_type, args, cfg_actions, tf, executable_tasks)
            for i in range(len(args)):
                if args[i] == "miejsce":
                    goal = args[i+1]
                    break
            # distance Knowledge item
            ki = KnowledgeItem()
            ki.knowledge_type = KnowledgeItem.FUNCTION
            ki.attribute_name = "distance"
            ki.values.append(diagnostic_msgs.msg.KeyValue("location", "stpt"))
            ki.values.append(diagnostic_msgs.msg.KeyValue("location", goal))
            ki.function_value = float(distance)

            # Updating/Creating KB MongoDB with updated KB state
            # print("Updating/Creating KB MongoDB")
            # update_kb_snapshot(self.task_id)
            print("Updating MongoDB with calculated function value")
            update_kb_db(da_id, ki)
            print("Updating KB with current DB state")
            update_kb_from_db(da_id)
            print("Successfully updated KB")

def update_kb_snapshot(task_id):
    dbc = dbConnector(task_id)
    snapshot = {}
    predicates = list_predicates_updates(task_id)
    functions = list_functions_updates(task_id)
    predicates_list = []
    functions_list = []
    for predicate in predicates:
        ki = predicate[0]
        query_resp = predicate[1]
        att_name = ""
        values = []
        values_list = []
        values_dict = {}
        status = False
        # print(predicate)
        if query_resp.all_true is False:
            query_resp.false_knowledge = query_resp.false_knowledge[0]
            att_name = query_resp.false_knowledge.attribute_name
            values_list = query_resp.false_knowledge.values
            status = False
        else:
            att_name = ki.attribute_name
            values_list = ki.values
            status = True

        for v in values_list:
            values_dict = {}
            values_dict[v.key] = v.value
            values.append(values_dict) 
        
        id = str(att_name)
        for v in values:
            id += str(v.values()[0])
        
        p = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'all_true': status}
        dbc.insert_predicate(p)

    for function in functions:
        values_list = []
        values_dict = {}
        values = []
        att_name = function.attribute_name
        values_list = function.values
        function_value = function.function_value

        for v in values_list:
            values_dict = {}
            values_dict[v.key] = v.value
            values.append(values_dict) 

        id = str(att_name)
        for v in values:
            id += str(v.values()[0])

        f = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'function_value': function_value}
        
        dbc.insert_function(f)

def update_kb_from_db(id):
    dbc = dbConnector(id)
    [predicates, functions] = dbc.read_db()
    for p in predicates:
        ki = KnowledgeItem()
        ki.knowledge_type = KnowledgeItem.FACT
        ki.attribute_name = str(p['attribute_name'])
        values_list = p['values']
        for value in values_list:
            k = str(value.keys()[0])
            v = str(value.values()[0])
            ki.values.append(diagnostic_msgs.msg.KeyValue(k, v))
        
        if p['all_true'] is True:
            remove_from_kb(ki, id)
            add_to_kb(ki, id)
        else:
            remove_from_kb(ki, id)
        
    for f in functions:
        old_ki = KnowledgeItem()
        old_ki.knowledge_type = KnowledgeItem.FUNCTION
        old_ki.attribute_name = str(f['attribute_name'])
        new_ki = KnowledgeItem()
        new_ki.knowledge_type = KnowledgeItem.FUNCTION
        new_ki.function_value = float(f['function_value'])
        new_ki.attribute_name = str(f['attribute_name'])
        values_list = f['values']
        for value in values_list:
            k = str(value.keys()[0])
            v = str(value.values()[0])
            new_ki.values.append(diagnostic_msgs.msg.KeyValue(k, v))
            old_ki.values.append(diagnostic_msgs.msg.KeyValue(k, v))
        f_val = query_kb_functions(old_ki, id).function_value
        old_ki.function_value = f_val
        remove_from_kb(old_ki, id)
        add_to_kb(new_ki, id)


def update_kb_db(id, ki, pred_status=True):
    values_list = []
    values_dict = {}
    values = []

    att_name = ki.attribute_name
    values_list = ki.values
    function_value = ki.function_value
    dbc = dbConnector(id)

    for v in values_list:
        values_dict = {}
        values_dict[v.key] = v.value
        values.append(values_dict) 

    id = str(att_name)
    for v in values:
        id += str(v.values()[0])
    
    if ki.knowledge_type == KnowledgeItem.FACT:
        f = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'all_true': pred_status}
        dbc.insert_predicate(f)
    
    elif ki.knowledge_type == KnowledgeItem.FUNCTION:
        f = {'_id' :  id,
            'attribute_name': att_name,
            'values': values,
            'function_value': function_value}
        dbc.insert_function(f)

def remove_from_kb(ki, da_id):
    # print "Waiting for service"
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
    try:
        # print "Calling Service"
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)

        # query_proxy takes 2 arguments. Update type (REMOVE_KNOWLEDGE=2, KI)
        resp = query_proxy(2, ki)
        # print "Response is:", resp
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

def add_to_kb(ki, da_id):
    # print "Waiting for service"
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(da_id)+'/update', timeout=10)
    try:
        # print "Calling Service"
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(da_id)+'/update', KnowledgeUpdateService)

        # query_proxy takes 2 arguments. Update type (REMOVE_KNOWLEDGE=2, KI)
        resp = query_proxy(0, ki)
        # print "Response is:", resp
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False
        
def powerset(iterable, elements_number):
    s = list(iterable)
    return list(chain.from_iterable(permutations(s, r) for r in range(elements_number, elements_number+1)))