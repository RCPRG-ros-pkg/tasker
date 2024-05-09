#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from diagnostic_msgs.msg import *
from itertools import chain, combinations, permutations, product
from db_connection import dbConnector
import json
import pymongo

def query_kb(ki, task_id):
    query = []
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(task_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(task_id)+'/query_state', KnowledgeQueryService)
        resp = query_proxy([ki])
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return ki


def query_kb_functions(ki, task_id):
    query = []
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(task_id)+'/state/functions_values', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(task_id)+'/state/functions_values', KnowledgeQueryService)
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

def list_functions_updates(task_id):
    fd_list = fetch_functions_details(task_id)
    ki_list = []
    obj_inst = fetch_objects_instances(task_id)
    for fd in fd_list:
        tp_objects = []
        types = fetch_types(task_id)
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
                    ki_list.append(query_kb_functions(ki, task_id))
    return ki_list


def list_predicates_updates(task_id):
    pd_list = fetch_predicates_details(task_id)
    ki_list = []
    obj_inst = fetch_objects_instances(task_id)
    for pd in pd_list:
        tp_objects = []
        types = fetch_types(task_id)
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
            ki_list.append([ki, query_kb(ki, task_id)])
    return ki_list

        
def fetch_functions_details(task_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(task_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(task_id)+'/domain/functions', GetDomainAttributeService)
        function_details = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch function service call failed: %s"%e
    return function_details.items

def fetch_predicates_details(task_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(task_id)+'/query_state', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(task_id)+'/domain/predicates', GetDomainAttributeService)
        predicates_details = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch predicates service call failed: %s"%e
    return predicates_details.items

def fetch_types(task_id):
    rospy.wait_for_service('/rosplan_knowledge_base_'+str(task_id)+'/domain/types', timeout=10)
    try:
        query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(task_id)+'/domain/types', GetDomainTypeService)
        types = query_proxy()
    except rospy.ServiceException, e:
        print "Fetch predicates service call failed: %s"%e
    return types.types

def fetch_objects_instances(task_id):
    types = fetch_types(task_id)
    type_instances_dict = {}
    for typ in types:
        rospy.wait_for_service('/rosplan_knowledge_base_'+str(task_id)+'/state/instances', timeout=10)
        try:
            params = {'type_name' :typ,
                    'include_constants' :'false',
                    'include_subtypes' :'false'
                    } 
            query_proxy = rospy.ServiceProxy('/rosplan_knowledge_base_'+str(task_id)+'/state/instances', GetInstanceService)
            instances = query_proxy(typ, False, False)
            type_instances_dict[typ] = instances.instances
        except rospy.ServiceException, e:
            print "Fetch predicates service call failed: %s"%e
    return type_instances_dict

def update_kb_snapshot(task_id, planner_type):
    dbc = dbConnector(task_id)
    print("Successfully connected to DB of id "+str(task_id))
    snapshot = {}
    predicates = list_predicates_updates(task_id)
    print("List of predicates updates ready.")
    functions = list_functions_updates(task_id)
    print("List of functions updates ready.")
    predicates_list = []
    functions_list = []
    p_iter = 0
    f_iter = 0
    for predicate in predicates:
        ki = predicate[0]
        query_resp = predicate[1]
        att_name = ""
        values = []
        values_list = []
        values_dict = {}
        status = False
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
        print("Predicate " + str(att_name) + " inserted.")
        p_iter += 1

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
        print("Function " + str(att_name) + " inserted.")
        f_iter += 1
    
    # INSERT PLANNER TYPE
    pl = {'planner' : planner_type}
    dbc.insert_planner(pl)

    # INSERT INFO IF RESTART 
    if_restart = {'restart' : 0}
    dbc.insert_if_restart(if_restart)

def powerset(iterable, elements_number):
    s = list(iterable)
    return list(chain.from_iterable(permutations(s, r) for r in range(elements_number, elements_number+1)))