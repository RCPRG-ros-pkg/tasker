from distutils.dir_util import copy_tree
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from math import sqrt
from rosplan_knowledge_msgs.msg import ExprBase, DomainFormula, DomainAssignment, ExprComposite
from diagnostic_msgs.msg import KeyValue
from query_kb_state import query_kb, fetch_functions_details, fetch_predicates_details, update_kb_snapshot
# , update_kb_db, update_kb_from_db
from db_connection import dbConnector
import os, rospy
from time import sleep
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from diagnostic_msgs.msg import *
from os.path import dirname, join
from os import getcwd
import psutil
import shlex
import subprocess

class TaskDBInt:
    def __init__(self, task_name, task_id, task_params, planner_type):
        self.rcprg_path = dirname(dirname(dirname(os.path.abspath(__file__))))
        self.task_name = task_name
        self.transform = None
        self.task_params = task_params
        self.task_id = task_id
        self.task_pddl_created = False
        self.planner_type = planner_type
        self.pddl_files_dir = join(self.rcprg_path, "ROSPlan/rosplan_plans/src/pddl")
        self.kb_launch_file = "roslaunch rosplan_plans launch_kb.launch domain_path:=DOMAIN \
                                    task_problem_path:=TASK_PROBLEM kb_name:=KB_NAME"
        self.domain_path = ""
        self.task_problem_path = ""
        self.kb_snapshot = ""

    def create_db_for_task(self):
        print("In create_db_for_task function.")
        self.find_pddl_files()
        print("find_pddl_files function finished.")
        self.exec_launch()
        print("ROSPlan KB launched.")
        self.create_mongodb()
        print("MongoDB created.")
        self.kill_node()
        print("ROSPlan KB node killed.")
    
    def kill_node(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            if node == "/rosplan_knowledge_base_" + str(self.task_id):
                os.system("rosnode kill "+ node)
                print("ROS Info: /rosplan_knowledge_base node killed successfully.")

    def find_pddl_files(self):
        for subdir, dirs, files in os.walk(self.pddl_files_dir):
            if self.task_name in subdir:
                for inner_subdir, inner_dirs, inner_files in os.walk(subdir):
                    for inner_file in inner_files:
                        filepath = inner_subdir + os.sep + inner_file
                        if 'domain_simple.pddl' in filepath:
                            if self.planner_type == "SMT":
                                self.domain_path = filepath                                
                        elif 'domain_ENHSP.pddl' in filepath:
                            if self.planner_type == "ENHSP":
                                self.domain_path = filepath
                        
                        if 'task_problem_simple.pddl' in filepath:
                            if self.planner_type == "SMT":
                                self.task_problem_path = filepath
                        if 'task_problem_ENHSP.pddl' in filepath:
                            if self.planner_type == "ENHSP":
                                self.task_problem_path = filepath

        # REPLACE DOMAIN AND PROBLEM WITH DOMAIN PATH AND PROBLEM PATH
        self.kb_launch_file = self.kb_launch_file.replace('DOMAIN', self.domain_path)
        self.kb_launch_file = self.kb_launch_file.replace('TASK_PROBLEM', self.task_problem_path)
        self.kb_launch_file = self.kb_launch_file.replace('KB_NAME', "rosplan_knowledge_base_"+str(self.task_id))
    
    def create_mongodb(self):
        update_kb_snapshot(self.task_id, self.planner_type)

    def exec_launch(self):
        print("Calling ROSPlan KB execution")
        subp = subprocess.Popen(shlex.split(self.kb_launch_file))
        p = psutil.Process(subp.pid)
