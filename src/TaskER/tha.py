#!/usr/bin/env python3
import signal
import sys
import threading
import rclpy
from TaskHarmoniser import TaskHarmoniser
import time
from multitasker.srv import TaskRequest, TaskRequestResponse
from multitasker.msg import InitParams
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

_FINISH = False
scheduler = threading.Thread
switcher = threading.Thread
th = TaskHarmoniser


def init_da(request, response):
    global th
    print("GOT REQUEST")
    new_id = th.get_next_id()
    print("ID: ", new_id)
    da_name = "DA_" + str(new_id)
    init_params = request.init_params
    th.initialise_da(request.application, request.version,
                     new_id, da_name, init_params)
    th.add_da(new_id, da_name, request.application)
    th.update_schedule_params(new_id, init_params)
    return TaskRequestResponse(-1)


def scheduler_thread():
    global _FINISH
    global th
    while True:
        cost_file = open("./TH_cost", "a+")
        print("\n SCHEDULING \n")
        th.schedule_new(cost_file)
        print("\n SCHEDULED \n")
        time.sleep(2)
        if _FINISH:
            th.send_indicator()
            cost_file.close()
            break


def switcher_thread():
    global th
    global _FINISH
    while True:
        print("\n SWITCHING \n")
        th.switch_da()
        if _FINISH:
            break


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    global _FINISH
    _FINISH = True
    global scheduler
    global switcher
    scheduler.join()
    switcher.join()
    sys.exit(0)


class TaskHarmoniserNode(Node):
    def __init__(self):
        super().__init__('TH')
        self.th = TaskHarmoniser()
        qos = QoSProfile(depth=10)
        self.srv = self.create_service(
            TaskRequest, 'TH/new_task', init_da, qos_profile=qos)
        self.srv.set_callback_group(MutuallyExclusiveCallbackGroup())
        self.get_logger().info('Ready')


if __name__ == "__main__":
    global th
    rclpy.init(args=sys.argv)
    print("ready")
    signal.signal(signal.SIGINT, signal_handler)
    global scheduler
    global switcher
    scheduler = threading.Thread(target=scheduler_thread)
    scheduler.start()
    switcher = threading.Thread(target=switcher_thread)
    switcher.start()

    rclpy.spin(TaskHarmoniserNode())

    print("END")
