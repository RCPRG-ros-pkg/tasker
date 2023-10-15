#!/usr/bin/env python3
# encoding: utf8
import signal
import sys
import threading
import rclpy
from io import BytesIO
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tasker_msgs.msg import Status, CMD, ScheduleParams
from tasker_msgs.srv import CostConditions, SuspendConditions
import zmq
import time

global socket_ports

# Socket ports
socket_ports = {"status": "5656", "cmd": "5657",
                "cost_cond": "5658", "sus_cond": "5659", "tha_life_tick": "5660"}


def init_socket(context, port, socket_type):

    if socket_type == 'pub':
        socket_status_pub = context.socket(zmq.PUB)
        socket_status_pub.bind("tcp://127.0.0.1:%s" % port)
        return socket_status_pub

    if socket_type == 'pub_connect':
        socket_status_push = context.socket(zmq.PUB)
        socket_status_push.connect("tcp://127.0.0.1:%s" % port)
        return socket_status_push

    elif socket_type == 'sub':
        socket_cmd_sub = context.socket(zmq.SUB)
        socket_cmd_sub.connect("tcp://127.0.0.1:%s" % port)
        socket_cmd_sub.subscribe(b'')
        return socket_cmd_sub

    elif socket_type == 'sub_bind':
        socket_cmd_sub = context.socket(zmq.SUB)
        socket_cmd_sub.bind("tcp://127.0.0.1:%s" % port)
        socket_cmd_sub.subscribe(b'')
        return socket_cmd_sub

    elif socket_type == 'server':
        socket_server = context.socket(zmq.REP)
        socket_server.connect("tcp://127.0.0.1:%s" % port)
        return socket_server

    elif socket_type == 'client':
        socket_client = context.socket(zmq.REQ)
        socket_client.bind("tcp://127.0.0.1:%s" % port)
        return socket_client


def sub_socket_filtered(socket, da_name):
    while True:
        msg = socket.recv_pyobj()
        if msg.recipient_name == da_name:
            return msg


class DACommunicator:
    def __init__(self, da_name, cond_cost_handler, sus_cost_handler):
        global socket_ports
        context = zmq.Context()
        self.da_name = da_name
        self.context = context
        self.cond_cost_handler = cond_cost_handler
        self.sus_cost_handler = sus_cost_handler
        self.sockets = []
        self.socket_status_pub = init_socket(
            self.context, socket_ports['status'], 'pub_connect')
        self.socket_status_pub.setsockopt(zmq.LINGER, 0)
        self.sockets.append(self.socket_status_pub)
        self.socket_cmd_sub = init_socket(
            self.context, socket_ports['cmd'], 'sub')
        self.socket_status_pub.setsockopt(zmq.LINGER, 0)
        self.sockets.append(self.socket_cmd_sub)
        self.socket_life_tick_sub = init_socket(
            self.context, socket_ports['tha_life_tick'], 'sub')
        self.socket_life_tick_sub.setsockopt(zmq.CONFLATE, 1)
        self.socket_life_tick_sub.setsockopt(zmq.RCVTIMEO, 3000)
        self.socket_status_pub.setsockopt(zmq.LINGER, 0)
        self.sockets.append(self.socket_life_tick_sub)
        self.socket_cost_cond_srv = init_socket(
            self.context, socket_ports['cost_cond'], 'server')
        self.socket_cost_cond_srv.setsockopt(zmq.LINGER, 0)
        self.sockets.append(self.socket_cost_cond_srv)
        self.socket_sus_cond_srv = init_socket(
            self.context, socket_ports['sus_cond'], 'server')
        self.socket_sus_cond_srv.setsockopt(zmq.LINGER, 0)
        self.sockets.append(self.socket_sus_cond_srv)
        self.da_is_running = True
        self.is_tha_alive = True
        self.thread_tha_alive = threading.Thread(
            target=self.tha_alive_thread, args=(1,))
        self.thread_tha_alive.start()
        print("started cmd thread_tha_alive thread")

    def __del__(self):
        # self.context.term()
        print("\n\n\n\n DEL DACommunicator \n\n\n\n")
        self.da_is_running = False
        print("\n\n DACommunicator join thread \n\n")
        # self.thread_tha_alive.join()

    def close(self):
        try:
            self.da_is_running = False
            print("\n\n DACommunicator terminating context \n\n")
            i = 0
            for socket in self.sockets:
                socket.close()
                print("closed: ", i)
                i += 1
            self.context.term()
            print("\n\n DACommunicator CLOSED \n\n")

        except Exception as e:
            print('Detected exception in DACommunicator')
            print(e)

    def pub_status(self, msg=None):
        isinstance(msg, Status)
        self.socket_status_pub.send_pyobj(msg)

    def sub_cmd(self):
        return sub_socket_filtered(self.socket_cmd_sub, self.da_name)

    def sub_tha_alive(self):
        return self.socket_life_tick_sub.recv(zmq.NOBLOCK)

    def handle_cost_cond(self):
        return self.socket_cost_cond_srv.send_pyobj(self.cond_cost_handler(self.socket_cost_cond_srv.recv_pyobj()))

    def handle_sus_cond(self):
        return self.socket_cost_cond_srv.send_pyobj(self.sus_cost_handler(self.socket_sus_cond_srv.recv_pyobj()))

    def get_tha_alive_flag(self):
        return self.is_tha_alive

    def tha_alive_thread(self, args):

        timeout = 0
        while self.da_is_running:
            try:
                # print "thaAliveThread: ", self.is_tha_alive
                self.socket_life_tick_sub.recv()
                # self.sub_tha_alive()
                self.is_tha_alive = True
                time.sleep(1)
            except zmq.ZMQError as e:
                # print "thaAliveThread: EXCEPTION: ", e.errno
                # print "thaAliveThread: EXCEPTION: ", zmq.EAGAIN
                if e.errno == zmq.EAGAIN:
                    self.socket_life_tick_sub.close()
                    self.is_tha_alive = False
                    break
                else:
                    raise
            except Exception as e:
                print('Detected exception in DACommunicator')


class THACommunicator:
    def __init__(self):
        global socket_ports
        context = zmq.Context()
        self.context = context
        self.sockets = []
        self.socket_status_sub = init_socket(
            self.context, socket_ports['status'], 'sub_bind')
        self.sockets.append(self.socket_status_sub)
        self.socket_cmd_pub = init_socket(
            self.context, socket_ports['cmd'], 'pub')
        self.socket_cmd_pub.setsockopt(zmq.LINGER, 1000)
        self.sockets.append(self.socket_cmd_pub)
        self.socket_life_tick_pub = init_socket(
            self.context, socket_ports['tha_life_tick'], 'pub')
        self.socket_life_tick_pub.setsockopt(zmq.LINGER, 1000)
        self.sockets.append(self.socket_life_tick_pub)
        self.socket_cost_cond_cli = init_socket(
            self.context, socket_ports['cost_cond'], 'client')
        self.socket_cost_cond_cli.setsockopt(zmq.LINGER, 1000)
        self.sockets.append(self.socket_cost_cond_cli)
        self.socket_sus_cond_cli = init_socket(
            self.context, socket_ports['sus_cond'], 'client')
        self.socket_sus_cond_cli.setsockopt(zmq.LINGER, 1000)
        self.sockets.append(self.socket_sus_cond_cli)
        self.tha_is_running = True
        self.thread_tha_alive = threading.Thread(
            target=self.tha_alive_thread, args=(1,))
        self.thread_tha_alive.start()
        print("started cmd thread_tha_alive thread")

    def __del__(self):
        self.context.term()
        print("\n\n\n\n DEL THACommunicator \n\n\n\n")
        self.tha_is_running = False
        self.thread_tha_alive.join()

    def close(self):
        try:
            self.tha_is_running = False
            print("\n\n THACommunicator terminating context \n\n")
            i = 0
            for socket in self.sockets:
                socket.close()
                print("closed: ", i)
                i += 1
            self.context.term()
            print("\n\n THACommunicator CLOSED \n\n")

        except Exception as e:
            print('Detected exception in THACommunicator')
            print(e)

    def sub_status(self):
        return self.socket_status_sub.recv_pyobj()

    def pub_cmd(self, msg=None):
        isinstance(msg, CMD)
        self.socket_cmd_pub.send_pyobj(msg)

    def pub_life_tick(self):
        self.socket_life_tick_pub.send(b'')

    def call_cost_cond(self, msg=None):
        isinstance(msg, CostConditionsRequest)
        self.socket_cost_cond_cli.send_pyobj(msg)
        return self.socket_cost_cond_cli.recv_pyobj()

    def call_sus_cond(self, msg=None):
        isinstance(msg, SuspendConditionsRequest)
        self.socket_sus_cond_cli.send_pyobj(msg)
        return self.socket_sus_cond_cli.recv_pyobj()

    def tha_alive_thread(self, args):
        while self.tha_is_running:
            try:
                self.pub_life_tick()
                time.sleep(1)
            except Exception as e:
                print('Detected exception in THACommunicator')
