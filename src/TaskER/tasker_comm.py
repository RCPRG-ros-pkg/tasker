#!/usr/bin/env python
# encoding: utf8
import zmq
import time
from tasker_msgs.msg import Status, CMD, ScheduleParams
from tasker_msgs.srv import CostConditionsResponse, CostConditionsRequest
from tasker_msgs.srv import SuspendConditionsResponse, SuspendConditionsRequest
from io import BytesIO

global context, socket_ports

context = zmq.Context()
# Socket ports
socket_ports = {"status": "5656", "cmd": "5657", "cost_cond":"5658", "sus_cond":"5659"}

def init_socket(port, socket_type):
	global context

	if socket_type == 'pub':
		socket_status_pub = context.socket(zmq.PUB)
		socket_status_pub.bind("tcp://*:%s" % port)
		return socket_status_pub

	elif socket_type == 'sub':
		socket_cmd_sub = context.socket(zmq.SUB)
		socket_cmd_sub.connect ("tcp://localhost:%s" % port)
		socket_cmd_sub.subscribe(b'')
		return socket_cmd_sub

	elif socket_type == 'server':
		socket_server = context.socket(zmq.REP)
		socket_server.bind("tcp://*:%s" % port)
		return socket_server

	elif socket_type == 'client':
		socket_client = context.socket(zmq.REQ)
		socket_client.connect ("tcp://localhost:%s" % port)
		return socket_client

def sub_socket_filtered(socket, da_name):
	while True:
		msg = socket.recv_pyobj()
		if msg.recipient_name == da_name:
			return msg



class DACommunicator():
	def __init__(self, da_name, cond_cost_handler, sus_cost_handler):
		global context, socket_ports
		self.da_name = da_name
		self.context = context
		self.cond_cost_handler = cond_cost_handler
		self.sus_cost_handler = sus_cost_handler
		self.socket_status_pub = init_socket(socket_ports['status'], 'pub')
		self.socket_cmd_sub = init_socket(socket_ports['cmd'], 'sub')
		self.socket_cost_cond_srv = init_socket(socket_ports['cost_cond'], 'server')
		self.socket_sus_cond_srv = init_socket(socket_ports['sus_cond'], 'server')

	def pub_status(self, msg=None):
		isinstance(msg, Status)
		self.socket_status_pub.send_pyobj( msg )

	def sub_cmd(self):
		return sub_socket_filtered(self.socket_cmd_sub, self.da_name)

	def handle_cost_cond(self):
		return self.socket_cost_cond_srv.send_pyobj(self.cond_cost_handler(self.socket_cost_cond_srv.recv_pyobj()))

	def handle_sus_cond(self):
		return self.socket_cost_cond_srv.send_pyobj(self.sus_cost_handler(self.socket_sus_cond_srv.recv_pyobj()))

class THACommunicator():
	def __init__(self):
		global context, socket_ports
		self.context = context
		self.socket_status_sub = init_socket(socket_ports['status'], 'sub')
		self.socket_cmd_pub = init_socket(socket_ports['cmd'], 'pub')
		self.socket_cost_cond_cli = init_socket(socket_ports['cost_cond'], 'client')
		self.socket_sus_cond_cli = init_socket(socket_ports['sus_cond'], 'client')


	def sub_status(self):
		return self.socket_status_sub.recv_pyobj()

	def pub_cmd(self, msg=None):
		isinstance(msg, CMD)
		self.socket_cmd_pub.send_pyobj( msg )
		
	def call_cost_cond(self, msg=None):
		isinstance(msg, CostConditionsRequest)
		self.socket_cost_cond_cli.send_pyobj(msg)
		return self.socket_cost_cond_cli.recv_pyobj()
		
	def call_sus_cond(self, msg=None):
		isinstance(msg, SuspendConditionsRequest)
		self.socket_sus_cond_cli.send_pyobj(msg)
		return self.socket_sus_cond_cli.recv_pyobj()
		


