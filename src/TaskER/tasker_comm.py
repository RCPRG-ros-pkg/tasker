#!/usr/bin/env python
# encoding: utf8
import zmq
import time
import threading
from tasker_msgs.msg import Status, CMD, ScheduleParams
from tasker_msgs.srv import CostConditionsResponse, CostConditionsRequest
from tasker_msgs.srv import SuspendConditionsResponse, SuspendConditionsRequest
from io import BytesIO
import logging
global  socket_ports
# Socket ports
socket_ports = {"status": "5656", "cmd": "5657", "cost_cond":"5658", "sus_cond":"5659", "tha_life_tick":"5660"}

class CustomFormatter(logging.Formatter):

	green = "\x1b[32m;20m"
	grey = "\x1b[38;20m"
	yellow = "\x1b[33;20m"
	red = "\x1b[31;20m"
	bold_red = "\x1b[31;1m"
	reset = "\x1b[0m"
	format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"

	FORMATS = {
		logging.DEBUG: green + format + reset,
		logging.INFO: grey + format + reset,
		logging.WARNING: yellow + format + reset,
		logging.ERROR: red + format + reset,
		logging.CRITICAL: bold_red + format + reset
	}

	def format(self, record):
		log_fmt = self.FORMATS.get(record.levelno)
		formatter = logging.Formatter(log_fmt)
		return formatter.format(record)

# create logger with 'spam_application'
logger = logging.getLogger("tasker_comm")
logger.setLevel(logging.INFO)

# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)

ch.setFormatter(CustomFormatter())

logger.addHandler(ch)

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
		socket_cmd_sub.connect ("tcp://127.0.0.1:%s" % port)
		socket_cmd_sub.subscribe(b'')
		return socket_cmd_sub

	elif socket_type == 'sub_bind':
		socket_cmd_sub = context.socket(zmq.SUB)
		socket_cmd_sub.bind ("tcp://127.0.0.1:%s" % port)
		socket_cmd_sub.subscribe(b'')
		return socket_cmd_sub

	elif socket_type == 'server':
		socket_server = context.socket(zmq.REP)
		socket_server.connect("tcp://127.0.0.1:%s" % port)
		return socket_server

	elif socket_type == 'client':
		socket_client = context.socket(zmq.REQ)
		socket_client.bind ("tcp://127.0.0.1:%s" % port)
		return socket_client

def sub_socket_filtered(socket, da_id):
	while True:
		msg = socket.recv_pyobj()
		logger.debug("RECEIVED MSG: '{0}'".format(str(msg)))
		logger.debug("name: '{0}'".format(str(msg.recipient_name)))
		logger.debug("da_id: '{0}'".format(str(da_id)))
		if int(msg.recipient_name) == int(da_id):
			logger.debug("Return MSG")
			return msg



class DACommunicator():
	def __init__(self, da_id, cond_cost_handler, sus_cost_handler, debug = False):
		global socket_ports
		self.debug = debug
		context = zmq.Context()
		self.da_id = da_id
		self.context = context
		self.cond_cost_handler = cond_cost_handler
		self.sus_cost_handler = sus_cost_handler
		self.sockets = []
		self.socket_status_pub = init_socket(self.context, socket_ports['status'], 'pub_connect')
		self.socket_status_pub.setsockopt(zmq.LINGER, 0)
		self.sockets.append(self.socket_status_pub)
		self.socket_cmd_sub = init_socket(self.context, socket_ports['cmd'], 'sub')
		self.socket_status_pub.setsockopt(zmq.LINGER, 0)
		self.sockets.append(self.socket_cmd_sub)
		self.socket_life_tick_sub = init_socket(self.context, socket_ports['tha_life_tick'], 'sub')
		self.socket_life_tick_sub.setsockopt(zmq.CONFLATE, 1)
		self.socket_life_tick_sub.setsockopt(zmq.RCVTIMEO, 3000)
		self.socket_status_pub.setsockopt(zmq.LINGER, 0)
		self.sockets.append(self.socket_life_tick_sub)
		self.socket_cost_cond_srv = init_socket(self.context, socket_ports['cost_cond'], 'server')
		self.socket_cost_cond_srv.setsockopt(zmq.LINGER, 0)
		self.sockets.append(self.socket_cost_cond_srv)
		self.socket_sus_cond_srv = init_socket(self.context, socket_ports['sus_cond'], 'server')
		self.socket_sus_cond_srv.setsockopt(zmq.LINGER, 0)
		self.sockets.append(self.socket_sus_cond_srv)
		self.da_is_running = True
		self.is_tha_alive = True
		self.thread_tha_alive = threading.Thread(target=self.thaAliveThread, args=(1,))
		self.thread_tha_alive.start()
		if self.debug:
			logger.debug("started cmd thread_da_alive thread")

	def __del__(self):
		# self.context.term()
		
		if self.debug:
			logger.debug (" DEL DACommunicator ")
		self.da_is_running = False
		
		if self.debug:
			logger.debug (" DACommunicator join thread ")
		#self.thread_tha_alive.join()

	def close(self):
		try:
			self.da_is_running = False
			
			if self.debug:
				logger.debug (" DACommunicator terminating context ")
			i=0
			for socket in self.sockets:
				socket.close()
				i+=1
			self.context.term()
			
			if self.debug:
				logger.debug (" DACommunicator CLOSED ")

		except Exception as e:
			logger.error ('Detected exception in DACommunicator')
			
	def pub_status(self, msg=None):
		isinstance(msg, Status)
		self.socket_status_pub.send_pyobj( msg )

	def sub_cmd(self):
		try:
			return sub_socket_filtered(self.socket_cmd_sub, self.da_id)
		except zmq.ContextTerminated:
			print("CONTEXT TERMINATED - closing socket")

	def sub_tha_alive(self):
		try:
			return self.socket_life_tick_sub.recv(zmq.NOBLOCK)
		except zmq.ContextTerminated:
			logger.error("CONTEXT TERMINATED - closing socket")

	def handle_cost_cond(self):
		return self.socket_cost_cond_srv.send_pyobj(self.cond_cost_handler(self.socket_cost_cond_srv.recv_pyobj()))

	def handle_sus_cond(self):
		return self.socket_cost_cond_srv.send_pyobj(self.sus_cost_handler(self.socket_sus_cond_srv.recv_pyobj()))

	def get_tha_alive_flag(self):
		return self.is_tha_alive

	def thaAliveThread(self, args):

		timeout = 0
		while self.da_is_running:
			try:
				# 
				if self.debug:
					logger.debug ("thaAliveThread: '{0}'".format(self.is_tha_alive))
				self.socket_life_tick_sub.recv()
				# self.sub_tha_alive()
				self.is_tha_alive = True
				time.sleep(1)
			except zmq.ZMQError as e:
				logger.error ("thaAliveThread: EXCEPTION: '{0}'".format(e.errno))
				# 
				logger.error ("thaAliveThread: EXCEPTION: '{0}'".format(zmq.EAGAIN))
				if e.errno == zmq.EAGAIN:
					self.socket_life_tick_sub.close()
					self.is_tha_alive = False
					break
				else:
					raise
			except Exception as e:
				logger.error ("Detected exception in DACommunicator -> thaAliveThread:\n'{0}'".format(e))


class THACommunicator():
	def __init__(self, debug = False):
		global socket_ports
		context = zmq.Context()
		self.context = context
		self.sockets = []
		self.socket_status_sub = init_socket(self.context, socket_ports['status'], 'sub_bind')
		self.sockets.append(self.socket_status_sub)
		self.socket_cmd_pub = init_socket(self.context, socket_ports['cmd'], 'pub')
		self.socket_cmd_pub.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_cmd_pub)
		self.socket_life_tick_pub = init_socket(self.context, socket_ports['tha_life_tick'], 'pub')
		self.socket_life_tick_pub.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_life_tick_pub)
		self.socket_cost_cond_cli = init_socket(self.context, socket_ports['cost_cond'], 'client')
		self.socket_cost_cond_cli.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_cost_cond_cli)
		self.socket_sus_cond_cli = init_socket(self.context, socket_ports['sus_cond'], 'client')
		self.socket_sus_cond_cli.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_sus_cond_cli)
		self.tha_is_running = True
		self.thread_tha_alive = threading.Thread(target=self.thaAliveThread, args=(1,))
		self.thread_tha_alive.start()
		self.debug = debug
		
		if self.debug:
			logger.debug ("started cmd thread_tha_alive thread")

	def __del__(self):
		self.context.term()
		
		if self.debug:
			logger.debug (" DEL THACommunicator ")
		self.tha_is_running = False
		self.thread_tha_alive.join()

	def close(self):
		try:
			self.tha_is_running = False
			
			if self.debug:
				logger.debug (" THACommunicator terminating context ")
			i=0
			for socket in self.sockets:
				socket.close()
				i+=1
			self.context.term()
			
			if self.debug:
				logger.debug (" THACommunicator CLOSED ")

		except Exception as e:
			logger.error ('Detected exception in THACommunicator')
			

	def sub_status(self):
		try:
			return self.socket_status_sub.recv_pyobj()
		except zmq.ContextTerminated:
			logger.error("CONTEXT TERMINATED - closing socket")

	def pub_cmd(self, msg=None):
		isinstance(msg, CMD)
		if self.debug:
			logger.debug("SENDING CMD: ", msg)
		self.socket_cmd_pub.send_pyobj( msg )
		
	def pub_life_tick(self):
		self.socket_life_tick_pub.send( b'' )
		
	def call_cost_cond(self, msg=None):
		isinstance(msg, CostConditionsRequest)
		self.socket_cost_cond_cli.send_pyobj(msg)
		return self.socket_cost_cond_cli.recv_pyobj()
		
	def call_sus_cond(self, msg=None):
		isinstance(msg, SuspendConditionsRequest)
		self.socket_sus_cond_cli.send_pyobj(msg)
		return self.socket_sus_cond_cli.recv_pyobj()
		

	def thaAliveThread(self, args):

		while self.tha_is_running:
			try:
				self.pub_life_tick()
				time.sleep(1)
			except Exception as e:
				logger.error ('Detected exception in THACommunicator')

