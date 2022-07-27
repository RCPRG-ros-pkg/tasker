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

def sub_socket_filtered(socket, da_id, logger):
	while True:
		msg = socket.recv_pyobj()
		logger.debug("RECEIVED MSG: '{0}'".format(str(msg)))
		logger.debug("name: '{0}'".format(str(msg.recipient_name)))
		logger.debug("da_id: '{0}'".format(str(da_id)))
		if int(msg.recipient_name) == int(da_id):
			logger.debug("Return MSG")
			return msg



class ZmqDACommunicator():
	def __init__(self, da_id, cond_cost_handler, sus_cost_handler, logger):
		global socket_ports
		self.logger = logger
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
		self.logger.debug("started cmd thread_da_alive thread")

	def __del__(self):
		# self.context.term()
		
		self.logger.debug (" DEL DACommunicator ")
		self.da_is_running = False
		
		self.logger.debug (" DACommunicator join thread ")
		#self.thread_tha_alive.join()

	def close(self):
		try:
			self.da_is_running = False
			
			self.logger.debug (" DACommunicator terminating context ")
			i=0
			for socket in self.sockets:
				socket.close()
				i+=1
			self.context.term()
			
			self.logger.debug (" DACommunicator CLOSED ")

		except Exception as e:
			self.logger.error ('Detected exception in DACommunicator')
			
	def pub_status(self, msg=None):
		isinstance(msg, Status)
		self.logger.debug (" DACommunicator publishing status ")
		self.socket_status_pub.send_pyobj( msg )
		self.logger.debug (" DACommunicator published status ")

	def sub_cmd(self):
		try:
			return sub_socket_filtered(self.socket_cmd_sub, self.da_id, self.logger)
		except zmq.ContextTerminated:
			self.logger.warning("CONTEXT TERMINATED - closing socket")

	def sub_tha_alive(self):
		try:
			return self.socket_life_tick_sub.recv(zmq.NOBLOCK)
		except zmq.ContextTerminated:
			self.logger.error("CONTEXT TERMINATED - closing socket")

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
				self.logger.debug ("thaAliveThread: '{0}'".format(self.is_tha_alive))
				self.socket_life_tick_sub.recv()
				# self.sub_tha_alive()
				self.is_tha_alive = True
				time.sleep(1)
			except zmq.ZMQError as e:
				self.logger.error ("thaAliveThread: EXCEPTION: '{0}'".format(e.errno))
				# 
				self.logger.error ("thaAliveThread: EXCEPTION: '{0}'".format(zmq.EAGAIN))
				if e.errno == zmq.EAGAIN:
					self.socket_life_tick_sub.close()
					self.is_tha_alive = False
					break
				else:
					raise
			except Exception as e:
				self.logger.error ("Detected exception in DACommunicator -> thaAliveThread:\n'{0}'".format(e))


class ZmqTHACommunicator():
	def __init__(self, logger):
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
		self.logger = logger
		
		self.logger.debug ("started cmd thread_tha_alive thread")

	def __del__(self):
		self.context.term()
		
		self.logger.debug (" DEL THACommunicator ")
		self.tha_is_running = False
		self.thread_tha_alive.join()

	def close(self):
		try:
			self.tha_is_running = False
			
			self.logger.debug (" THACommunicator terminating context ")
			i=0
			for socket in self.sockets:
				socket.close()
				i+=1
			self.context.term()
			
			self.logger.debug (" THACommunicator CLOSED ")

		except Exception as e:
			self.logger.error ('Detected exception in THACommunicator')
			

	def sub_status(self):
		try:
			self.logger.debug (" THACommunicator receiveing status")
			msg = self.socket_status_sub.recv_pyobj()
			if msg is not None:
				self.logger.debug (" THACommunicator received status: ")
				print (str(msg.da_id) + str(msg.da_state))
			else:
				self.logger.warning(" THACommunicator received None status")
			return msg
		except zmq.ContextTerminated:
			self.logger.error("CONTEXT TERMINATED - closing socket")

	def pub_cmd(self, msg=None):
		isinstance(msg, CMD)
		self.logger.debug("SENDING CMD: '{0}'".format(msg))
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
				self.logger.error ('Detected exception in THACommunicator')

class TaskerSub():
	def __init__(self):
		self.sent = threading.Event()
		self.sent.clear()
		self.msg = None
		self.closed = False
	def receive(self, unsubscribe_event):
		if self.sent.wait() and self.closed and unsubscribe_event:
			return self.msg
		self.sent.clear()
	def publish(self, msg):
		self.msg = msg
		self.sent.set()
	def close(self):
		self.closed = True

class TaskerRpc():
	def __init__(self, logger, rpc_name):
		self.callbacks = []
		self.logger = logger
		self.closed = False
		self.rpc_name = rpc_name
		self.lock = threading.Lock()
		self.da_ids = []
		# self.msg = None
	def call(self, req, id=None):
		self.lock.acquire()
		while len(self.callbacks) == 0:
			self.logger.warning("No Callback of TaskerRpc is set")
			time.sleep(0.2)
			if self.closed:
				self.logger.warning("RCP tasker is closed")
				self.lock.release()
				return None
		self.logger.warning("RCP '{0}' send: '{1}'".format(str(self.rpc_name), str(req)))
		if id is not None and str(id) in self.da_ids:
			record = filter(lambda x: x['da_id']==str(id),self.callbacks)[0]
			res = record['callback'](req)
		else:	
			for record in self.callbacks:
				res = record['callback'](req)
		self.logger.warning("RCP '{0}' received: '{1}'".format(str(self.rpc_name ), str(res)))
		self.lock.release()
		return res
		# if self.sent.wait():
		# 	return self.msg
		# self.sent.clear()

	def set_callback(self, callback, id = None):
		self.callbacks.append({'callback':callback, 'da_id':str(id)})
		if id is not None:
			self.da_ids.append(str(id))

	def close(self, id):
		self.logger.warning("callbacks before '{0}' ".format(str(self.callbacks)))
		self.callbacks[:] = [d for d in self.callbacks if d.get('da_id') != str(id)]
		self.logger.warning("callbacks after '{0}' ".format(str(self.callbacks)))

	def close_all(self):
		self.callbacks = []
		self.closed = True
		
class TaskerCommunicator():
	def __init__(self, logger):
		global socket_ports
		self.tha_is_running = True
		self.sockets = []
		self.socket_status = TaskerRpc(logger, rpc_name='status')#init_socket(self.context, socket_ports['status'], 'sub_bind')
		self.sockets.append(self.socket_status)
		self.socket_cmd = TaskerRpc(logger, rpc_name='cmd')#init_socket(self.context, socket_ports['cmd'], 'pub')
		# self.socket_cmd_pub.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_cmd)
		self.socket_life_tick = TaskerRpc(logger, rpc_name='life_tick')#init_socket(self.context, socket_ports['tha_life_tick'], 'pub')
		# self.socket_life_tick_pub.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_life_tick)
		self.socket_cost_cond = TaskerRpc(logger, rpc_name='cost_cond')#init_socket(self.context, socket_ports['cost_cond'], 'client')
		# self.socket_cost_cond.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_cost_cond)
		self.socket_sus_cond =  TaskerRpc(logger, rpc_name='sus_cond')#init_socket(self.context, socket_ports['sus_cond'], 'client')
		# self.socket_sus_cond.setsockopt(zmq.LINGER, 1000)
		self.sockets.append(self.socket_sus_cond)
		self.comm_active = True
		# self.thread_tha_alive = threading.Thread(target=self.thaAliveThread, args=(1,))
		# self.thread_tha_alive.start()
		self.logger = logger
		
		self.logger.debug ("started cmd thread_tha_alive thread")

	def isCommActive(self):
		return self.comm_active
	
	def close(self,id):
		if id == 'harmoniser':
			self.comm_active = False
			for socket in self.sockets:
				socket.close_all()
		else:
			for socket in self.sockets:
				socket.close(id)
		# self.tha_is_running = False
		# while self.tha_is_running:
		# 	try:
		# 		self.pub_life_tick()
		# 		time.sleep(1)
		# 	except Exception as e:
		# 		self.logger.error ('Detected exception in THACommunicator')

	# def pub_life_tick(self):
	# 	self.socket_life_tick.publish( b'' )
	
	def call_status(self, msg):
		self.socket_status.call(msg)

	def set_status(self, callback, id =None):
		return self.socket_status.set_callback(callback, id)

	def call_cmd(self, msg, id =None):
		self.socket_cmd.call(msg, id)

	def set_cmd(self, callback, id =None):
		return self.socket_cmd.set_callback(callback, id)

	def call_cost_cond(self, msg, id =None):
		return self.socket_cost_cond.call(msg, id)

	def set_cost_cond(self, callback, id =None):
		self.socket_cost_cond.set_callback(callback, id)

	def call_sus_cond(self, msg, id =None):
		return self.socket_sus_cond.call(msg, id)

	def set_sus_cond(self, callback, id =None):
		self.socket_sus_cond.set_callback(callback, id)

