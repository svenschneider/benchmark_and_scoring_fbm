from threading import Condition, Lock, Thread

import rospy
from rockin_scoring.msg import BmBoxState, RefBoxState, ClientState
from FSM import FSM


STATE_UPDATE_RATE = 10 # 10Hz


class Client:
	def _pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
		while (self._fsm.state() != ClientState.END) and (not rospy.is_shutdown()):
			self._state_pub.publish(self._fsm.state(), self._fsm.payload())
			r.sleep()


	def _bmbox_state_cb(self, msg):
		
		self._bmbox_state = msg.state
		self._bmbox_payload = msg.payload

		if (self._bmbox_state == BmBoxState.TRANSMITTING_GOAL) and (self._fsm.state() == ClientState.WAITING_GOAL):
			self._fsm.update(ClientState.EXECUTING_GOAL)

		if (self._bmbox_state == BmBoxState.TRANSMITTING_GOAL) and (self._fsm.state() == ClientState.COMPLETED_GOAL):
			self._fsm.update(ClientState.READY)

		if (self._bmbox_state == BmBoxState.TRANSMITTING_SCORE) and (self._fsm.state() == ClientState.COMPLETED_GOAL):
			self._fsm.update(ClientState.END)


	def _refbox_state_cb(self, msg):
		
		self._refbox_state = msg.state
		self._refbox_payload = msg.payload
		
		if (self._refbox_state == RefBoxState.READY) and (self._fsm.state() == ClientState.CONNECTING):
			self._fsm.update(ClientState.READY)

		if self._refbox_state == RefBoxState.END:
			self._fsm.update(ClientState.END)

	def Connect(self):
		rospy.logdebug("Client.Connect()");
		
		if not self._fsm.check_state(ClientState.START): return
		
		rospy.loginfo("Connecting to the referee box...");
		self._fsm.update(ClientState.CONNECTING)
		self._fsm.wait_transition(ClientState.CONNECTING, ClientState.READY)
	
	
	def RequestGoal(self):
	   	rospy.logdebug("Client.RequestGoal()");

		if not self._fsm.check_state(ClientState.READY): return
	   	
	   	rospy.loginfo("Requesting goal...");
	   	self._fsm.update(ClientState.WAITING_GOAL)
		self._fsm.wait_transition(ClientState.WAITING_GOAL, ClientState.EXECUTING_GOAL)
		return self._bmbox_payload
		

	def SendResult(self, result = None):
	   	rospy.logdebug("Client.SendResult()");
		
		if not self._fsm.check_state(ClientState.EXECUTING_GOAL): return
		
		rospy.loginfo("Sending goal result...");
	   	self._fsm.update(ClientState.COMPLETED_GOAL, result)
	   	self._fsm.wait_transition(ClientState.COMPLETED_GOAL, None)
	
	
	def Running(self):
		return self._fsm.state() != ClientState.END;
	
	
	def __init__(self, node_name):
		self._fsm = FSM(
			('START', 'CONNECTING', 'READY', 'WAITING_GOAL', 'EXECUTING_GOAL', 'COMPLETED_GOAL', 'END'),
			(
				(ClientState.CONNECTING, ClientState.END), # allowed transitions from ClientState.START
				(ClientState.READY, ClientState.END), # allowed transitions from ClientState.CONNECTING
				(ClientState.WAITING_GOAL, ClientState.END), # allowed transitions from ClientState.READY
				(ClientState.EXECUTING_GOAL, ClientState.END), # allowed transitions from ClientState.WAITING_GOAL
				(ClientState.COMPLETED_GOAL, ClientState.END), # allowed transitions from ClientState.EXECUTING_GOAL
				(ClientState.READY, ClientState.END), # allowed transitions from ClientState.COMPLETED_GOAL
				(ClientState.END, ), # allowed transitions from ClientState.END
			),
			ClientState.START
		);

		self._bmbox_state = BmBoxState.START
		self._bmbox_payload = ""
		self._bmbox_condvar = Condition()
		self._refbox_state = RefBoxState.START
		self._refbox_payload = ""
		self._refbox_condvar = Condition()

		rospy.init_node(node_name)
		self._bmbox_state_sub = rospy.Subscriber("bmbox_state", BmBoxState, self._bmbox_state_cb)
		self._refbox_state_sub = rospy.Subscriber("refbox_state", RefBoxState, self._refbox_state_cb)
		self._state_pub = rospy.Publisher("client_state", ClientState, queue_size=10)
		self._pub_thread = Thread(name="pub_thread", target=self._pub_thread)
		self._pub_thread.start()
