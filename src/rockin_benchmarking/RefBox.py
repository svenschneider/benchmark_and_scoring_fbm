from threading import Condition, Lock, Thread

import rospy
from rockin_benchmarking.msg import BmBoxState, ClientState, RefBoxState
from FSM import FSM


STATE_UPDATE_RATE = 10 # 10Hz


class RefBox:
	def _pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
		while (self._fsm.state() != RefBoxState.END) and (not rospy.is_shutdown()):
			self._state_pub.publish(self._fsm.state(), self._fsm.payload())
			r.sleep()


	def _client_state_cb(self, msg):
		if msg.state == self._client_state:
			return
		
		self._client_state = msg.state
		self._client_payload = msg.payload
		
		if (self._client_state == ClientState.CONNECTING) and (self._fsm.state() == RefBoxState.WAITING_CLIENT):
			self._fsm.update(RefBoxState.READY)

		if (self._client_state == ClientState.EXECUTING_GOAL) and (self._fsm.state() in (RefBoxState.WAITING_CLIENT, RefBoxState.EXECUTING_MANUAL_OPERATION)):
			self._fsm.update(RefBoxState.EXECUTING_GOAL)

		if (self._client_state == ClientState.COMPLETED_GOAL) and (self._fsm.state() == RefBoxState.EXECUTING_GOAL):
			self._fsm.update(RefBoxState.READY)


	def _bmbox_state_cb(self, msg):
		if msg.state == self._bmbox_state:
			return
		
		self._bmbox_state = msg.state
		self._bmbox_payload = msg.payload

		if (self._bmbox_state == BmBoxState.EXECUTING_GOAL) and (self._fsm.state() == RefBoxState.READY):
			self._fsm.update(RefBoxState.EXECUTING_GOAL)
		
		if (self._bmbox_state == BmBoxState.READY) and (self._fsm.state() == RefBoxState.EXECUTING_GOAL):
			self._fsm.update(RefBoxState.READY)

		if self._bmbox_state == BmBoxState.TRANSMITTING_SCORE:
			self._fsm.update(RefBoxState.RECEIVED_SCORE)

	def WaitClient(self):
		rospy.logdebug("RefBox.WaitClient()");
		
		if not self._fsm.check_state(RefBoxState.START): return
		
		rospy.loginfo("Waiting for client...");
		self._fsm.update(RefBoxState.WAITING_CLIENT)
		self._fsm.wait_transition(RefBoxState.WAITING_CLIENT, RefBoxState.READY)
		
	def Running(self):
		rospy.logdebug("RefBox.Running()");
		
		if self._fsm.state() not in (RefBoxState.RECEIVED_SCORE, RefBoxState.END):
			return True
		else:
			return False
		
	def RequestedManualOperation(self):
		rospy.logdebug("RefBox.RequestedManualOperation()");
		
		if (self._bmbox_state == BmBoxState.WAITING_MANUAL_OPERATION):
			self._fsm.update(RefBoxState.EXECUTING_MANUAL_OPERATION)
			return self._bmbox_payload
		else:
			return False


	def ConfirmManualOperation(self):
		rospy.logdebug("RefBox.ConfirmManualOperation()");
		
		if not self._fsm.check_state(RefBoxState.EXECUTING_MANUAL_OPERATION): return
		
		self._fsm.update(RefBoxState.EXECUTING_GOAL)
			
			
	def GetScore(self):
	   	rospy.logdebug("RefBox.SendResult()");
		
		if not self._fsm.check_state(RefBoxState.RECEIVED_SCORE): return
		
		self._fsm.update(RefBoxState.END)
			   		   	
	   	return self._bmbox_payload


	def End(self):
		rospy.logdebug("RefBox.End()");
		
		self._fsm.update(RefBoxState.END)
	
	
	def __init__(self, node_name):
		self._fsm = FSM(
			('START', 'WAITING_CLIENT', 'READY', 'EXECUTING_MANUAL_OPERATION', 'EXECUTING_GOAL', 'RECEIVED_SCORE', 'END'),
			(
				(RefBoxState.WAITING_CLIENT, RefBoxState.END), # allowed transitions from RefBoxState.START
				(RefBoxState.READY, RefBoxState.END), # allowed transitions from RefBoxState.WAITING_CLIENT
				(RefBoxState.EXECUTING_MANUAL_OPERATION, RefBoxState.EXECUTING_GOAL, RefBoxState.RECEIVED_SCORE, RefBoxState.END), # allowed transitions from RefBoxState.READY
				(RefBoxState.EXECUTING_GOAL, RefBoxState.END), # allowed transitions from RefBoxState.EXECUTING_MANUAL_OPERATION
				(RefBoxState.READY, RefBoxState.RECEIVED_SCORE, RefBoxState.END), # allowed transitions from RefBoxState.EXECUTING_GOAL
				(RefBoxState.END, ), # allowed transitions from RefBoxState.RECEIVED_SCORE
				(RefBoxState.END, ), # allowed transitions from RefBoxState.END
			),
			RefBoxState.START
		);

		self._client_state = ClientState.START
		self._client_payload = ""
		self._bmbox_state = RefBoxState.START
		self._bmbox_payload = ""

		rospy.init_node(node_name)
		self._client_state_sub = rospy.Subscriber("client_state", ClientState, self._client_state_cb)
		self._bmbox_state_sub = rospy.Subscriber("bmbox_state", BmBoxState, self._bmbox_state_cb)
		self._state_pub = rospy.Publisher("refbox_state", RefBoxState, queue_size=10)
		self._pub_thread = Thread(name="pub_thread", target=self._pub_thread)
		self._pub_thread.start()
