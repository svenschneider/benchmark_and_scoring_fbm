from threading import Condition, Lock, Thread

import rospy
from rockin_scoring.msg import BmBoxState, RefBoxState, ClientState
from FSM import FSM


STATE_UPDATE_RATE = 10 # 10Hz


class BmBox:
	def _pub_thread(self):
		r = rospy.Rate(STATE_UPDATE_RATE)
		
		while (self._fsm.state() != BmBoxState.END) and (not rospy.is_shutdown()):
			self._state_pub.publish(self._fsm.state(), self._fsm.payload())
			r.sleep()


	def _client_state_cb(self, msg):
		
		self._client_state = msg.state
		self._client_payload = msg.payload

		if (self._client_state == ClientState.EXECUTING_GOAL) and (self._fsm.state() == BmBoxState.TRANSMITTING_GOAL):
			self._fsm.update(BmBoxState.EXECUTING_GOAL)
		
		if (self._client_state == ClientState.COMPLETED_GOAL) and (self._fsm.state() == BmBoxState.WAITING_RESULT):
			self._fsm.update(BmBoxState.READY)


	def _refbox_state_cb(self, msg):
		
		self._refbox_state = msg.state
		self._refbox_payload = msg.payload
		
		if (self._refbox_state == RefBoxState.READY) and (self._fsm.state() == BmBoxState.WAITING_CLIENT):
			self._fsm.update(BmBoxState.READY)

		if (self._refbox_state == RefBoxState.EXECUTING_GOAL) and (self._fsm.state() == BmBoxState.WAITING_MANUAL_OPERATION):
			self._fsm.update(BmBoxState.COMPLETED_MANUAL_OPERATION)
			
		if self._refbox_state == RefBoxState.RECEIVED_SCORE:
			self._fsm.update(BmBoxState.END)

		if (self._refbox_state == RefBoxState.END) and (self._refbox_payload == "reason: timeout") and (self._fsm.state() in (BmBoxState.EXECUTING_GOAL, BmBoxState.WAITING_RESULT)):
			self._fsm.update(BmBoxState.READY)
			print "###### TIMEOUT"
			
		if (self._refbox_state == RefBoxState.END) and (self._refbox_payload == "reason: stop") and (self._fsm.state() != BmBoxState.TRANSMITTING_SCORE):
			self._fsm.update(BmBoxState.READY)
			print "###### STOP"


	def WaitClient(self):
		rospy.logdebug("BmBox.WaitClient()")
		
		if not self._fsm.check_state(BmBoxState.START): return
		
		rospy.loginfo("Waiting for client...")
		self._fsm.update(BmBoxState.WAITING_CLIENT)
		self._fsm.wait_transition(BmBoxState.WAITING_CLIENT, BmBoxState.READY)
		
	def ManualOperation(self, message):
		rospy.logdebug("BmBox.ManualOperation()")
		
		if not self._fsm.check_state(BmBoxState.READY):	 return
		
		self._fsm.update(BmBoxState.WAITING_MANUAL_OPERATION, message)
		self._fsm.wait_transition(BmBoxState.WAITING_MANUAL_OPERATION, None)
	
	def SendGoal(self, goal = None):
		rospy.logdebug("BmBox.SendGoal()")
		
		if not ((self._fsm.state() == BmBoxState.READY) or (self._fsm.state() == BmBoxState.COMPLETED_MANUAL_OPERATION)): return

		rospy.loginfo("Sending goal...")
		self._fsm.update(BmBoxState.TRANSMITTING_GOAL, goal)

		self._fsm.wait_transition(BmBoxState.TRANSMITTING_GOAL, BmBoxState.EXECUTING_GOAL)
		rospy.loginfo("Executing goal...")
		

	def WaitResult(self):
	   	rospy.logdebug("BmBox.WaitResult()")
		if not self._fsm.check_state(BmBoxState.EXECUTING_GOAL): return
		
		rospy.loginfo("Waiting for result...")
		self._fsm.update(BmBoxState.WAITING_RESULT)
	   	self._fsm.wait_transition(BmBoxState.WAITING_RESULT, None)
  	
		if self._refbox_payload == "reason: timeout":
			rospy.loginfo("Timeout!")

		if self._refbox_payload == "reason: stop":
			rospy.loginfo("Stop!")
	   	
	   	return self._client_payload


	def SendScore(self, score):
		rospy.logdebug("BmBox.SendScore()")
		
		if not self._fsm.check_state(BmBoxState.READY): return
		
		self._fsm.update(BmBoxState.TRANSMITTING_SCORE, score)
		self._fsm.wait_transition(BmBoxState.TRANSMITTING_SCORE, None)

	def Timeout(self):
		rospy.logdebug("BmBox.Timeout()")
		
		return self._refbox_payload == "reason: timeout"

	def End(self):
		rospy.logdebug("BmBox.End()")
		
		self._fsm.update(BmBoxState.END)

		
	def Running(self):
		return self._refbox_payload != "reason: stop"
	
	
	def __init__(self):
		self._fsm = FSM(
			('START', 'WAITING_CLIENT', 'READY', 'WAITING_MANUAL_OPERATION', 'COMPLETED_MANUAL_OPERATION', 'TRANSMITTING_GOAL', 'EXECUTING_GOAL', 'WAITING_RESULT', 'TRANSMITTING_SCORE', 'END'),
			(
				(BmBoxState.WAITING_CLIENT, BmBoxState.END), # allowed transitions from BmBoxState.START
				(BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_CLIENT
				(BmBoxState.WAITING_MANUAL_OPERATION, BmBoxState.TRANSMITTING_GOAL, BmBoxState.TRANSMITTING_SCORE, BmBoxState.END), # allowed transitions from BmBoxState.READY
				(BmBoxState.COMPLETED_MANUAL_OPERATION, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_MANUAL_OPERATION
				(BmBoxState.TRANSMITTING_GOAL, BmBoxState.END), # allowed transitions from BmBoxState.COMPLETED_MANUAL_OPERATION
				(BmBoxState.EXECUTING_GOAL, BmBoxState.END), # allowed transitions from BmBoxState.TRANSMITTING_GOAL
				(BmBoxState.WAITING_RESULT, BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.EXECUTING_GOAL
				(BmBoxState.READY, BmBoxState.END), # allowed transitions from BmBoxState.WAITING_RESULT
				(BmBoxState.END, ), # allowed transitions from BmBoxState.TRANSMITTING_SCORE
				(BmBoxState.END, ), # allowed transitions from BmBoxState.END
			),
			BmBoxState.START
		)

		self._client_state = ClientState.START
		self._client_payload = ""
		self._refbox_state = RefBoxState.START
		self._refbox_payload = ""

		rospy.init_node("benchmark")
		self._client_state_sub = rospy.Subscriber("client_state", ClientState, self._client_state_cb)
		self._refbox_state_sub = rospy.Subscriber("refbox_state", RefBoxState, self._refbox_state_cb)
		self._state_pub = rospy.Publisher("bmbox_state", BmBoxState, queue_size=10)
		self._pub_thread = Thread(name="pub_thread", target=self._pub_thread)
		self._pub_thread.start()
