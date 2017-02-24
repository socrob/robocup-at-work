#!/usr/bin/env python

import rospy
from smach import State


class CounterState(State):
    def __init__(self, count_to):
        State.__init__(self, outcomes=['overflow', 'counting'])
        self.limit = count_to
        self.counter = 0

    def execute(self, userdata):
        self.counter += 1
        if self.counter == self.limit:
            self.counter = 0
            return 'overflow'
        else:
            return 'counting'

class CounterStateObject(CounterState):
	def execute(self, userdata):
		ret = CounterState.execute(self, userdata)
		save_pcl = rospy.get_param('save_pcl')
		if save_pcl == 'no':
			self.counter = 0
			return 'overflow'
		return ret
