from mbot_robot_class_ros import mbot as mbot_class


import moveit_commander

import rospy

# para aprender a fazer um .py file + node (executavel) paara mover o braco para uma posicao ja definida
# isto nao e preciso however pq ja existe nos manipulation_states

class Move_arm(object):

	def __init__(self):

        
		arm = rospy.get_param('~arm', 'left_arm')
		self.arm = moveit_commander.MoveGroupCommander(arm)

		self.mbot = mbot_class.mbotRobot(enabled_components=['hri', 'perception', 'manipulation'])

 	def moveit_goto_pose(self, arm_configuration):

 		self.mbot.manipulation.go_to_pose(arm_configuration, wait=True)

	def start(self):
       
		self.moveit_goto_pose('candle')



def main():
    rospy.init_node("move_arm", anonymous=True)
    objeto = Move_arm()
    objeto.start()