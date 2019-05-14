"""
    State Machine Node implementation
    Created by William Gu on Feb 23 2019
"""

import rospy
import smach
from std_msgs.msg import String


class StateMachine:

    """ Task status: ongoing, idle, next """
    """ Various task nodes should output only ongoing and idle """
    move_status = 'next'  # indicates next expected state
    drill_status = 'idle'

    state_pub = rospy.Publisher('state', String, queue_size=1)  # State publisher

    def __init__(self, node_name):
        rospy.init_node(node_name)

        ''' Setup subscribers for state variables '''
        rospy.Subscriber('move_status', String, self.moveCallBack)
        rospy.Subscriber('drill_status', String, self.drillCallBack)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=[])

        # Open the container
        with sm:
            # Add states to the container (1st state listed = initial state?)
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'ongoing': 'MOVE',
                                                'done': 'DRILL'})
            smach.StateMachine.add('DRILL', Drill(),
                                   transitions={'ongoing': 'DRILL',
                                                'done': 'MOVE'})

        # Execute SMACH plan
        sm.execute()

    """ Call backs """
    def moveCallBack(self, msg):
        if msg.data == 'ongoing':  # Override 'next' status when it starts
            StateMachine.move_status = msg.data
        elif StateMachine.move_status != 'next':  # Otherwise do NOT update if next state
            StateMachine.move_status = msg.data

    def drillCallBack(self, msg):
        if msg.data == 'ongoing':
            StateMachine.drill_status = msg.data
        elif StateMachine.drill_status != 'next':
            StateMachine.drill_status = msg.data


# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done', 'ongoing'])

    def execute(self, userdata):
        StateMachine.state_pub.publish('MOVE')  # publish current state
        if StateMachine.move_status == 'idle':
            StateMachine.drill_status = 'next'  # Setup next expected status
            return 'done'
        else:  # ongoing or next
            return 'ongoing'


# define state Drill
class Drill(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done', 'ongoing'])

    def execute(self, userdata):
        StateMachine.state_pub.publish('DRILL')
        if StateMachine.drill_status == 'idle':
            StateMachine.move_status = 'next'
            return 'done'
        else:
            return 'ongoing'
