"""
    State Machine Node implementation
    Created by William Gu on Feb 23 2019
"""

import rospy
import smach
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class StateMachine:
    """ Static variables """

    """ Task status codes: 'idle', 'ongoing', 'done' """
    move_status = 'ongoing'
    drill_status = 'idle'

    state_pub = rospy.Publisher('state', String, queue_size=1)  # State publisher

    def __init__(self, node_name):
        rospy.init_node(node_name)

        ''' Setup subscribers for state variables '''
        rospy.Subscriber('/cmd_vel', Twist, self.moveCallBack)  # TODO: change temp subscriber topic to move_status
        rospy.Subscriber('/cmd_vel', Twist, self.drillCallBack)  # TODO: change temp subscriber topic to drill_status

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
        StateMachine.move_status = 'done'  # TODO: update static variable from msg.status (will be a stream)

    def drillCallBack(self, msg):
        StateMachine.drill_status = 'done'  # TODO: update static variable from msg.status


# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done', 'ongoing'])

    def execute(self, userdata):
        StateMachine.state_pub.publish('MOVE')  # publish current state
        if StateMachine.move_status == 'done':
            return 'done'
        else:  # ongoing or idle (should eventually leave idle since current state is being published)
            return 'ongoing'


# define state Drill
class Drill(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done', 'ongoing'])

    def execute(self, userdata):
        StateMachine.state_pub.publish('DRILL')  # publish current state
        if StateMachine.drill_status == 'done':
            return 'done'
        else:
            return 'ongoing'
