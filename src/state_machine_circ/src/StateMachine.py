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
    movement_status = 'ongoing'
    drill_status = 'idle'

    state_pub = rospy.Publisher('state', String, queue_size=1)  # State publisher

    def __init__(self, node_name):
        rospy.init_node(node_name)  # Register node

        ''' Setup subscribers for state variables '''
        rospy.Subscriber('/cmd_vel', Twist, self.movementCallBack)  # TODO: change temp subscriber topic
        rospy.Subscriber('/cmd_vel', Twist, self.drillCallBack)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['course_complete'])  # TODO: Currently unreachable, may need a final state

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

    def movementCallBack(self, msg):
        StateMachine.movement_status = 'done'  # TODO: update static variable from msg.status (will be a stream)

    def drillCallBack(self, msg):
        StateMachine.drill_status = 'done'  # TODO: update static variable from msg.status


# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done', 'ongoing'])

    def execute(self, userdata):
        StateMachine.state_pub.publish('MOVE')  # publish current state
        if StateMachine.movement_status == 'done':
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
