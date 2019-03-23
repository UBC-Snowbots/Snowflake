#
# Hello Python Class to show simple python class
#
#
import rospy
import smach
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class StateMachine:
    """ Member variables """
    arrived = False
    drilling = False

    def __init__(self, node_name):
        rospy.init_node(node_name)  # Register node

        ''' Setup subscribers for state variables '''
        rospy.Subscriber('/cmd_vel', Twist, self.arrivedCallBack)  # TODO: change temp subscriber topic
        rospy.Subscriber('/cmd_vel', Twist, self.drillingCallBack)

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['course_complete'])  # TODO: Currently unreachable, may need a final state

        # Open the container
        with sm:
            # Add states to the container (1st state listed = initial state?)
            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'en_route': 'MOVE',
                                                'arrived': 'DRILL'})
            smach.StateMachine.add('DRILL', Drill(),
                                   transitions={'drilling': 'DRILL',
                                                'done_drilling': 'MOVE'})

        # Execute SMACH plan
        sm.execute()

    """ Call backs """

    def arrivedCallBack(self, msg):
        StateMachine.arrived = not StateMachine.arrived  # Note that the class static variable is switched

    def drillingCallBack(self, msg):
        StateMachine.drilling = not StateMachine.drilling


# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['en_route', 'arrived'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE')
        rospy.loginfo(StateMachine.arrived)
    # Need to get current status of robot somehow
        if StateMachine.arrived:
            return 'arrived'
        else:
            return 'en_route'


# define state Drill
class Drill(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['drilling', 'done_drilling'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DRILL')
        if StateMachine.drilling:
            return 'drilling'
        else:
            return 'done_drilling'
