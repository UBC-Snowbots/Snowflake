#!/usr/bin/env python

# Test dependencies
from std_msgs.msg import String
import time
import sys
import unittest
import rospy
import rostest

PKG = 'state_machine_circ'
NAME = 'state_machine_rostest'

class ROSTest(unittest.TestCase):

    def __init__(self, *args):
        super(ROSTest, self).__init__(*args)
        self.curState = 'default'
        self.move_publisher = rospy.Publisher('move_status', String, queue_size=1)
        self.drill_publisher = rospy.Publisher('drill_status', String, queue_size=1)

    # Callback subscriber for current state of state node
    def callback(self, msg):
        self.curState = msg.data

    # Test for move -> drill -> move state
    def test_state_transition(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.Subscriber('state', String, self.callback)

        # Initial Move State
        timeout_t = time.time() + 1.0  # 1 second
        while not rospy.is_shutdown() and time.time() < timeout_t:
            self.move_publisher.publish('ongoing')
            time.sleep(0.1)
        self.assertEqual(self.curState, 'MOVE')

        # Transition to Drill State
        self.move_publisher.publish('idle')
        time.sleep(0.1)  # delay to simulate time to process new state
        self.assertEqual(self.curState, 'DRILL')
        timeout_t = time.time() + 1.0  # 1 second
        while not rospy.is_shutdown() and time.time() < timeout_t:
            self.move_publisher.publish('idle')
            self.drill_publisher.publish('ongoing')
            time.sleep(0.1)
        self.assertEqual(self.curState, 'DRILL')

        # Transition back to Move State
        self.drill_publisher.publish('idle')
        time.sleep(0.1)  # delay to simulate time to process new state
        self.assertEqual(self.curState, 'MOVE')
        timeout_t = time.time() + 1.0  # 1 second
        while not rospy.is_shutdown() and time.time() < timeout_t:
            self.drill_publisher.publish('idle')
            self.move_publisher.publish('ongoing')
            time.sleep(0.1)
        self.assertEqual(self.curState, 'MOVE')


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, ROSTest, sys.argv)