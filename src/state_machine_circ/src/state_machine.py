#!/usr/bin/env python
"""
    Entry point for state machine
"""
import rospy
import StateMachine

if __name__== "__main__":

    hp = StateMachine.StateMachine("StateMachine")  # initialize node
    rospy.spin()
