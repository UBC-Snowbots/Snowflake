#!/usr/bin/env python
"""
    Entry point for state machine
    Created by William Gu on Feb 23 2019
"""
import rospy
import StateMachine

if __name__== "__main__":

    hp = StateMachine.StateMachine("StateMachine")  # initialize node
    rospy.spin()
