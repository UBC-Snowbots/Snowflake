#!/usr/bin/env python
"""
    Hello Python!
"""
import rospy
import state_machine_circ

if __name__== "__main__":
    rospy.init_node('state_machine_circ') # Resgistering node in ros master

    hp = state_machine_circ.StateMachine()
    hp.spin()