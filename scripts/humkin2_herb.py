#!/usr/bin/env python
import logging 
import numpy
import rospy
import herbpy
#from modular_action_planning.tasks.taskUtils import setupTableEnv
import openravepy



if __name__ == "__main__":   
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)
    
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker', sim=False)

    herb.DetectHuman(env)