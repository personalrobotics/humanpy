#!/usr/bin/env python
import argparse
import logging 
import argparse
import rospy
import herbpy
from modular_action_planning.tasks.taskUtils import setupTableEnv

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='load human skeleton in herb env')
    parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
    args = parser.parse_args(rospy.myargv()[1:])
    
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)

    env, herb = herbpy.initialize(attach_viewer=True, sim=True) 

    herb.DetectHuman(env, orhuman=2)


