#!/usr/bin/env python
import logging 
import rospy
import herbpy


if __name__ == "__main__":   
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)
    
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker')

    herb.DetectHuman(env, orhuman=2)



