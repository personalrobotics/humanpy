#!/usr/bin/env python
import logging 
import rospy
import herbpy


if __name__ == "__main__":   
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)
    
    rospy.init_node('humkin2') 
    segway_sim = rospy.get_param("~seg_sim");    
    
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker', segway_sim=segway_sim)

    herb.DetectHuman(env, orhuman=2, segway_sim=segway_sim)



