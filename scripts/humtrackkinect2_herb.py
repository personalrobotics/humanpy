#!/usr/bin/env python
import logging 
import rospy
import herbpy


if __name__ == "__main__":   
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)
    
    rospy.init_node('humkin2') 
    segway_sim = rospy.get_param("~seg_sim");    
    
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker', segway_sim=segway_sim, talker_sim=False, sim=True)

    if segway_sim==True: 
        refsys = '/head/skel_depth_frame2' 
    else: 
        refsys = '/head/skel_depth_frame'
    
    herb.DetectHuman(env, segway_sim=segway_sim, kin_frame=refsys)
