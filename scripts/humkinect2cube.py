#!/usr/bin/env python
import argparse
import logging 
import argparse
import numpy
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

    #env, herb = herbpy.initialize(attach_viewer=True, sim=True) 
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker', sim=True) 
    
    
    indices, values = herb.configurations.get_configuration('relaxed_home') # Faster for testing
    #values[7:14] = numpy.array([-0.2, 1.60, 2.0,  2.0222084,  
    #                            1.0, 0.0, 0.0])
    values[7:14] = numpy.array([numpy.pi/2, 1.30, -2.9, 1.92, 0.0, 0.0, 0.0])
    herb.SetDOFValues(values=values, dofindices=indices)  
    
    herb.right_arm.SetActive()   

    
    with env:
        ee_r = herb.GetActiveManipulator().GetEndEffectorTransform()
        stamp = env.ReadKinBodyXMLFile('objects/stamp.kinbody.xml')
        pose_hand = numpy.array([0.0, -0.01, 0.255, 1.])
        pose_world = numpy.dot(ee_r, pose_hand)
        ee_r[0:3,3] =  pose_world[0:3]       
        stamp.SetTransform(ee_r)
        stamp.SetName('stamp')
        env.AddKinBody(stamp)
        herb.Grab(env.GetKinBody('stamp'))
    
    herb.GetActiveManipulator().hand.CloseHand()
   
       
    with env:     
        #Manually add objects to the environment
        table = setupTableEnv.add_table(env, herb)
        table.SetName('table')
        setupTableEnv.set_robot_pose(env, herb, table)  
       
        #glass
        box1 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')       
        box1_pose = numpy.array([[1, 0, 0, -0.55],
                                  [0, 1, 0, 0.55],
                                  [0, 0, 1, 0.7449],
                                  [0, 0, 0, 1]])  
        box1.SetTransform(box1_pose)
        box1.SetName('box1')
        env.AddKinBody(box1)

        #glass
        box2 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')
        box2_pose = numpy.array([[1, 0, 0, -0.8],
                                 [0, 1, 0, 0.55],
                                 [0, 0, 1, 0.7449],
                                 [0, 0, 0, 1]])   
        box2.SetTransform(box2_pose)
        box2.SetName('box2')
        env.AddKinBody(box2)

        #glass
        box3 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')
        box3_pose = numpy.array([[1, 0, 0, -1.2],
                                  [0, 1, 0, 0.50], #0.60
                                  [0, 0, 1, 0.7449],  
                                  [0, 0, 0, 1]])   
        box3.SetTransform(box3_pose)
        box3.SetName('box3')
        env.AddKinBody(box3)



    herb.DetectHuman(env, orhuman=2, hum_goal_predic=True)


