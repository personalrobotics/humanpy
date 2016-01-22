#!/usr/bin/env python
import logging 
import numpy
import rospy
import herbpy
#from modular_action_planning.tasks.taskUtils import setupTableEnv
import openravepy
import random
import copy
from prpy.planning import VectorFieldPlanner, CHOMPPlanner, OMPLPlanner
from prpy import viz

ACTIONS = ['stamp','grasp']

REF_OBJ = ['glass1','glass2','glass3','box1','box2','box3', 
           'box5','box6','box7','box8','box9']

if __name__ == "__main__":   
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)
    
    #print '*************************TEST1***************************'
    
    rospy.init_node('humherb_kin2', anonymous=True)
    
    herb_sim = rospy.get_param("~herb_sim");
    obj_detec = rospy.get_param("~obj_detec");
    
    segway_sim = True  #1) if true in herbrobot use map; oterwise use herb_base lines 195 and 255  2) in updatetf.py line 14 coherent
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker', sim=herb_sim, segway_sim=segway_sim)
    
    action = ACTIONS[0]
    if obj_detec == True:
        herb.DetectObjects()
        #TODO: check the coherence among the detected objects and the action  
        res = raw_input("Are detected objects in the correct spot? y/n: ")
        while res is not 'y':
            herb.DetectObjects()
            res = raw_input("Are detected objects in the correct spot? y/n: \n")        
            
        #TODO:test with obj different from BOX
        table_z = None
        for obj in env.GetBodies():
            if 'table' in obj.GetName():
                table_aabb = obj.ComputeAABB()
                table_z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 
        print table_z
        for obj in env.GetBodies():
            if 'table' not in obj.GetName() and 'herb' not in obj.GetName():
                obj_tra = obj.GetTransform()
                obj_aabb = obj.ComputeAABB()
                obj_tra[2,3] = table_z + obj_aabb.extents()[2]/2
                obj.SetTransform(obj_tra)   
    
    if obj_detec == False and herb_sim == True:
        with env:            
            table = env.ReadKinBodyXMLFile('objects/table.kinbody.xml')       
            table_pose = numpy.array([[0., 0.,  1., 1],
                                    [1., 0., 0., 0.],
                                    [0., 1., 0., 0.0], 
                                    [0., 0.,  0., 1.]])
            table.SetTransform(table_pose)
            table.SetName('table')
            env.AddKinBody(table)        
            
            table_aabb = table.ComputeAABB()
            x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 # middle of table in x
            y = table_aabb.pos()[1] + table_aabb.extents()[1]*.6 # closer to one side of table in y
            z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # slightly above table in z (so its not in collision
    
            if action == ACTIONS[0]:         
                #box1
                box1 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')    
                box1_aabb = box1.ComputeAABB()
                box1_pose = numpy.array([[0, 1, 0, 0.],
                                         [-1, 0, 0, 0.],
                                         [0, 0, 1, 0.],
                                         [0, 0, 0, 1]])  
                #box1_pose = numpy.identity(4)  
                box1_pose[:3,3] = numpy.transpose([x - 0.1 , y - 0.7 , z + box1_aabb.extents()[2]])
                #box1_pose = numpy.array([[1, 0, 0, -0.55],
                            #[0, 1, 0, 0.55],
                            #[0, 0, 1, 0.7449],
                            #[0, 0, 0, 1]])  
                box1.SetTransform(box1_pose)
                box1.SetName('box1')
                env.AddKinBody(box1)

                #box2
                box2 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')
                box2_aabb = box2.ComputeAABB()
                #box2_pose = numpy.identity(4)  
                box2_pose = numpy.array([[0, 1, 0, 0.],
                                         [-1, 0, 0, 0.],
                                         [0, 0, 1, 0.],
                                         [0, 0, 0, 1]])  
                box2_pose[:3,3] = numpy.transpose([x - 0.1, y - 1.0, z + box2_aabb.extents()[2]])  
                #box2_pose = numpy.array([[1, 0, 0, -0.8],
                                            #[0, 1, 0, 0.55],
                                            #[0, 0, 1, 0.7449],
                                            #[0, 0, 0, 1]])   
                box2.SetTransform(box2_pose)
                box2.SetName('box2')
                env.AddKinBody(box2)

                #box3
                box3 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')
                box3_aabb = box3.ComputeAABB()
                #box3_pose = numpy.identity(4)  
                box3_pose = numpy.array([[0, 1, 0, 0.],
                                         [-1, 0, 0, 0.],
                                         [0, 0, 1, 0.],
                                         [0, 0, 0, 1]])  
                box3_pose[:3,3] = numpy.transpose([x - 0.1 , y - 1.3 , z + box3_aabb.extents()[2]]) 
                #box3_pose = numpy.array([[1, 0, 0, -1.2],
                                            #[0, 1, 0, 0.50], #0.60
                                            #[0, 0, 1, 0.7449],  
                                            #[0, 0, 0, 1]])   
                box3.SetTransform(box3_pose)
                box3.SetName('box3')
                env.AddKinBody(box3)    
    
            if action == ACTIONS[1]:     
                #glass
                glass = env.ReadKinBodyXMLFile('objects/plastic_glass.kinbody.xml')       
                glass_pose = numpy.identity(4)  
                glass_pose[:3,3] = numpy.transpose([x - 0.1 , y - 0.7 , z + 0.02])
                glass.SetTransform(glass_pose)
                glass.SetName('glass1')
                env.AddKinBody(glass)

                #glass
                glass2 = env.ReadKinBodyXMLFile('objects/plastic_glass.kinbody.xml')
                glass2_pose = numpy.identity(4)  
                glass2_pose[:3,3] = numpy.transpose([x - 0.1, y - 1.0, z + 0.02])  
                glass2.SetTransform(glass2_pose)
                glass2.SetName('glass2')
                env.AddKinBody(glass2)

                #glass
                glass3 = env.ReadKinBodyXMLFile('objects/plastic_glass.kinbody.xml')
                glass3_pose = numpy.identity(4)  
                glass3_pose[:3,3] = numpy.transpose([x - 0.1 , y - 1.3 , z + 0.02]) 
                glass3.SetTransform(glass3_pose)
                glass3.SetName('glass3')
                env.AddKinBody(glass3)

                ##bowl
                #bowl = env.ReadKinBodyXMLFile('objects/bowl.kinbody.xml')
                #bowl.SetName('bowl')
                #bowl_pose = numpy.array([[1, 0, 0, -0.2423],
                                        #[0, 1, 0, 0.15],
                                        #[0, 0, 1, 0.7449],
                                        #[0, 0, 0, 1]])        
                #bowl.SetTransform(bowl_pose)
                #env.AddKinBody(bowl)
                
                ##palte
                #plate = env.ReadKinBodyXMLFile('objects/plate.kinbody.xml')
                #plate.SetName('plate')
                #plate_pose = numpy.array([[1, 0, 0, -1.5],
                                        #[0, 1, 0, 0.30],
                                        #[0, 0, 1, 0.7449],
                                        #[0, 0, 0, 1]])        
                #plate.SetTransform(plate_pose)
                #env.AddKinBody(plate)

    if action == ACTIONS[0]:        
        herb.right_arm.SetActive()
        #herb.GetActiveManipulator().hand.OpenHand()
        with env:
            ee_r = herb.GetActiveManipulator().GetEndEffectorTransform()
            stamp = env.ReadKinBodyXMLFile('objects/stamp.kinbody.xml')
            pose_hand = numpy.array([0.0, -0.05, 0.35, 1.])  #0.255
            pose_world = numpy.dot(ee_r, pose_hand)
            ee_r[0:3,3] =  pose_world[0:3]       
            stamp.SetTransform(ee_r)
            stamp.Enable(False)
            stamp.SetName('stamp')
            stm = env.AddKinBody(stamp)
        #p = openravepy.KinBody.SaveParameters
        #with herb.CreateRobotStateSaver(p.ActiveManipulator):
            #herb.SetActiveManipulator(manip)
            #herb.Grab(env.GetKinBody('stamp'))   
        herb.Grab(env.GetKinBody('stamp'))  
        #raw_input("Press enter")  
        if herb_sim == True:
            herb.GetActiveManipulator().hand.CloseHand()
        
    
    herb.left_arm.SetStiffness(1)     
    herb.left_arm.SetActive()
    herb.PlanToNamedConfiguration('relaxed_home', execute=True)    
    herb.right_arm.SetStiffness(1) 
    herb.right_arm.SetActive()
    herb.PlanToNamedConfiguration('home', execute=True)
    
    tsr_box_distance = 0.04
    body_list = [ obj for obj in env.GetBodies() if 'box' in obj.GetName() ]  
  
    for obj in env.GetBodies():
        for ref_name in REF_OBJ:                
            if ref_name in obj.GetName():
                with env:
                    tsr_list = herb.tsrlibrary(obj, action)

                with viz.RenderTSRList(tsr_list, env, render=True):
                    herb.GetActiveManipulator().PlanToTSR(tsr_list, num_attempts=20, execute=True)

                logger.info('Approaching the object')
                if herb_sim == True:
                    traj = VectorFieldPlanner().PlanToEndEffectorOffset(herb,
                                                                        numpy.array([0,0,-1]),
                                                                        tsr_box_distance,
                                                                        position_tolerance=0.04,
                                                                        angular_tolerance=0.15)            
                    herb.ExecutePath(traj, timeout=5.0) 
                else:
                    herb.GetActiveManipulator().MoveUntilTouch(numpy.array([0,0,-1]),  
                                                                tsr_box_distance, 
                                                                ignore_collisions=body_list,
                                                                max_force=10.0)
                    
                logger.info('Deproaching the object')
                traj = VectorFieldPlanner().PlanToEndEffectorOffset(herb,
                                                                    numpy.array([0,0,1]),
                                                                    tsr_box_distance,
                                                                    position_tolerance=0.04,
                                                                    angular_tolerance=0.15)            

                herb.ExecutePath(traj, timeout=5.0) 


    herb.PlanToNamedConfiguration('home', execute=True)
    logger.info('Task done!!!!')
    #rospy.on_shutdown('humherb_kin2')
    
    raw_input('press enter')
