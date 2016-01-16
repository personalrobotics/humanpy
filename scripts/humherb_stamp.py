#!/usr/bin/env python
import logging 
import numpy
import rospy
import herbpy
#from modular_action_planning.tasks.taskUtils import setupTableEnv
import openravepy
import random
import copy
from prpy.planning import VectorFieldPlanner

ACTIONS = ['stamp','grasp']

REF_OBJ = ['glass1','glass2','glass3','box1','box2','box3', 
           'box5','box6','box7','box8','box9']

if __name__ == "__main__":   
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)
    
    rospy.init_node('humherb_kin2', anonymous=True)
    
    herb_sim = rospy.get_param("~herb_sim");
    bagfile = rospy.get_param("~bagfile");
    
    segway_sim = True  #1) if true in herbrobot use map; oterwise use herb_base lines 195 and 255  2) in updatetf.py line 14 coherent
    env, herb = herbpy.initialize(attach_viewer='InteractiveMarker', sim=herb_sim, segway_sim=segway_sim)
    
    doc = False
    action = ACTIONS[0]
    if bagfile == False:
        doc = herb.DetectObjects()
        #TODO: check the coherence among the detected objects and the action  
        res = raw_input("Are detected objects in the correct spot? y/n: ")
        while res is not 'y':
            doc = herb.DetectObjects()
            res = raw_input("Are detected objects in the correct spot? y/n: \n")
    
    if doc == False and herb_sim == True:
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
                box1_pose = numpy.identity(4)  
                box1_pose[:3,3] = numpy.transpose([x - 0.1 , y - 0.7 , z + 0.02])
                #box1_pose = numpy.array([[1, 0, 0, -0.55],
                            #[0, 1, 0, 0.55],
                            #[0, 0, 1, 0.7449],
                            #[0, 0, 0, 1]])  
                box1.SetTransform(box1_pose)
                box1.SetName('box1')
                env.AddKinBody(box1)

                #box2
                box2 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')
                box2_pose = numpy.identity(4)  
                box2_pose[:3,3] = numpy.transpose([x - 0.1, y - 1.0, z + 0.02])  
                #box2_pose = numpy.array([[1, 0, 0, -0.8],
                                            #[0, 1, 0, 0.55],
                                            #[0, 0, 1, 0.7449],
                                            #[0, 0, 0, 1]])   
                box2.SetTransform(box2_pose)
                box2.SetName('box2')
                env.AddKinBody(box2)

                #box3
                box3 = env.ReadKinBodyXMLFile('objects/box.kinbody.xml')
                box3_pose = numpy.identity(4)  
                box3_pose[:3,3] = numpy.transpose([x - 0.1 , y - 1.3 , z + 0.02]) 
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
        herb.GetActiveManipulator().hand.OpenHand()
        with env:
            ee_r = herb.GetActiveManipulator().GetEndEffectorTransform()
            stamp = env.ReadKinBodyXMLFile('objects/stamp.kinbody.xml')
            pose_hand = numpy.array([0.0, -0.05, 0.21, 1.])  #0.255
            pose_world = numpy.dot(ee_r, pose_hand)
            ee_r[0:3,3] =  pose_world[0:3]       
            stamp.SetTransform(ee_r)
            stamp.SetName('stamp')
            env.AddKinBody(stamp)
            herb.Grab(env.GetKinBody('stamp'))  
        #raw_input("Press enter")           
        herb.GetActiveManipulator().hand.CloseHand()
        
    
    herb.left_arm.SetStiffness(1)     
    herb.left_arm.SetActive()
    herb.PlanToNamedConfiguration('relaxed_home', execute=True)    
    herb.right_arm.SetStiffness(1) 
    herb.right_arm.SetActive()
    herb.PlanToNamedConfiguration('home', execute=True)
    
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
    tsr_box_distance = 0.04
    
    tsrnb = 30
    for obj in env.GetBodies():
        for ref_name in REF_OBJ:                
            if ref_name in obj.GetName():
                with env:
                    tsr_list = herb.tsrlibrary(obj, action)
                    tsr_chain_idx = random.randint(0, len(tsr_list) - 1)                  
                    tsr_chain = tsr_list[tsr_chain_idx]                                           
                    for idx in range(tsrnb):
                        sample = tsr_chain.sample() 
                        config_tsr = None
                        config_stamp = None
                        with env:
                            config_tsr = herb.GetActiveManipulator().FindIKSolution(sample, filter_options) # will return None if no config can be found
                            sample_stamp = copy.deepcopy(sample)
                            sample_stamp[2,3] -= tsr_box_distance
                            config_stamp = herb.GetActiveManipulator().FindIKSolution(sample_stamp, filter_options)                            
                        
                        if config_tsr is not None and config_stamp is not None: 
                            traj = herb.planner.PlanToEndEffectorPose(herb, sample)

                            if traj.GetNumWaypoints() > 0:                                    
                                herb.ExecutePath(traj)
                                print '****here'
                                logger.info('Approching the object')                            
                                traj = VectorFieldPlanner().PlanToEndEffectorOffset(herb,
                                                                                    numpy.array([0,0,-1]),
                                                                                    tsr_box_distance, 
                                                                                    position_tolerance=0.04,
                                                                                    angular_tolerance=0.15)  
                                if traj.GetNumWaypoints() > 0:
                                    herb.ExecutePath(traj)
                                    
                                    logger.info('Deproaching the object')
                                    traj = VectorFieldPlanner().PlanToEndEffectorOffset(herb,
                                                                                        numpy.array([0,0,1]),
                                                                                        tsr_box_distance,
                                                                                        position_tolerance=0.04,
                                                                                        angular_tolerance=0.15)
                                    if traj.GetNumWaypoints() > 0:
                                        herb.ExecutePath(traj) 
                                        break
                        



    herb.PlanToNamedConfiguration('home', execute=True)
    logger.info('Task done!!!!')
    #rospy.on_shutdown('humherb_kin2')
    
    raw_input('press enter')
