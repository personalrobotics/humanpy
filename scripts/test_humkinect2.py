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
    
    
    with env:    
        #Manually add objects to the environment
        table = setupTableEnv.add_table(env, herb)
        table.SetName('table')
        setupTableEnv.set_robot_pose(env, herb, table)  
        
        #glass
        glass = env.ReadKinBodyXMLFile('objects/glass.kinbody.xml')
        glass_pose = numpy.array([[1, 0, 0, -0.5423],
                                  [0, 1, 0, 0.15],
                                  [0, 0, 1, 0.7449],
                                  [0, 0, 0, 1]])  
        glass.SetTransform(glass_pose)
        glass.SetName('glass')
        env.AddKinBody(glass)

        #bowl
        bowl = env.ReadKinBodyXMLFile('objects/bowl.kinbody.xml')
        bowl.SetName('bowl')
        bowl_pose = numpy.array([[1, 0, 0, -0.2423],
                                 [0, 1, 0, 0.15],
                                 [0, 0, 1, 0.7449],
                                 [0, 0, 0, 1]])        
        bowl.SetTransform(bowl_pose)
        env.AddKinBody(bowl)
        
        #palte
        plate = env.ReadKinBodyXMLFile('objects/plate.kinbody.xml')
        plate.SetName('plate')
        plate_pose = numpy.array([[1, 0, 0, -1.5],
                                  [0, 1, 0, 0.30],
                                  [0, 0, 1, 0.7449],
                                  [0, 0, 0, 1]])        
        plate.SetTransform(plate_pose)
        env.AddKinBody(plate)
        
                
        # add a fuze bottle on top of the table       
        #table_aabb = table.ComputeAABB()
        #x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 - 0.5 # middle of table in x
        #y = table_aabb.pos()[1] - table_aabb.extents()[1]*.6 # closer to one side of table in y
        #z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # slightly above table in z (so its not in collision
        #fuze = add_object(env, 'fuze_bottle', 'objects/fuze_bottle.kinbody.xml')
        #fuze_pose = fuze.GetTransform()
        #fuze_pose[:3,3] = numpy.transpose([x, y, z])
        ##fuze_pose = numpy.array([[1,0,0,-0.9834],[0,1,0,0.3911],[0,0,1,0.9135],[0,0,0,1]])
        #fuze.SetTransform(fuze_pose)
        ##env.AddKinBody(fuze)    

        
        #not changing color with this loading function                
        #table = setupTableEnv.add_table(env, herb)
        #table.SetName('table')
        #setupTableEnv.set_robot_pose(env, herb, table)  
        #glass = setupTableEnv.place_glass_on_table(env, table, .7, .2)              
        #bowl = setupTableEnv.place_bowl_on_table(env, table, .45, .6)
        #plate = setupTableEnv.place_plate_on_table(env, table, .2, .5)

    herb.DetectHuman(env, orhuman=2, hum_goal_predic=True)


