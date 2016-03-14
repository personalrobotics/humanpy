#!/usr/bin/env python
import os
import numpy
import time
import logging
import math
from prpy.rave import add_object
from modular_action_planning.tasks.taskUtils import setupTableEnv
import herbpy
import humanpy

if __name__ == "__main__":
    
    logger = logging.getLogger('test_humherb')
    logger.setLevel(logging.INFO)
    
    sim = True
    attach_viewer = True
    env, herb = herbpy.initialize(attach_viewer=attach_viewer, sim=sim)  

    _, human = humanpy.initialize(attach_viewer=attach_viewer, sim=sim, env=env)    
    humanLocation = numpy.array([[ -1. ,  0. ,  0. ,   -0.8],  #-1.2
                                [ 0. ,  0. ,  1. ,  -0.4],   #-0.5
                                [ 0. ,  1. ,  0. ,   0.85],
                                [ 0. ,  0. ,  0. ,   1. ]])
    human.SetTransform(humanLocation)
    human.right_arm.SetActive()

    with env:             
        #Manually add objects to the environment
        table = setupTableEnv.add_table(env, herb)
        table.SetName('table')
        setupTableEnv.set_robot_pose(env, herb, table)  
        glass = setupTableEnv.place_glass_on_table(env, table, .7, .4)
        bowl = setupTableEnv.place_bowl_on_table(env, table, .45, .6)
        plate = setupTableEnv.place_plate_on_table(env, table, .2, .5)
        
        # add a fuze bottle on top of the table
        fuze = add_object(env, 'fuze_bottle', 'objects/fuze_bottle.kinbody.xml')
        table_aabb = table.ComputeAABB()
        x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 - 0.5 # middle of table in x
        y = table_aabb.pos()[1] - table_aabb.extents()[1]*.6 # closer to one side of table in y
        z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # slightly above table in z (so its not in collision
        fuze_pose = fuze.GetTransform()
        fuze_pose[:3,3] = numpy.transpose([x, y, z])
        fuze.SetTransform(fuze_pose)
   
    # Grasp the bottle 
    #human.right_arm.Grasp(fuze)
    herb.right_arm.Grasp(bowl)

    raw_input("press enter to quit!")
 
        