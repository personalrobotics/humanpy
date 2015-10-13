#!/usr/bin/env python
import herbpy, humanpy, os
#import prpy.clone
import numpy, time, logging, math
#import IPython
from prpy.rave import add_object
from modular_action_planning.tasks.taskUtils import setupTableEnv
#from  humanpy.action import grasping
#from modular_action_planning.core_actions.robot import MoveHandTo
#from taskPlanningActions.pushGrasp import PushGrasp



if __name__ == "__main__":
    
    logger = logging.getLogger('test_hum_herb')
    logger.setLevel(logging.INFO)
    
    sim = True;
    attach_viewer = "InteractiveMarker"
    env, herb = herbpy.initialize(attach_viewer=attach_viewer, sim=sim)  
    
     
    #with herb:
       #herb.right_arm.SetStiffness(1)
       #herb.left_arm.SetStiffness(1)  
       
    _, human = humanpy.initialize(attach_viewer=attach_viewer, sim=sim, env=env)    
    humanLocation = numpy.array([[ -1. ,  0. ,  0. ,   -1.2],
                                [ 0. ,  0. ,  1. ,  -0.6],
                                [ 0. ,  1. ,  0. ,   0.85],
                                [ 0. ,  0. ,  0. ,   1. ]])
    human.SetTransform(humanLocation)
    human.right_arm.SetActive()
              
    with env:             
        #Manually add objects to the environment
        table = setupTableEnv.add_table(env, herb)
        table.SetName('table')
        setupTableEnv.set_robot_pose(env, herb, table)  
        glass = setupTableEnv.place_glass_on_table(env, table, .2, .7)
        bowl = setupTableEnv.place_bowl_on_table(env, table, .45, .8)
        plate = setupTableEnv.place_plate_on_table(env, table, .3, .8)
        
        # add a fuze bottle on top of the table
        fuze = add_object(env, 'fuze_bottle', 'objects/fuze_bottle.kinbody.xml')
        table_aabb = table.ComputeAABB()
        x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 - 0.5 # middle of table in x
        y = table_aabb.pos()[1] - table_aabb.extents()[1]*.6 # closer to one side of table in y
        z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # slightly above table in z (so its not in collision
        fuze_pose = fuze.GetTransform()
        fuze_pose[:3,3] = numpy.transpose([x, y, z])
        fuze.SetTransform(fuze_pose)
        env.AddKinBody(fuze)
   
    # Grasp the bottle 

    human.right_arm.Grasp(fuze)
    #human.right_arm.PlanToNamedConfiguration('home',execute=True)
    
    herb.right_arm.Grasp(glass)
    #herb.right_arm.PushGrasp(glass, push_required=False)
    #herb.right_arm.PlanToNamedConfiguration('home',execute=True)

    raw_input("press enter to quit!")
        
        