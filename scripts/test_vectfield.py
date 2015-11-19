#!/usr/bin/env python
import rospy
import math
import numpy
import os
import herbpy
import prpy
import logging 
import copy
import openravepy
from prpy.planning import Sequence
from prpy.planning import (
    TSRPlanner,
    VectorFieldPlanner,
    CHOMPPlanner
)

#from modular_action_planning.tasks.taskUtils import setupTableEnv
from catkin.find_in_workspaces import find_in_workspaces

   
    
if __name__ == "__main__":
    
    
    # ===========================
    #   ENVIRONMENT
    # ===========================
       
    objects_path = find_in_workspaces(
    search_dirs=['share'],
    project='pr_ordata',
    path='data/objects',
    first_match_only=True)[0]

    #env, herb = herbpy.initialize(attach_viewer=True, sim=True) 
    env, robot = herbpy.initialize(attach_viewer='InteractiveMarker', sim=True) 
    
    ## add a table to the environment
    #table_file = os.path.join(objects_path, 'table.kinbody.xml')
    #table = env.ReadKinBodyXMLFile(table_file)
    #env.AddKinBody(table)
    #table_pose = numpy.array([[1., 0.,  0., 2],
                        #[0., 0., -1., 2],
                        #[0., 1.,  0., 0.0], 
                        #[0., 0.,  0., 1.]])
    #table.SetTransform(table_pose)

    ## add a fuze bottle on top of the table
    #fuze_path = os.path.join(objects_path, 'fuze_bottle.kinbody.xml')    
    #fuze = env.ReadKinBodyXMLFile(fuze_path)
    #fuze.SetName('fuze1')
    #table_aabb = table.ComputeAABB()
    #x = table_aabb.pos()[0] + table_aabb.extents()[0]*0 # middle of table in x
    #y = table_aabb.pos()[1] + table_aabb.extents()[1]*.6 # closer to one side of table in y
    #z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01 # slightly above table in z (so its not in collision
    #fuze_pose = fuze.GetTransform()
    #fuze_pose[:3,3] = numpy.transpose([x, y, z])
    #fuze.SetTransform(fuze_pose)
    #env.AddKinBody(fuze)
    
    
    ## add a fuze bottle 2 on top of the table
    ##fuze2 = env.ReadKinBodyXMLFile(fuze_path)
    ##fuze2.SetName('fuze2')
    ##fuze_pose2 = fuze2.GetTransform()
    ##fuze_pose2[:3,3] = numpy.transpose([x-0.5, y, z])
    ##fuze2.SetTransform(fuze_pose2)
    ##env.AddKinBody(fuze2)

    # ===========================
    #   PLANNING
    # ===========================
    ## move to a good start position
    ##robot.head.MoveTo([0, -math.pi/16]) # look down slightly
    ##robot.PlanToNamedConfiguration('relaxed_home') # move the arms to the 'relaxed_home' position
    #indices, values = robot.configurations.get_configuration('relaxed_home') # Faster for testing
    ##print values
    #values[7] = numpy.pi*3/2
    #values[9] = 0
    #values[11] = 0
    #values[13] = 0
    #robot.SetDOFValues(values=values, dofindices=indices)
    

    ## drive to the table
    #robot_in_table = numpy.array([[0., 1., 0.,  0.], 
                                #[0., 0., 1.,  0.],
                                #[1., 0., 0., -1.025],
                                #[0., 0., 0.,  1.]])
    #base_pose = numpy.dot(table.GetTransform(), robot_in_table)
    #base_pose[2,3] = 0
    ##robot.base.PlanToBasePose(base_pose)
    #robot.SetTransform(base_pose) # way faster for testing    

    # set planner
    #actual_planner = Sequence(VectorFieldPlanner(),CHOMPPlanner())
    actual_planner = VectorFieldPlanner()
    robot.planner = Sequence(actual_planner, TSRPlanner(delegate_planner=actual_planner))
  


#############my code   
    #robot.right_arm.SetActive()
    
    ##working
    ##planner = VectorFieldPlanner()
    ##traj = planner.PlanToEndEffectorOffset(robot,numpy.array([0,0,-1]),0.2)
    ##if traj.GetNumWaypoints() > 0:
        ##traj = robot.ExecutePath(traj)

    ##not working
    ##manip = robot.GetActiveManipulator()
    ##target_ee_pose = manip.GetEndEffectorTransform()
    ##target_ee_pose[2,3] += 0.05   #translation of 0.05 m in world z axis
    ##planner = VectorFieldPlanner()
    ##traj = planner.PlanToEndEffectorPose(robot, target_ee_pose)
    
       
    ##not working    
    ##planner = VectorFieldPlanner()
    ##config = robot.GetActiveDOFValues()
    ##config1 = copy.deepcopy(config)
    ###config[3] = config[3] + numpy.pi/2
    ###config[6] = config[6] + numpy.pi/2
    ###config[7] = config[7] + numpy.pi/2
    ##config1[3] += -numpy.pi/6
    ###config[4] = config[11] - numpy.pi/3
    ##traj = planner.PlanToConfiguration(robot, config1)
    
    #if traj.GetNumWaypoints() > 0:
        #traj = robot.ExecutePath(traj)
    

  
#############my code 

    # Grasp the bottle  
    #robot.right_arm.Grasp(fuze, push_required=False, num_attempts=30)
    #robot.right_arm.PlanToNamedConfiguration('home')
    
    #TO LOAD HUMAN
    #herb.DetectHuman(env, orhuman=2)
    
    
    

    # we do this so the viewer doesn't close when the example is done
    raw_input('press enter to esc')
    #import IPython; IPython.embed()
    
    













#######DELETE      
    #config = robot.GetActiveDOFValues()
    #manip = robot.GetActiveManipulator()
    #target_ee_pose = manip.GetEndEffectorTransform()
    #target_ee_pose1 = copy.deepcopy(target_ee_pose)
    #target_ee_pose1[2,3] -= 0.2
    
    #filter_options = openravepy.IkFilterOptions.CheckEnvCollisions #or 0 for no collision checks
    #config1 = robot.right_arm.FindIKSolution(target_ee_pose1, filter_options) # will return None if no config can be found
    #robot.SetActiveDOFValues(config1)
    
    #raw_input('ppp') 
    #robot.SetActiveDOFValues(config)
    #raw_input('ppp') 
    
    #planner = VectorFieldPlanner()
    #traj = planner.PlanToEndEffectorPose(robot,target_ee_pose)
    
