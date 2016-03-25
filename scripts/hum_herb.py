#!/usr/bin/env python
import numpy
import logging
import herbpy
import humanpy

if __name__ == "__main__":
    
    logger = logging.getLogger('test_humherb')
    logger.setLevel(logging.INFO)
    
    sim = True
    env, herb = herbpy.initialize(attach_viewer=True, sim=sim)  
     
    with env:
        _, human = humanpy.initialize(sim=sim, env=env)   
        humanLocation = numpy.array([[ 0. ,  0. ,  -1. ,   1.9],  #-1.2
                                    [ -1. ,  0. ,  0. ,  0.],   #-0.5
                                    [ 0. ,  1. ,  0. ,   0.85],
                                    [ 0. ,  0. ,  0. ,   1. ]])
        human.SetTransform(humanLocation)
        human.right_arm.SetActive()
            
        table = env.ReadKinBodyXMLFile('objects/table.kinbody.xml')       
        table_pose = numpy.array([[0., 0.,  1., 1.1],
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
        
        glass = env.ReadKinBodyXMLFile('objects/plastic_glass.kinbody.xml')       
        glass_pose = numpy.identity(4)  
        glass_pose[:3,3] = numpy.transpose([x - 0.25 , y -0.75 , z + 0.01])
        glass.SetTransform(glass_pose)
        glass.SetName('glass')
        env.AddKinBody(glass)        
        
        fuze = env.ReadKinBodyXMLFile('objects/fuze_bottle.kinbody.xml')       
        fuze_pose = numpy.identity(4)  
        fuze_pose[:3,3] = numpy.transpose([x + 0.3 , y - 0.4 , z + 0.01])
        fuze.SetTransform(fuze_pose)
        fuze.SetName('fuze')
        env.AddKinBody(fuze)
        
        herb.right_arm.SetActive()
        human.right_arm.SetActive()
   
    herb.Grasp(glass)
    human.right_arm.Grasp(fuze)    

    #raw_input("press enter to quit!")
 
        