#!/usr/bin/env python
import humanpy
import numpy
import IPython
from prpy.rave import add_object

if __name__ == "__main__":

    env, robot = humanpy.initialize(attach_viewer=True)

    with env:
        fuze = add_object(env, 'fuze_bottle', 'objects/fuze_bottle.kinbody.xml')
        glass = add_object(env, 'plastic_glass', 'objects/plastic_glass.kinbody.xml')
        fuze_pose = numpy.eye(4)
        glass_pose = numpy.eye(4)
        fuze_pose[0:3, 3] = [ 0.8, -0.3, 0.4 ]
        glass_pose[0:3, 3] = [ 0.9, 0, 0.4 ]
        fuze.SetTransform(fuze_pose)
        glass.SetTransform(glass_pose)
        fuze.Enable(True)
        glass.Enable(True)
    
    robotLocation = numpy.array([[ 0. ,  0. ,  1. ,   0.4],
                                [ 1. ,  0. ,  0. ,  -0.1],
                                [ 0. ,  1. ,  0. ,   0.3],
                                [ 0. ,  0. ,  0. ,   1. ]]) 
    robot.SetTransform(robotLocation)
    
    robot.right_arm.SetActive()
    #robot.right_arm.Grasp(fuze)

    raw_input("press enter to quit!")
