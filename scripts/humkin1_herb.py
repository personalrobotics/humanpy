#!/usr/bin/env python
import argparse
import logging 
import argparse
import time
import herbpy
from modular_action_planning.tasks.taskUtils import setupTableEnv

if __name__ == "__main__":    
    logger = logging.getLogger('test_skel_herb')
    logger.setLevel(logging.INFO)

    env, herb = herbpy.initialize(attach_viewer=True, sim=True) 

    with env:             
        #Manually add objects to the environment
        table = setupTableEnv.add_table(env, herb)
        table.SetName('table')
        setupTableEnv.set_robot_pose(env, herb, table)  
        glass = setupTableEnv.place_glass_on_table(env, table, .2, .7)
        bowl = setupTableEnv.place_bowl_on_table(env, table, .45, .8)
        plate = setupTableEnv.place_plate_on_table(env, table, .3, .8)

    herb.DetectHuman(env, orhuman=1)


