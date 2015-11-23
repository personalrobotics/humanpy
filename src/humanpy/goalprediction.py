#!/usr/bin/env python
PACKAGE = 'humanpy'
import math
import prpy
import tf
import rospy
import logging
import numpy
import time
import copy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray


logger = logging.getLogger('goalprediction')
W = 1.5

import os
FILE = open('/home/spelle/check.log', 'w')


class goal_prediction():
    def __init__(self,user_id, user_hand):
        self.user_hand = user_hand
        self.user_id = user_id
        self.cost_ee_pos_s_ee_pos_u = 0 
        self.ee_pos_gs = []  #list of array of lenght 3 (1 array for each goal)
        self.prob_tg_prob_g = []  #[num_goal]
        self.ee_pos_u_init = True
        self.ee_pos_s = numpy.zeros(3)
        self.ee_pos_up = numpy.zeros(3)
        self.prob_goal_traj_pub = rospy.Publisher('/skel/prob_goal_user_' + self.user_id , 
                                                  Float32MultiArray, queue_size=10)

    def cost(self, ee_pos_f, ee_pos_s):
        """
        evaluation of the euclidean distance between two poses
        @param ee_pos_f - hand effector starting position 
        @param ee_pos_s - hand effector ending position 
        """
        min_dist = 0.0
        for i in range(len(ee_pos_s)):
            min_dist += math.pow(ee_pos_f[i] - ee_pos_s[i],2)              
        return math.sqrt(min_dist)*W
        

    def callback_env_obj_pose(self, obj_poses):
        """
        read object/goal positions 
        @param obj_poses - list of objects/goal
        """
        self.ee_pos_gs = []
        for i in range(len(obj_poses.poses)):
            currpos = numpy.array([obj_poses.poses[i].position.x, 
                       obj_poses.poses[i].position.y, 
                       obj_poses.poses[i].position.z])
            #currorient = numpy.array([obj_poses.poses[i].orientation.x, 
                          #obj_poses.poses[i].orientation.y, 
                          #obj_poses.poses[i].orientation.z, 
                          #obj_poses.poses[i].orientation.w])
            self.ee_pos_gs.append(currpos)
        
    def callback_user(self, hand_pose):
        """
        read human hand pose, evaluate the probability on goal and publishes the probability vector 
        @param hand_pose - human hand pose
        """
        currpos = numpy.array([hand_pose.position.x, 
                   hand_pose.position.y, 
                   hand_pose.position.z])
        #currorient = numpy.array([hand_pose.orientation.x, 
                      #hand_pose.orientation.y, 
                      #hand_pose.orientation.z, 
                      #hand_pose.orientation.w])
        
        if len(self.ee_pos_gs) > 0:     #wait for published objects
            ee_pos_g_curr = copy.deepcopy(self.ee_pos_gs)  #to be sure to not have variable changes during callback
            if self.ee_pos_u_init:
                self.ee_pos_s = currpos #starting pos
                self.ee_pos_up = currpos
                self.goal_den = numpy.zeros(len(ee_pos_g_curr))
                for i in range(len(ee_pos_g_curr)):                    
                    self.goal_den[i] = math.exp(-math.pow(self.cost(self.ee_pos_s, ee_pos_g_curr[i]),2))
                self.ee_pos_u_init = False          
            
            self.prob_tg_prob_g = numpy.zeros(len(ee_pos_g_curr))   

            self.cost_ee_pos_s_ee_pos_u += self.cost(self.ee_pos_up, currpos)
            prob_goal = 1./len(ee_pos_g_curr)   #equal probability of the abstacles
            for i in range(len(ee_pos_g_curr)): 

                self.cost_ee_pos_s_ee_pos_u += self.cost(self.ee_pos_up, currpos)
                goal_num = math.exp(-math.pow(self.cost_ee_pos_s_ee_pos_u + 
                                              self.cost(currpos, ee_pos_g_curr[i]),2))
                self.prob_tg_prob_g[i] = goal_num/self.goal_den[i]*prob_goal
           
            tot_prob =  numpy.sum(self.prob_tg_prob_g)            
            prob_goal_traj = Float32MultiArray();      
            print '--------------'            
            for i in range(len(ee_pos_g_curr)):
                pro = self.prob_tg_prob_g[i]/tot_prob
                if math.isnan(pro):
                    pro = 0.0
                print pro
                prob_goal_traj.data.append(pro)
            self.prob_goal_traj_pub.publish(prob_goal_traj)
            
            self.ee_pos_up = currpos
            
            
            
   
    def listener(self):
        """
        initialiazes two subscriber and the probability evluation
        """
        rospy.Subscriber('/env_obj_pos', PoseArray, self.callback_env_obj_pose)
        if self.user_hand == 'R':
            sub_id = '/skel/user_' + str(self.user_id) + '/rhand_pos'
        else:
            sub_id = '/skel/user_' + str(self.user_id) + '/lhand_pos'
        rospy.Subscriber(sub_id, Pose, self.callback_user)
        rospy.spin()


if __name__ == "__main__":   
    time_to_start = 4.0
    while time_to_start > 0.0:
        time.sleep(1)
        time_to_start -= 1 
    
    prpy.logger.initialize_logging()
    logger.info('Starting probability evaluation..... ')
   
    rospy.init_node('hg_prediction', anonymous=True)
    
    user_hand = rospy.get_param("~user_hand");
    user_id = rospy.get_param("~user_id");

    goal_pred = goal_prediction(str(user_id),str(user_hand))   #indicate subject number
    goal_pred.listener()



 