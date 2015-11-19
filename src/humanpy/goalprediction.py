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


class goal_prediction():
    def __init__(self,user_id, user_hand):
        self.user_hand = user_hand
        self.user_id = user_id
        self.cost_ee_pos_s_ee_pos_u = 0 
        self.obj_pos = []  #list of array of lenght 3 (1 array for each goal)
        self.prob_traj_goals = []  #[num_goal]
        self.ee_pos_u_init = True
        self.ee_pos_s = numpy.zeros(3)
        self.ee_pos_up = numpy.zeros(3)
        self.unreachedgoal = True
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
        return math.sqrt(min_dist)
        
    def prob_traj_goal(self, ee_pos_u, ee_pos_g):   
        """
        probability of moving from strating position s to current position u given the goal position g 
        @param ee_pos_g - hand effector goal position 
        @param ee_pos_u - hand effector current position 
        """
        num = math.exp(-math.pow(self.cost_ee_pos_s_ee_pos_u + self.cost(ee_pos_u, ee_pos_g),2))
        den = math.exp(-math.pow(self.cost(self.ee_pos_s, ee_pos_g),2))
        return (num/den)

    def callback_env_obj_pose(self, obj_poses):
        """
        read object/goal positions 
        @param obj_poses - list of objects/goal
        """
        self.obj_pos = []
        self.prob_traj_goals = numpy.zeros(len(obj_poses.poses))
        for i in range(len(obj_poses.poses)):
            currpos = numpy.array([obj_poses.poses[i].position.x, 
                       obj_poses.poses[i].position.y, 
                       obj_poses.poses[i].position.z])
            #currorient = numpy.array([obj_poses.poses[i].orientation.x, 
                          #obj_poses.poses[i].orientation.y, 
                          #obj_poses.poses[i].orientation.z, 
                          #obj_poses.poses[i].orientation.w])
            self.obj_pos.append(currpos)
        
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
        
        if len(self.obj_pos) > 0:     #wait for published objects
            if self.ee_pos_u_init:
                self.ee_pos_s = currpos #starting pos
                self.ee_pos_up = currpos
                self.ee_pos_u_init = False          
       
            self.cost_ee_pos_s_ee_pos_u += self.cost(self.ee_pos_up, currpos)
            prob_goal = 1./len(self.obj_pos)   #equal probability of the abstacles
            tot_prob = 0
            for i in range(len(self.obj_pos)):  
                self.prob_traj_goals[i] = self.prob_traj_goal(currpos, self.obj_pos[i])
                tot_prob +=  self.prob_traj_goals[i]*prob_goal
            
            prob_goal_traj = Float32MultiArray();                
            for i in range(len(self.obj_pos)):
                pro = self.prob_traj_goals[i]*prob_goal/tot_prob
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



 