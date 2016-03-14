PACKAGE = 'humanpy'
import numpy
import rospy
import rospkg
import logging
import copy
import collections
from scipy import signal
from tf import transformations, LookupException, ConnectivityException, ExtrapolationException, TransformListener
import humanpy
import herbpy
from prpy.planning import Sequence, VectorFieldPlanner, TSRPlanner
from prpy.base import wam
from prpy import util, viz
from prpy.planning.exceptions import CollisionPlanningError, SelfCollisionPlanningError
import random
import openravepy

BASE_FRAME = '/map'
KIN_FRAME = '/head/skel_depth_frame'
KIN_FRAME2 = '/head/skel_depth_frame2'

logger = logging.getLogger('humanpy')
logger.setLevel(logging.INFO)

class Orhuman(object):
    def __init__(self, id, env, enable_legs=True, segway_sim=True):
        assert id != ''
        self.id = id
        self.enabled = True
        self.enable_legs = enable_legs
        self.env = env
        self.robot = env.GetRobot('herb')
        _,self.body = humanpy.initialize(sim=True, user_id=id, env=env)
        self.body.Enable(False)
        self.initialDOFvalues = self.body.GetDOFValues()
        self.currentDOFvalues = self.body.GetDOFValues()        
        self.starting_angle = dict()
        self.last_updated = rospy.get_rostime().secs 
        if segway_sim==True: 
            self.refsys = KIN_FRAME2 
        else: 
            self.refsys = KIN_FRAME

    def hide(self):
        self.body.SetVisible(False)
        self.body.Enable(False)
        self.enabled = False

    def show(self):
        self.body.SetVisible(True)
        self.body.Enable(True)
        self.enabled = True

    def getFullTfName(self, link_name):
        return '/skel/' + self.id + '/' + link_name
                
    def checkArg(self, arg):
        if arg > 1.0: arg = 1.0
        if arg < -1.0: arg = -1.0
        return arg

    def getSkeletonTransformation(self, tf, tf_name, world_name=BASE_FRAME):
        tf_name_full = self.getFullTfName(tf_name)
        if tf.frameExists(tf_name_full) and tf.frameExists(world_name):   
            try:                
                time = tf.getLatestCommonTime(tf_name_full, world_name)
                pos, quat = tf.lookupTransform(world_name, tf_name_full, time)
            except:
                return None
            self.last_updated = time.secs if self.last_updated <= time.secs else self.last_updated
            r = transformations.quaternion_matrix(quat)
            skel_trans = transformations.identity_matrix()
            skel_trans[0:4,0:4] = r
            skel_trans[0:3, 3] = pos    
            return skel_trans
        else:
            return None
 
    def getUpperLimbAngles(self, tf, side):
        if side == 'L':        
            #4: elbow (-x), #3: shoulder 3 (y), #2: shoulder 2 (z), #1: shoulder (-x) in kin frame
            sys_shoulder = self.getSkeletonTransformation(tf, 'ShoulderLeft', self.refsys)
            sys_elbow = self.getSkeletonTransformation(tf, 'ElbowLeft', self.refsys)
            sys_hand = self.getSkeletonTransformation(tf, 'WristLeft', self.refsys)
        else:
            #4: elbow (-x), #3: shoulder 3 (-y), #2: shoulder 2 (-z), #1: shoulder (-x)  in kin frame
            sys_shoulder = self.getSkeletonTransformation(tf, 'ShoulderRight', self.refsys)
            sys_elbow = self.getSkeletonTransformation(tf, 'ElbowRight', self.refsys)
            sys_hand = self.getSkeletonTransformation(tf, 'WristRight', self.refsys)
        
        sys_shcen = self.getSkeletonTransformation(tf, 'SpineShoulder', self.refsys)
           
        if sys_shoulder is None or sys_elbow is None or sys_hand is None or sys_shcen is None:
            return None
        
        sys_shcen_elb = numpy.dot(numpy.linalg.inv(sys_shcen), sys_elbow)
        
        den_vect_es = numpy.linalg.norm([sys_shoulder[0:3,3] - sys_elbow[0:3,3]])
        den_vect_he = numpy.linalg.norm([sys_elbow[0:3,3] - sys_hand[0:3,3]])

        if numpy.abs(den_vect_es) > 0.001 and numpy.abs(den_vect_he) > 0.001:    
            vect_es = (sys_shoulder[0:3,3] - sys_elbow[0:3,3])/ den_vect_es                  
            vect_he = (sys_elbow[0:3,3] - sys_hand[0:3,3])/ den_vect_he                  
            vect_norm_es_eh = numpy.cross(vect_he, vect_es)
            
            q4 = - numpy.arccos(numpy.asscalar(numpy.dot(vect_he.T,vect_es)))                   #[0,-pi]
            
            if side == 'L': 
                q2 = numpy.asscalar(numpy.arcsin(self.checkArg(-vect_es[0])))                   #[-pi/2,pi/2]
            else:
                q2 = numpy.asscalar(numpy.arcsin(self.checkArg(vect_es[0]))) 
            if sys_shcen_elb[1,3] > 0:                                                          #projection on y of the SpineShoulder. if y positive, add pi/2
                q2 = q2 + numpy.sign(q2)*numpy.pi/2                                             #[-pi,pi]

            if numpy.abs(numpy.cos(q2)) > 0.1:
                q1 = numpy.asscalar(numpy.arccos(self.checkArg(vect_es[1]/numpy.cos(q2))))      #[0,pi]
                if numpy.asscalar(numpy.arcsin(self.checkArg(-vect_es[2]/numpy.cos(q2)))) < 0:  #[-pi,pi]
                    q1 = -q1
            
                if numpy.abs(numpy.sin(q1)) > 0.1:
                    q3 = numpy.asscalar(numpy.arccos(self.checkArg(-vect_norm_es_eh[0]/numpy.cos(q2))))     #[0,pi]
                    comm_q3 = numpy.cos(q1)*numpy.cos(q3)*numpy.sin(q2)
                    if side == 'L':                 
                        val_q3 = (vect_norm_es_eh[1] + comm_q3)/numpy.sin(q1)                               
                        q3_b = numpy.asscalar(numpy.arcsin(self.checkArg(val_q3))) 
                    else:
                        val_q3 = (-vect_norm_es_eh[1] + comm_q3)/numpy.sin(q1)
                        q3_b = numpy.asscalar(numpy.arcsin(self.checkArg(val_q3))) 
                    if q3_b < 0:                                                                            #[-pi,pi]
                        q3 = -q3
                else:
                    q3 = None
            else:
                q1 = None
                q3 = None

            return [q1, q2, q3, q4]
        else:
            return None

    def getLowerLimbAngles(self, tf, side):
        if side == 'L':        
            sys_hip = self.getSkeletonTransformation(tf, 'HipLeft', self.refsys)
            sys_knee = self.getSkeletonTransformation(tf, 'KneeLeft', self.refsys)
            sys_foot = self.getSkeletonTransformation(tf, 'AnkleLeft', self.refsys)
        else:            
            sys_hip = self.getSkeletonTransformation(tf, 'HipRight', self.refsys)
            sys_knee = self.getSkeletonTransformation(tf, 'KneeRight', self.refsys)
            sys_foot = self.getSkeletonTransformation(tf, 'AnkleRight', self.refsys)
            
        if sys_hip is None or sys_knee is None or sys_foot is None:
            return None
        
        den_vect_kh = numpy.linalg.norm([sys_hip[0:3,3] - sys_knee[0:3,3]])
        den_vect_fk = numpy.linalg.norm([sys_knee[0:3,3] - sys_foot[0:3,3]])
        if numpy.abs(den_vect_kh) > 0.001 and numpy.abs(den_vect_fk) > 0.001:  
            vect_kh = (sys_hip[0:3,3] - sys_knee[0:3,3])/den_vect_kh                   
            vect_fk = (sys_knee[0:3,3] - sys_foot[0:3,3])/den_vect_fk   
            q2 = - numpy.arccos(self.checkArg(numpy.asscalar(numpy.dot(vect_kh.T,vect_fk))))
            
            q1 = numpy.asscalar(numpy.arccos(vect_kh[1]))                                       #[0,pi]
            if numpy.asscalar(numpy.arcsin(vect_kh[2])) < 0:                                    #[-pi,pi]
                q1 = -q1 
            
            return [q1, q2]
        else:
            return None
  
    def checkLimits(self, joint, angle_vect, angle_elem):
        if angle_vect is not None:
            angle = angle_vect[angle_elem]
            if angle is not None and joint is not None:
                lower,upper = joint.GetLimits()
                if angle < lower: angle = lower
                if angle > upper: angle = upper
                
                self.currentDOFvalues[joint.GetDOFIndex()] = angle           
                self.body.SetDOFValues(self.currentDOFvalues)
    
    def define_pose_from_eetransform(self, ee_pose):
        ee_quat = transformations.quaternion_from_matrix(ee_pose)        
        pose = Pose()
        pose.position.x = ee_pose[0,3]
        pose.position.y = ee_pose[1,3]
        pose.position.z = ee_pose[2,3]
        pose.orientation.x = ee_quat[0]
        pose.orientation.y = ee_quat[1]
        pose.orientation.z = ee_quat[2]
        pose.orientation.w = ee_quat[3]        
        return pose   
    
    def define_pose_from_posquat(self, pos, quat):
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]        
        return pose  

    def update(self, tf):     
        TIME_TO_WAIT = 1
        time_since_last_update = rospy.get_rostime().secs - self.last_updated
        if self.enabled and time_since_last_update > TIME_TO_WAIT:
            self.hide()
        elif not self.enabled and time_since_last_update <= TIME_TO_WAIT:
            self.show()
        
        #Chest
        person_transform = self.getSkeletonTransformation(tf, 'SpineBase')        
        if person_transform is not None:           
            #requied to have human standing 
            #angle/axis rotation for the alignment of human y with world z
            th = numpy.arccos(numpy.asscalar(numpy.dot(numpy.array([0.,0.,1.]).T, person_transform[0:3,1])))
            nn_axis = numpy.cross(person_transform[0:3,1], [0.,0.,1.])
            axis = nn_axis/numpy.linalg.norm(nn_axis)
            a11 = numpy.cos(th) + numpy.square(axis[0])*(1 - numpy.cos(th))
            a12 = axis[0]*axis[1]*(1 - numpy.cos(th)) - axis[2]*numpy.sin(th)
            a13 = axis[0]*axis[2]*(1 - numpy.cos(th)) + axis[1]*numpy.sin(th)
            a21 = axis[1]*axis[0]*(1 - numpy.cos(th)) + axis[2]*numpy.sin(th)
            a22 = numpy.cos(th) + numpy.square(axis[1])*(1 - numpy.cos(th))
            a23 = axis[1]*axis[2]*(1 - numpy.cos(th)) - axis[0]*numpy.sin(th)
            a31 = axis[2]*axis[0]*(1 - numpy.cos(th)) - axis[1]*numpy.sin(th)
            a32 = axis[2]*axis[1]*(1 - numpy.cos(th)) + axis[0]*numpy.sin(th)
            a33 = numpy.cos(th) + numpy.square(axis[2])*(1 - numpy.cos(th))
            matrix_rot = numpy.array([[a11, a12 ,a13],
                                        [a21, a22, a23],
                                        [a31, a32, a33]])
            
            person_position_transform_rot = numpy.dot(matrix_rot, person_transform[0:3,0:3])
            person_position_transform = numpy.zeros((4, 4))
            person_position_transform[0:3,0:3] = person_position_transform_rot
            person_position_transform[0:4,3] = person_transform[0:4,3]
            person_position_transform[2,3] = 0.85
            self.body.SetTransform(person_position_transform)
        
        ##Left arm
        ul_angles = self.getUpperLimbAngles(tf, 'L')
        if ul_angles is not None:
            self.checkLimits(self.body.GetJoint('JLShoulder'), ul_angles, 0)
            self.checkLimits(self.body.GetJoint('JLShoulder2'), ul_angles, 1)
            self.checkLimits(self.body.GetJoint('JLShoulder3'), ul_angles, 2)
            self.checkLimits(self.body.GetJoint('JLForearm'), ul_angles, 3)

        #Right arm
        ul_angles = self.getUpperLimbAngles(tf, 'R')
        if ul_angles is not None: 
            self.checkLimits(self.body.GetJoint('JRShoulder'), ul_angles, 0)
            self.checkLimits(self.body.GetJoint('JRShoulder2'), ul_angles, 1)
            self.checkLimits(self.body.GetJoint('JRShoulder3'), ul_angles, 2)        
            self.checkLimits(self.body.GetJoint('JRForearm' ), ul_angles, 3)
        
        if self.enable_legs == True:        
            #Left leg
            ul_angles = self.getLowerLimbAngles(tf, 'L')
            if ul_angles is not None:
                self.checkLimits(self.body.GetJoint('JLThigh'), ul_angles, 0)
                self.checkLimits(self.body.GetJoint('JLCalf'), ul_angles, 1)

            #Right leg
            ul_angles = self.getLowerLimbAngles(tf, 'R')
            if ul_angles is not None:
                self.checkLimits(self.body.GetJoint('JRThigh'), ul_angles, 0)
                self.checkLimits(self.body.GetJoint('JRCalf'), ul_angles, 1)
        
        #TODO; head, neck  


def humanInList(human, ids):
    for id in ids:
        if human.id == 'user_' + id: return True
    return False
        
def addRemoveHumans(tf, humans, env, enable_legs=True, segway_sim=True):
    import re
    matcher = re.compile('.*user_(\\d+).*')    
    all_tfs = tf.getFrameStrings()
    all_human_ids = []
    for frame_name in all_tfs:
        match = matcher.match(frame_name)
        if match is not None:
            all_human_ids.append(match.groups()[0])

    # removing
    humans_to_remove = [x for x in humans if not humanInList(x, all_human_ids)]
    for human in humans_to_remove:
        env.RemoveKinBody(human);
        human.destroy()
        humans.remove(human)
        
    # adding
    for id in all_human_ids:
        found = False
        for human in humans:
            if human.id == 'user_' + id:
                found = True
                break
        if not found:
            humans.append(Orhuman('user_' + id, env, 
                                  enable_legs=enable_legs,
                                  segway_sim=segway_sim))





