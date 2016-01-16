PACKAGE = 'humanpy'
import numpy
import rospy
import rospkg
import logging
import copy
import collections
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray, Bool
from msg import PoseArrays
from scipy import signal
from tf import transformations, LookupException, ConnectivityException, ExtrapolationException
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
#REF_OBJ = ['fuze', 'pitcher', 'glass', 'block', 'plate', 'pop_tarts', 
           #'tray', 'bowl', 'glass1', 'glass2', 'glass3',
           #'plastic_glass']

REF_OBJ = ['glass1','glass2','glass3','box1','box2','box3', 
           'box5','box6','box7','box8','box9']

logger = logging.getLogger('humanpy')
logger.setLevel(logging.INFO)

#import os
#f = open('/home/spelle/ros_ws/myfile.txt','w')


class Orhuman(object):
    def __init__(self, id, env, node_name, hum_goal_predic=False, segway_sim=True, action='stamp'):
        assert id != ''
        self.id = id
        self.enabled = True
        self.env = env
        self.robot = env.GetRobot('herb')
        _,self.body = humanpy.initialize(sim=True, passive = hum_goal_predic, 
                                         user_id=id, env=env)
        self.initialDOFvalues = self.body.GetDOFValues()
        self.currentDOFvalues = self.body.GetDOFValues()        
        self.starting_angle = dict()
        self.last_updated = rospy.get_rostime().secs 
        self.hum_goal_predic = hum_goal_predic
        self.action = action
        self.node_name = node_name
        if segway_sim==True: 
            self.refsys = KIN_FRAME2 
        else: 
            self.refsys = KIN_FRAME
        if hum_goal_predic:     
            import re
            #self.filter_options = openravepy.IkFilterOptions.CheckEnvCollisions  #or 0 for no collision checks
            self.filter_options = openravepy.IkFilterOptions.IgnoreSelfCollisions
            
            self.tsr_box_distance = 0.04 #0.14
            self.nottouchedobj = []
            self.tsrviz = []   # needed in order to visualize tsr continuosly
            self.tsrs = []
            self.tsrmsg = self.tsrchain()
            self.min_tsr = numpy.identity(4)      
            
            #self.xxx = []
            
            self.restartnode = False            
            
            cubeexist = False
            for i in env.GetBodies():
                if i.GetName == 'cube' + str(id):
                    cubeexist = True
                    break
            if cubeexist == False:
                self.cube = openravepy.RaveCreateKinBody(env,'')
                self.cube.SetName('cube' + str(id))
                self.cube.InitFromBoxes(numpy.array([[0,0,0,0.01,0.01,0.01]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
                self.cube.Enable(False)
                env.AddKinBody(self.cube)  
            
            self.node_rh = rospy.Publisher(self.getFullTfName('rhand_pos'), Pose, queue_size=10)
            self.node_lh = rospy.Publisher(self.getFullTfName('lhand_pos'), Pose, queue_size=10)
            self.env_obj_pose = rospy.Publisher('/env_obj/pos', PoseArray, queue_size=10)
            self.env_obj_tsr = rospy.Publisher('/env_obj/tsr', PoseArrays, queue_size=10)
            self.robot_eep = rospy.Publisher('/herb/active_eep', Pose, queue_size=10)
            self.restart = rospy.Publisher('/restart', Bool, queue_size=10)
            self.prob_goal = rospy.Subscriber('/skel/prob_goal_' + self.id, Float32MultiArray, 
                                              self.callback_color_obj, queue_size=10)
            self.rob_up_twist = rospy.Subscriber('/herb/next_eep', Float32MultiArray,
                                              self.callback_update_twist, queue_size=10)
            self.mint_sr = rospy.Subscriber('/herb/mintsr', Float32MultiArray,
                                              self.callback_update_min_tsr, queue_size=10)
            #self.rob_up_pos = rospy.Subscriber('/herb/next_eep', Pose,
                                              #self.callback_update_pos, queue_size=1)
                                              
            id_num = [int(s) for s in re.findall('\\d+', self.id)]
            rospy.set_param("/global_user", str(id_num[0]))
            self.rate = rospy.Rate(150) # 10hz
            
            

        #for joint in self.body.GetJoints():
        #    joint.SetLimits([-numpy.pi+1e-5],[numpy.pi-1e-5])

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
       
    def publish_poses(self):  
        hand_pose_r = self.define_pose_from_eetransform(self.body.right_arm.GetEndEffectorTransform())
        hand_pose_l = self.define_pose_from_eetransform(self.body.left_arm.GetEndEffectorTransform())
        
        self.node_rh.publish(hand_pose_r)
        self.node_lh.publish(hand_pose_l) 

    def publish_object_pos(self):
        poselist = PoseArray() 
        for obj in self.body.GetEnv().GetBodies():
            for ref_name in REF_OBJ:                
                if ref_name in obj.GetName():
                    ee_body = obj.GetTransform()                    
                    block_pose = self.define_pose_from_eetransform(ee_body)
                    poselist.poses.append(block_pose)
                    break
        self.env_obj_pose.publish(poselist)
    
    def tsrchain(self):
        tsrmsg = PoseArrays()
        tsrnb =  30 
        self.tsrs = []
        #filter_options = openravepy.IkFilterOptions.CheckEnvCollisions #or 0 for no collision checks
        
        if len(self.nottouchedobj) == 0:
            init_nottouchedobj = True
        else:
            init_nottouchedobj = False

        count = -1
        for obj in self.body.GetEnv().GetBodies():
            for ref_name in REF_OBJ:
                if ref_name in obj.GetName():
                    count += 1
                    singletsr = []
                    
                    
                    tsr_list = self.robot.tsrlibrary(obj, self.action)
                  
                    if init_nottouchedobj:
                        self.nottouchedobj.append(True)
                    
                    if all(x==False for x in self.nottouchedobj) == True:
                        self.robot.PlanToNamedConfiguration('home', execute=True)
                        logger.info('Task done!!!!')
                        rospy.on_shutdown(self.node_name)
                    
                    posar = PoseArray()
                    if self.nottouchedobj[count] == True:
                        tsr_chain_idx = random.randint(0, len(tsr_list) - 1)                  
                        tsr_chain = tsr_list[tsr_chain_idx]                                           
                        for idx in range(tsrnb):
                            sample = tsr_chain.sample() 
                            config_tsr = None
                            config_stamp = None
                            with self.env:
                                config_tsr = self.robot.GetActiveManipulator().FindIKSolution(sample, self.filter_options) # will return None if no config can be found
                                sample_stamp = copy.deepcopy(sample)
                                sample_stamp[2,3] -= self.tsr_box_distance
                                config_stamp = self.robot.GetActiveManipulator().FindIKSolution(sample_stamp, self.filter_options)                            
                            #prova = self.robot.GetActiveManipulator().FindIKSolutions(sample, filter_options)  
                            #prova2 = self.robot.GetActiveManipulator().FindIKSolutions(sample_stamp, filter_options)
                            
                            #f.write('---------------1---------------' + os.linesep)
                            #f.write(str(prova))
                            #f.write('---------------2---------------' + os.linesep)
                            #f.write(str(prova2))
                            
                            ##print '----------------'
                            #print prova 
                            if config_tsr is not None and config_stamp is not None: 
                                singletsr.append(sample)
                            if config_tsr is not None and config_stamp is not None:     
                                with self.env:                                
                                        self.tsrviz.append(openravepy.misc.DrawAxes(self.env, sample, dist=0.15))
                                        pose = openravepy.poseFromMatrix(sample)
                                        quat, xyz = pose[0:4], pose[4:7]
                                        tsrpose = Pose()
                                        tsrpose.position.x = xyz[0]
                                        tsrpose.position.y = xyz[1]
                                        tsrpose.position.z = xyz[2]
                                        tsrpose.orientation.w = quat[0]
                                        tsrpose.orientation.x = quat[1]
                                        tsrpose.orientation.y = quat[2]
                                        tsrpose.orientation.z = quat[3]
                                        posar.poses.append(tsrpose)
                        if len(posar.poses) == 0:  #if the obj is not reachable (not found tsr) a fake tsr is send (tsr so far that it will be discarded by the POMPD)
                            self.nottouchedobj[count] = False  #chnage the state of the obj to touched since it cannot be reached
                            tsrpose = Pose()
                            tsrpose.position.x = -9999
                            tsrpose.position.y = -9999
                            tsrpose.position.z = -9999
                            tsrpose.orientation.w = 1
                            tsrpose.orientation.x = 0
                            tsrpose.orientation.y = 0
                            tsrpose.orientation.z = 0
                            posar.poses.append(tsrpose)  
                            singletsr.append(numpy.array([[1.,0.,0.,-9999],
                                                          [0.,1.,0.,-9999],
                                                          [0.,0.,1.,-9999],
                                                          [0.,0.,0.,1.]]))
                    else:
                        tsrpose = Pose()
                        tsrpose.position.x = -9999
                        tsrpose.position.y = -9999
                        tsrpose.position.z = -9999
                        tsrpose.orientation.w = 1
                        tsrpose.orientation.x = 0
                        tsrpose.orientation.y = 0
                        tsrpose.orientation.z = 0
                        posar.poses.append(tsrpose)   
                        singletsr.append(numpy.array([[1.,0.,0.,-9999],
                                                      [0.,1.,0.,-9999],
                                                      [0.,0.,1.,-9999],
                                                      [0.,0.,0.,1.]]))
                    tsrmsg.poses.append(posar)   
                    self.tsrs.append(singletsr)
                    break         
        return tsrmsg

 

    
    def publish_object_tsr(self):
        self.env_obj_tsr.publish(self.tsrmsg)   #so that I can publish always same tsr poses

       
    def publish_robot_eep(self):       
        robot_pose_ee = self.define_pose_from_eetransform(self.robot.GetActiveManipulator().GetEndEffectorTransform());       
        self.robot_eep.publish(robot_pose_ee)
     
    def callback_color_obj(self, prob_goal_traj):
        #self.xxx = prob_goal_traj.data
        count_obj = 0     
        if len(prob_goal_traj.data) > 0:
            for obj in self.body.GetEnv().GetBodies():
                for ref_name in REF_OBJ:                
                    if ref_name in obj.GetName(): 
                        if len(prob_goal_traj.data) > count_obj:
                            if prob_goal_traj.data[count_obj] < 0.1:
                                color = numpy.array([0.745, 0.745, 0.745]) #gray
                            elif prob_goal_traj.data[count_obj] < 0.2:
                                color = numpy.array([0.0, 1.0, 0.0]) #verde
                            elif prob_goal_traj.data[count_obj] < 0.3:
                                color = numpy.array([0.2, 0.8, 0.0]) #verde
                            elif prob_goal_traj.data[count_obj] < 0.4:
                                color = numpy.array([0.4, 0.7, 0.0]) #giallo
                            elif prob_goal_traj.data[count_obj] < 0.5:
                                color = numpy.array([0.5, 0.6, 0.0]) #giallo
                            elif prob_goal_traj.data[count_obj] < 0.6:
                                color = numpy.array([0.6, 0.5, 0.0]) #orange
                            elif prob_goal_traj.data[count_obj] < 0.7:
                                color = numpy.array([0.7, 0.4, 0.0]) #orange
                            elif prob_goal_traj.data[count_obj] < 0.8:
                                color = numpy.array([0.8, 0.3, 0.0]) #orange
                            elif prob_goal_traj.data[count_obj] < 0.95:
                                color = numpy.array([0.9, 0.2, 0.0]) #orange
                            else:
                                color = numpy.array([1.0, 0.0, 0.0]) #red
                            with self.body.GetEnv():
                                obj.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(color)
                                obj.GetLinks()[0].GetGeometries()[0].SetAmbientColor(color)
                            count_obj += 1
                            break
        #else:
            #for obj in self.body.GetEnv().GetBodies():
                #for ref_name in REF_OBJ:                
                    #if ref_name in obj.GetName(): 
                        #color = numpy.array([0.745, 0.745, 0.745]) #gray
                        #obj.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(color)
                        #obj.GetLinks()[0].GetGeometries()[0].SetAmbientColor(color)
                        #count_obj += 1
                        #break
  
    def callback_update_twist(self, rob_new_twist):  
        if self.min_tsr[0,3] !=  -9999:     #the selected tsr is not the fake tsr
            if rob_new_twist.data[0] != -9999:   #the goal was not reached  
                self.restartnode = False            
                twist = numpy.array([rob_new_twist.data[0],
                                    rob_new_twist.data[1],
                                    rob_new_twist.data[2],
                                    rob_new_twist.data[3],
                                    rob_new_twist.data[4],
                                    rob_new_twist.data[5]])
        
                vlimits = self.robot.GetDOFVelocityLimits(self.robot.GetActiveDOFIndices())
                
                if self.min_tsr is None:
                    dqout, tout = util.ComputeJointVelocityFromTwist(self.robot, 
                                                                twist, 
                                                                joint_velocity_limits=vlimits,
                                                                objective=util.quadraticPlusJointLimitObjective)
                else:
                    with self.env:
                        #filter_options = openravepy.IkFilterOptions.CheckEnvCollisions #or 0 for no collision checks
                        config = self.robot.GetActiveManipulator().FindIKSolution(self.min_tsr, self.filter_options) # will return None if no config can be found
                    dqout, tout = util.ComputeJointVelocityFromTwist(self.robot, 
                                                                twist, 
                                                                joint_velocity_limits=vlimits,
                                                                objective=util.quadraticPlusJointLimitObjective)
                    #dqout, tout = util.ComputeJointVelocityFromTwist(self.robot, 
                                                                    #twist, 
                                                                    #joint_velocity_limits=vlimits,                                                            
                                                                    #objective=util.quadraticPlusJointConf2Objective,
                                                                    #q_target=config)
            
                # Go as fast as possible 
                #dqout = min(abs(vlimits[i] / dqout[i]) if dqout[i] != 0. else 1. for i in xrange(vlimits.shape[0])) * dqout
                #for i in xrange(vlimits.shape[0]): 
                    #if not abs(dqout[i]) < vlimits[i]:
                        #dqout[i] =  (vlimits[i] - 0.0001)*numpy.sign(dqout[i]) 
                        
                ### Check collision.
                with self.env:
                    with self.robot.CreateRobotStateSaver():
                        q = self.robot.GetActiveDOFValues()
                        self.robot.SetActiveDOFValues(q + (dqout/20))  #check on herb position in 1/20 sec 
                        report = openravepy.CollisionReport()
                        if self.env.CheckCollision(self.robot, report=report):
                            raise CollisionPlanningError.FromReport(report)
                        elif self.robot.CheckSelfCollision(report=report):
                            raise SelfCollisionPlanningError.FromReport(report)
                
                self.robot.right_arm.Servo(dqout) 
            else:
                if self.action == 'stamp':
                    #print 'prob values when reaching the target:', self.xxx
                    #the robot touches the box
                    self.robot.right_arm.Servo(numpy.array([0., 0., 0., 0., 0., 0., 0.])) 
                    rospy.sleep(0.5) #to be sure the robot is not moving
                    #rospy.sleep(0.01) #to be sure the robot is not moving
                    
                    #TODO: valido only for the boxes
                    logger.info('Approching the object')
                    #import openravepy
                    #creport = openravepy.CollisionReport()
                    #print self.robot.CheckSelfCollision(creport)
                    #print creport.plink1
                    #print creport.plink2
                    #rospy.sleep(0.5)
                    traj = VectorFieldPlanner().PlanToEndEffectorOffset(self.robot,
                                                                        numpy.array([0,0,-1]),
                                                                        self.tsr_box_distance, #0.105 = 0.26(tsr)-0.04(box hieght)-0.01(cyl diplacement)-0.1(cyl hight)
                                                                        position_tolerance=0.04,
                                                                        angular_tolerance=0.15)  
                    if traj.GetNumWaypoints() > 0:
                        self.robot.ExecutePath(traj)
                    
                    #mark the stamped/graped object
                    #values_prox = numpy.array([])
                    #robee_pos = self.robot.GetActiveManipulator().GetEndEffectorTransform()[0:3,3]
                    #for obj in self.body.GetEnv().GetBodies():
                        #for ref_name in REF_OBJ:
                            #if ref_name in obj.GetName():
                                #obj_pos = obj.GetTransform()[0:3,3]                        
                                #error = obj_pos - robee_pos
                                #values_prox = numpy.append(values_prox, [numpy.dot(numpy.transpose(error), error)])  
                    #print values_prox
                    #self.nottouchedobj[numpy.argmin(values_prox)] = False

                    for i in range(len(self.tsrs)):
                        for singletsr in self.tsrs[i]:
                            error = self.min_tsr[0:3,3]-singletsr[0:3,3]
                            if numpy.sqrt(numpy.dot(numpy.transpose(error), error)) < 0.001:
                                self.nottouchedobj[i] = False
                    
                    logger.info('Deproaching the object')
                    traj = VectorFieldPlanner().PlanToEndEffectorOffset(self.robot,
                                                                        numpy.array([0,0,1]),
                                                                        self.tsr_box_distance,
                                                                        position_tolerance=0.04,
                                                                        angular_tolerance=0.15)
                    if traj.GetNumWaypoints() > 0:
                        self.robot.ExecutePath(traj)                

                    self.tsrviz = []
                    self.tsrmsg = self.tsrchain()
                    rospy.sleep(0.1) #to be sure the new tsr is send 0.05>0.006
                    self.restartnode = True
                    rospy.sleep(0.4) #to be sure the new tsr is send 0.05>0.006
                    self.restartnode = False
                if self.action == 'grasp':
                    #TODO: implement grasp action
                    pass
       
         
    def callback_update_min_tsr(self, new_min_tsr):
        new_min_tsr = numpy.array([[new_min_tsr.data[0], new_min_tsr.data[1], new_min_tsr.data[2], new_min_tsr.data[3]],
                                   [new_min_tsr.data[4], new_min_tsr.data[5], new_min_tsr.data[6], new_min_tsr.data[7]],
                                   [new_min_tsr.data[8], new_min_tsr.data[9], new_min_tsr.data[10], new_min_tsr.data[11]],
                                   [new_min_tsr.data[12], new_min_tsr.data[13], new_min_tsr.data[14], new_min_tsr.data[15]]])

        #new_min_tsr_pos = numpy.array([new_min_tsr.data[4], new_min_tsr.data[8], new_min_tsr.data[12]])        
        self.cube.SetTransform(new_min_tsr)
        self.min_tsr = new_min_tsr
           
                    

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
        
        if self.hum_goal_predic == False:        
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
        
        if self.hum_goal_predic and self.enabled:  
            self.publish_object_tsr()
            self.publish_object_pos()
            self.publish_poses()  
            self.publish_robot_eep() 
            self.restart.publish(self.restartnode) 
            self.rate.sleep()


def humanInList(human, ids):
    for id in ids:
        if human.id == 'user_' + id: return True
    return False
        
def addRemoveHumans(tf, humans, env, node_name, hum_goal_predic=False, segway_sim=True, action='stamp'):
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
            humans.append(Orhuman('user_' + id, env, node_name, 
                                  hum_goal_predic=hum_goal_predic, 
                                  segway_sim=segway_sim,
                                  action=action))








        
    #def callback_update_obj_reached(self, reached):
        #self.obj_reached = reached

  
    #def callback_update_pos(self, rob_new_pos):
        #quat = numpy.array([rob_new_pos.orientation.x, 
                            #rob_new_pos.orientation.y, 
                            #rob_new_pos.orientation.z, 
                            #rob_new_pos.orientation.w])       
        #r = transformations.quaternion_matrix(quat)
        #pose_in_world = transformations.identity_matrix()
        #pose_in_world[0:4,0:4] = r
        #pose_in_world[0:3, 3] = numpy.array([rob_new_pos.position.x, 
                                 #rob_new_pos.position.y,
                                 #rob_new_pos.position.z])
        
        ##filter_options = openravepy.IkFilterOptions.CheckEnvCollisions #or 0 for no collision checks
        #config = self.robot.GetActiveManipulator().FindIKSolution(pose_in_world, self.filter_options) # will return None if no config can be found
        #traj = planner.PlanToConfiguration(robot, config, execute=True)    
        ##if traj.GetNumWaypoints() > 0:
            ##traj = robot.ExecutePath(traj)





   #def topose(self, sample):       
        #pose = openravepy.poseFromMatrix(sample)
        #quat, xyz = pose[0:4], pose[4:7]
        #tsrpose = Pose()
        #tsrpose.position.x = xyz[0]
        #tsrpose.position.y = xyz[1]
        #tsrpose.position.z = xyz[2]
        #tsrpose.orientation.w = quat[0]
        #tsrpose.orientation.x = quat[1]
        #tsrpose.orientation.y = quat[2]
        #tsrpose.orientation.z = quat[3]
        #return tsrpose


   
    #def tsrchain2(self):
        #tsrmsg = PoseArrays() 
        #posar1 = PoseArray()
        #posar2 = PoseArray()
        #o1t1 = numpy.array([[ 0.80740015,  0. ,         0.59000424 ,-0.68275095],
                            #[ 0.59000424  ,0.  ,       -0.80740015,  0.63166503],
                            #[ 0.     ,     1.       ,   0.     ,     0.83557692],
                            #[ 0.     ,     0.      ,    0.     ,     1.        ]])
        #posar1.poses.append(self.topose(o1t1))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o1t1, dist=0.15))
        #o1t2 = numpy.array([[ 0.49558863 , 0.     ,     0.86855738 ,-0.74542541],
                            #[ 0.86855738  ,0.    ,     -0.49558863 , 0.56150744],
                            #[ 0.          ,1.   ,       0.        ,  0.82496102],
                            #[ 0.       ,   0.    ,      0.        ,  1.        ]])
        #posar1.poses.append(self.topose(o1t2))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o1t2, dist=0.15))
        #o1t3 = numpy.array([[ 0.62302083 , 0.     ,     0.78220524 ,-0.72599618],
                            #[ 0.78220524 , 0.    ,     -0.62302083 , 0.59017969],
                            #[ 0.         , 1.    ,      0.    ,      0.82661616],
                            #[ 0.         , 0.    ,      0.    ,      1.        ]])
        #posar1.poses.append(self.topose(o1t3))   
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o1t3, dist=0.15))
        #tsrmsg.poses.append(posar1)   

        #o2t1 = numpy.array([[ 0.88449288 , 0.      ,   -0.4665537 , -1.09502542],
                            #[-0.4665537  , 0.      ,   -0.88449288,  0.6990109 ],
                            #[ 0.        ,  1.      ,    0.     ,     0.82661154],
                            #[ 0.       ,   0.     ,     0.    ,      1.        ]])
        #posar2.poses.append(self.topose(o2t1))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o2t1, dist=0.15))
        #o2t2 = numpy.array([[ 0.90929182 , 0.      ,    0.41615909 ,-1.2936358 ],
                            #[ 0.41615909 , 0.     ,    -0.90929182 , 0.70459066],
                            #[ 0.       ,   1.     ,     0.        ,  0.83068533],
                            #[ 0.       ,   0.     ,     0.        ,  1.        ]])
        
        ##filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
        ##config = self.robot.GetActiveManipulator().FindIKSolution(o2t2, filter_options) 
        ##print 'config ', config
        #posar2.poses.append(self.topose(o2t2))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o2t2, dist=0.15))
        #o2t3 = numpy.array([[ 0.90436331 , 0.  ,       -0.42676341 ,-1.10397823],
                            #[-0.42676341 , 0.  ,       -0.90436331 , 0.70348174],
                            #[ 0.        ,  1.  ,        0.        ,  0.83839377],
                            #[ 0.        ,  0.  ,        0.        ,  1.        ]])
        #posar2.poses.append(self.topose(o2t3))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o2t3, dist=0.15))
        #o2t4 = numpy.array([[ 0.94496101 , 0.      ,   -0.32718297 ,-1.12638383],
                            #[-0.32718297 , 0.     ,    -0.94496101 , 0.71261623],
                            #[ 0.         , 1.     ,     0.        ,  0.83345649],
                            #[ 0.         , 0.     ,     0.        ,  1.        ]])
        #posar2.poses.append(self.topose(o2t4))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o2t4, dist=0.15))
        #o2t5 = numpy.array([[ 0.759732  ,  0.     ,    -0.65023634, -1.05369682],
                            #[-0.65023634 , 0.    ,     -0.759732  ,  0.6709397 ],
                            #[ 0.       ,   1.    ,      0.     ,     0.83430555],
                            #[ 0.       ,   0.    ,      0.     ,     1.        ]])
        #posar2.poses.append(self.topose(o2t5))  
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o2t5, dist=0.15))
        #o2t6 = numpy.array([[ 0.37992052 , 0.     ,    -0.92501913, -0.99187069],
                            #[-0.92501913 , 0.     ,    -0.37992052,  0.58548212],
                            #[ 0.         , 1.     ,     0.        ,  0.84069892],
                            #[ 0.        ,  0.     ,     0.       ,   1.        ]])
        #posar2.poses.append(self.topose(o2t6)) 
        #self.tsrviz.append(openravepy.misc.DrawAxes(self.env, o2t6, dist=0.15))
        #tsrmsg.poses.append(posar2)   
        
        #return tsrmsg
