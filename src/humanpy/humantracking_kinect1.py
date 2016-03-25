PACKAGE = 'humanpy'
import numpy
import rospy
import rospkg
import collections
from scipy import signal
from tf import transformations, LookupException, ConnectivityException, ExtrapolationException
import humanpy
import utils

#Butterworth digital and analog filter design
N=4
WN=0.01
OUTPUT='ba'
MAXLEN = 300         #maximum number of samples to be considered when fitlering
FILTERING_SAMPLES=15 #number of samples to be in the vector before starting filtering

    
class Orhuman(object):
    def __init__(self, id, env, 
                 base_frame='/map',
                 kin_frame='/head/skel_depth_frame',
                 **kw_args):
        assert id != ''
        self.id = id
        self.enabled = True
        self.env = env
        self.base_frame = base_frame
        self.kin_frame = kin_frame
        _,self.body = humanpy.initialize(sim=False, user_id=id, env=env)
        with env:
            self.initialDOFvalues = self.body.GetDOFValues()
            self.currentDOFvalues = self.body.GetDOFValues()        
        self.starting_angle = dict()
        self.last_updated = rospy.get_rostime().secs
        self.collshl = collections.deque(maxlen=MAXLEN)
        self.collsh2l = collections.deque(maxlen=MAXLEN)
        self.collsh3l = collections.deque(maxlen=MAXLEN)
        self.collell = collections.deque(maxlen=MAXLEN)
        self.collshr = collections.deque(maxlen=MAXLEN)
        self.collsh2r = collections.deque(maxlen=MAXLEN)
        self.collsh3r = collections.deque(maxlen=MAXLEN)
        self.collelr = collections.deque(maxlen=MAXLEN)
        self.collthl = collections.deque(maxlen=MAXLEN)
        self.collcal = collections.deque(maxlen=MAXLEN)
        self.collthr = collections.deque(maxlen=MAXLEN)
        self.collcar = collections.deque(maxlen=MAXLEN)
        #N=2 or 4, Wn = 0.01-0.1.  0<Wn<1
        self.B, self.A = signal.butter(N, WN, output=OUTPUT)

    def getUpperLimbAngles(self, tf, side):
        """
        Defines the joint angles of the human arm, starting from the position
        of the tf generated accordin to the data coming from the kinect
        
        @param tf tf
        @param 'L' for left upper limb, 'R' for right upper limb
        """
        if side == 'L':        
            #4: elbow (-x), #3: shoulder 3 (y), #2: shoulder 2 (z), #1: shoulder (-x) in kin frame
            self.last_updated, sys_shoulder = utils.getSkeletonTransformation(self.id, tf, 'left_shoulder', self.kin_frame, self.last_updated)
            self.last_updated, sys_elbow = utils.getSkeletonTransformation(self.id, tf, 'left_elbow', self.kin_frame, self.last_updated)
            self.last_updated, sys_hand = utils.getSkeletonTransformation(self.id, tf, 'left_hand', self.kin_frame, self.last_updated)
        else:
            #4: elbow (-x), #3: shoulder 3 (-y), #2: shoulder 2 (-z), #1: shoulder (-x)  in kin frame
            self.last_updated, sys_shoulder = utils.getSkeletonTransformation(self.id, tf, 'right_shoulder', self.kin_frame, self.last_updated)
            self.last_updated, sys_elbow = utils.getSkeletonTransformation(self.id, tf, 'right_elbow', self.kin_frame, self.last_updated)
            self.last_updated, sys_hand = utils.getSkeletonTransformation(self.id, tf, 'right_hand', self.kin_frame, self.last_updated)
           
        if sys_shoulder is None or sys_elbow is None or sys_hand is None:
            return None
        
        sys_sh_elb = numpy.dot(numpy.linalg.inv(sys_shoulder), sys_elbow)
        
        vect_es = (sys_shoulder[0:3,3] - sys_elbow[0:3,3])/ \
                  numpy.linalg.norm([sys_shoulder[0:3,3] - sys_elbow[0:3,3]])
        vect_he = (sys_elbow[0:3,3] - sys_hand[0:3,3])/ \
                  numpy.linalg.norm([sys_elbow[0:3,3] - sys_hand[0:3,3]])
        vect_norm_es_eh = numpy.cross(vect_he, vect_es)
        
        q4 = - numpy.arccos(numpy.asscalar(numpy.dot(vect_he.T,vect_es)))                   #[0,-pi]
        
        if side == 'L': 
            q2 = numpy.asscalar(numpy.arcsin(utils.checkArg(-vect_es[0])))                   #[-pi/2,pi/2]
        else:
            q2 = numpy.asscalar(numpy.arcsin(utils.checkArg(vect_es[0]))) 
        if sys_sh_elb[1,3] > 0:                                                             #projection on y of the sys origin. if y positive, add pi/2
            q2 = q2 + numpy.sign(q2)*numpy.pi/2                                             #[-pi,pi]

        if numpy.abs(numpy.cos(q2)) > 0.1:
            q1 = numpy.asscalar(numpy.arccos(utils.checkArg(vect_es[1]/numpy.cos(q2))))      #[0,pi]
            if numpy.asscalar(numpy.arcsin(utils.checkArg(-vect_es[2]/numpy.cos(q2)))) < 0:  #[-pi,pi]
                q1 = -q1
            
            q3 = numpy.asscalar(numpy.arccos(utils.checkArg(-vect_norm_es_eh[0]/numpy.cos(q2))))                
        else:
            q1 = None
            q3 = None

        return [q1, q2, q3, q4]

    def getLowerLimbAngles(self, tf, side):
        """
        Defines the joint angles of the human legs, starting from the position
        of the tf generated accordin to the data coming from the kinect
        
        @param tf tf
        @param 'L' for left lower limb, 'R' for right lower limb
        """
        if side == 'L':        
            self.last_updated, sys_hip = utils.getSkeletonTransformation(self.id, tf, 'left_hip', self.kin_frame, self.last_updated)
            self.last_updated, sys_knee = utils.getSkeletonTransformation(self.id, tf, 'left_knee', self.kin_frame, self.last_updated)
            self.last_updated, sys_foot = utils.getSkeletonTransformation(self.id, tf, 'left_foot', self.kin_frame, self.last_updated)
        else:            
            self.last_updated, sys_hip = utils.getSkeletonTransformation(self.id, tf, 'right_hip', self.kin_frame, self.last_updated)
            self.last_updated, sys_knee = utils.getSkeletonTransformation(self.id, tf, 'right_knee', self.kin_frame, self.last_updated)
            self.last_updated, sys_foot = utils.getSkeletonTransformation(self.id, tf, 'right_foot', self.kin_frame, self.last_updated)
            
        if sys_hip is None or sys_knee is None or sys_foot is None:
            return None
        
        vect_kh = (sys_hip[0:3,3] - sys_knee[0:3,3])/  \
                  numpy.linalg.norm([sys_hip[0:3,3] - sys_knee[0:3,3]])
        vect_fk = (sys_knee[0:3,3] - sys_foot[0:3,3])/ \
                  numpy.linalg.norm([sys_knee[0:3,3] - sys_foot[0:3,3]])
        q2 = - numpy.arccos(utils.checkArg(numpy.asscalar(numpy.dot(vect_kh.T,vect_fk))))
        
        q1 = numpy.asscalar(numpy.arccos(vect_kh[1]))                                       #[0,pi]
        if numpy.asscalar(numpy.arcsin(vect_kh[2])) < 0:                                    #[-pi,pi]
            q1 = -q1 
            
        return [q1, q2]
  
    def checkLimits(self, joint, angle_vect, angle_elem, collection):
        """
        Check if the identify joint value is in the range imposed by the human
        loded into the simulation env. If not, the joint value is adjusted. 
        Joint value is finally filtered.
        
        @param joint joint
        @param angle_vector vector of the joint values
        @param angle_elem position of the joint value to be checked in angle_vector
        """
        if angle_vect is not None:
            angle = angle_vect[angle_elem]
            if angle is not None and joint is not None:
                lower,upper = joint.GetLimits()
                angle = utils.constrain(angle, lower, upper)
                
                collection.append(float(angle))
                if len(collection) > FILTERING_SAMPLES:
                    filtered_angles = signal.filtfilt(self.B, self.A, list(collection))
                    filtered_angle = filtered_angles[len(collection)-1]
                    if filtered_angle < lower: filtered_angle = lower
                    if filtered_angle > upper: filtered_angle = upper
                    self.currentDOFvalues[joint.GetDOFIndex()] = filtered_angle     
                    with self.env:
                        self.body.SetDOFValues(self.currentDOFvalues)

    def update(self, tf, time_to_wait):
        """
        Update the joint valued of human in the simualted environment
        on the basis of the position of the reference systems coming from
        the kinect
        
        @param tf tf
        @param time_to_wait determine the frequency of the update. Expressed in seconds.
        """ 
        time_since_last_update = rospy.get_rostime().secs - self.last_updated
        if self.enabled and time_since_last_update > time_to_wait:
            self.enabled = utils.hide(self.env, self.body)
        elif not self.enabled and time_since_last_update <= time_to_wait:
            self.enabled = utils.show(self.env, self.body)
        
        #Chest
        self.last_updated, person_transform = utils.getSkeletonTransformation(self.id, tf, 'torso', self.base_frame, self.last_updated)
        if person_transform is not None: 
            #human face to kinect
            person_orient_change =  numpy.array([[ -1. ,  0. ,  0. ,  0.],
                                                [   0. ,  1. ,  0. ,  0.],
                                                [   0. ,  0. , -1. ,  0.],
                                                [   0. ,  0. ,  0. ,  1. ]])
            person_transform = numpy.dot(person_transform, person_orient_change)
            
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
        
        #Left arm
        ul_angles = self.getUpperLimbAngles(tf, 'L')
        self.checkLimits(self.body.GetJoint('JLShoulder'), ul_angles, 0, self.collshl)
        self.checkLimits(self.body.GetJoint('JLShoulder2'), ul_angles, 1, self.collsh2l)
        self.checkLimits(self.body.GetJoint('JLShoulder3'), ul_angles, 2, self.collsh3l)
        self.checkLimits(self.body.GetJoint('JLForearm'), ul_angles, 3, self.collell)

        #Right arm
        ul_angles = self.getUpperLimbAngles(tf, 'R')
        self.checkLimits(self.body.GetJoint('JRShoulder'), ul_angles, 0, self.collshr)
        self.checkLimits(self.body.GetJoint('JRShoulder2'), ul_angles, 1, self.collsh2r)
        self.checkLimits(self.body.GetJoint('JRShoulder3'), ul_angles, 2, self.collsh3r)        
        self.checkLimits(self.body.GetJoint('JRForearm' ), ul_angles, 3, self.collelr)
            
        #Left leg
        ul_angles = self.getLowerLimbAngles(tf, 'L')
        self.checkLimits(self.body.GetJoint('JLThigh'), ul_angles, 0, self.collthl)
        self.checkLimits(self.body.GetJoint('JLCalf'), ul_angles, 1, self.collcal)

        #Right leg
        ul_angles = self.getLowerLimbAngles(tf, 'R')
        self.checkLimits(self.body.GetJoint('JRThigh'), ul_angles, 0, self.collthr)
        self.checkLimits(self.body.GetJoint('JRCalf'), ul_angles, 1, self.collcar)
                
        #TODO; head, neck   


def addRemoveHumans(tf, humans, env,                     
                    base_frame='/map',
                    kin_frame='/head/skel_depth_frame',
                    **kw_args):
        
    """
    Add or remove humans in the simulated environment on the basis 
    of the information coming from the kinect
    
    @param tf tf
    @param humans vector of the humans that are loaded into the simualted environment
    @base_frame reference frame of the environment
    @kin_frame frame respect to all the tf coming from the kinect are publisched
    """ 

    all_human_ids = utils.humanList(tf)
    utils.removeHumans(all_human_ids, humans, env)
        
    # adding
    for id in all_human_ids:
        found = False
        for human in humans:
            if human.id == 'user_' + id:
                found = True
                break
        if not found:
            humans.append(Orhuman('user_' + id, env,
                                  base_frame='/map',
                                  kin_frame='/head/skel_depth_frame'))

