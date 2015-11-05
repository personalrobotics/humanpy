PACKAGE = 'humanpy'
import numpy
import rospy
import rospkg
import collections
from scipy import signal
from tf import transformations, LookupException, ConnectivityException, ExtrapolationException
import humanpy

BASE_FRAME = '/map'
KIN_FRAME = '/head/skel_depth_frame'

    
class Orhuman(object):
    def __init__(self, id, env):
        assert id != ''
        self.id = id
        self.enabled = True
        _,self.body = humanpy.initialize(sim=False, user_id=id, env=env)
        self.initialDOFvalues = self.body.GetDOFValues()
        self.currentDOFvalues = self.body.GetDOFValues()        
        self.starting_angle = dict()
        self.last_updated = rospy.get_rostime().secs
        self.collshl = collections.deque(maxlen=300)
        self.collsh2l = collections.deque(maxlen=300)
        self.collsh3l = collections.deque(maxlen=300)
        self.collell = collections.deque(maxlen=300)
        self.collshr = collections.deque(maxlen=300)
        self.collsh2r = collections.deque(maxlen=300)
        self.collsh3r = collections.deque(maxlen=300)
        self.collelr = collections.deque(maxlen=300)
        self.collthl = collections.deque(maxlen=300)
        self.collcal = collections.deque(maxlen=300)
        self.collthr = collections.deque(maxlen=300)
        self.collcar = collections.deque(maxlen=300)
        #N=2 o 4, Wn = 0.01-0.1 cmq comrepso tra 0 e 1
        self.B, self.A = signal.butter(4, 0.01, output='ba')
       


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
            sys_shoulder = self.getSkeletonTransformation(tf, 'left_shoulder', KIN_FRAME)
            sys_elbow = self.getSkeletonTransformation(tf, 'left_elbow', KIN_FRAME)
            sys_hand = self.getSkeletonTransformation(tf, 'left_hand', KIN_FRAME)
        else:
            #4: elbow (-x), #3: shoulder 3 (-y), #2: shoulder 2 (-z), #1: shoulder (-x)  in kin frame
            sys_shoulder = self.getSkeletonTransformation(tf, 'right_shoulder', KIN_FRAME)
            sys_elbow = self.getSkeletonTransformation(tf, 'right_elbow', KIN_FRAME)
            sys_hand = self.getSkeletonTransformation(tf, 'right_hand', KIN_FRAME)
           
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
            q2 = numpy.asscalar(numpy.arcsin(self.checkArg(-vect_es[0])))                   #[-pi/2,pi/2]
        else:
            q2 = numpy.asscalar(numpy.arcsin(self.checkArg(vect_es[0]))) 
        if sys_sh_elb[1,3] > 0:                                                             #projection on y of the sys origin. if y positive, add pi/2
            q2 = q2 + numpy.sign(q2)*numpy.pi/2                                             #[-pi,pi]

        if numpy.abs(numpy.cos(q2)) > 0.1:
            q1 = numpy.asscalar(numpy.arccos(self.checkArg(vect_es[1]/numpy.cos(q2))))      #[0,pi]
            if numpy.asscalar(numpy.arcsin(self.checkArg(-vect_es[2]/numpy.cos(q2)))) < 0:  #[-pi,pi]
                q1 = -q1
            
            q3 = numpy.asscalar(numpy.arccos(self.checkArg(-vect_norm_es_eh[0]/numpy.cos(q2))))                
        else:
            q1 = None
            q3 = None

        return [q1, q2, q3, q4]

    def getLowerLimbAngles(self, tf, side):
        if side == 'L':        
            sys_hip = self.getSkeletonTransformation(tf, 'left_hip', KIN_FRAME)
            sys_knee = self.getSkeletonTransformation(tf, 'left_knee', KIN_FRAME)
            sys_foot = self.getSkeletonTransformation(tf, 'left_foot', KIN_FRAME)
        else:            
            sys_hip = self.getSkeletonTransformation(tf, 'right_hip', KIN_FRAME)
            sys_knee = self.getSkeletonTransformation(tf, 'right_knee', KIN_FRAME)
            sys_foot = self.getSkeletonTransformation(tf, 'right_foot', KIN_FRAME)
            
        if sys_hip is None or sys_knee is None or sys_foot is None:
            return None
        
        vect_kh = (sys_hip[0:3,3] - sys_knee[0:3,3])/  \
                  numpy.linalg.norm([sys_hip[0:3,3] - sys_knee[0:3,3]])
        vect_fk = (sys_knee[0:3,3] - sys_foot[0:3,3])/ \
                  numpy.linalg.norm([sys_knee[0:3,3] - sys_foot[0:3,3]])
        q2 = - numpy.arccos(self.checkArg(numpy.asscalar(numpy.dot(vect_kh.T,vect_fk))))
        
        q1 = numpy.asscalar(numpy.arccos(vect_kh[1]))                                       #[0,pi]
        if numpy.asscalar(numpy.arcsin(vect_kh[2])) < 0:                                    #[-pi,pi]
            q1 = -q1 
            
        return [q1, q2]
  
    def checkLimits(self, joint, angle_vect, angle_elem, collection):
        if angle_vect is not None:
            angle = angle_vect[angle_elem]
            if angle is not None and joint is not None:
                lower,upper = joint.GetLimits()
                if angle < lower: angle = lower
                if angle > upper: angle = upper
                
                collection.append(float(angle))
                if len(collection) > 15:
                    filtered_angles = signal.filtfilt(self.B, self.A, list(collection))
                    #print 'collection'
                    #print collection
                    #print 'filtered_angles'
                    filtered_angle = filtered_angles[len(collection)-1]
                    if filtered_angle < lower: filtered_angle = lower
                    if filtered_angle > upper: filtered_angle = upper
                    self.currentDOFvalues[joint.GetDOFIndex()] = filtered_angle           
                    self.body.SetDOFValues(self.currentDOFvalues)
        
    def update(self, tf):
        TIME_TO_WAIT = 1
        time_since_last_update = rospy.get_rostime().secs - self.last_updated
        #if self.enabled and time_since_last_update > TIME_TO_WAIT:
            #self.hide()
        #elif not self.enabled and time_since_last_update <= TIME_TO_WAIT:
            #self.show()
        
        #Chest
        person_transform = self.getSkeletonTransformation(tf, 'torso')
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
        


def humanInList(human, ids):
    for id in ids:
        if human.id == 'user_' + id: return True
    return False
        
def addRemoveHumans(tf, humans, env):
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
            humans.append(Orhuman('user_' + id, env))

