PACKAGE = 'humanpy'
from tf import transformations, LookupException, ConnectivityException, ExtrapolationException


def constrain(x, x_min, x_max):
    """
    Check if a number is in the range. If not, 
    the min or max values are returned.
    
    @param x value to be checked
    @param x_min minimum range value
    @parem x_max maximum range value
    """
    return min(max(x, x_min), x_max)

def humanInList(human, ids):
    """
    Check if the given human id matched one of the id in the list ids
    
    @param human human loaded in the env
    @param ids list of id to be checked
    """
    for id in ids:
        if human.id == 'user_' + id: return True
    return False
        
def define_pose_from_eetransform(ee_pose):
    """
    Transform a pose in a Pose()
    
    @param ee_pose end effector pose
    """
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

def define_pose_from_posquat(pos, quat):
    """
    Transform a position and a quaternion in a Pose()
    
    @param pos x y z position in meters
    @param quat quaternion
    """
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]        
    return pose  

def hide(env, body):
    """
    Hide the human in the simulation env
    
    @param env environment
    @param body human representation in the env
    """
    with env:
        body.SetVisible(False)
        body.Enable(False)
    return False

def show(env, body):
    """
    Make visible the human in the simulation env
    
    @param env environment
    @param body human representation in the env
    """
    with env:
        body.SetVisible(True)
        body.Enable(True)
    return True

def getFullTfName(id, link_name):
    """
    Return the full name of the link according to tf tree
    
    @param id human id
    @param link_name name of the link in the tf tree
    """
    return '/skel/' + id + '/' + link_name
            
def checkArg(arg):
    """
    Approximated a value 
    to -1 o 1 if the number is not in the range [-1,1]
    
    @param arg value to be checked
    """
    if arg > 1.0: arg = 1.0
    if arg < -1.0: arg = -1.0
    return arg

def getSkeletonTransformation(user_id, tf, tf_name, world_name, last_updated):
    """
    Return the rototranslation matrix between tf_name and world_name 
    and the time stamp
    
    @param user_id id of the human
    @param tf 
    @param tf_name name of the element in the tf tree
    @param world_name name of the world in the tf tree
    @param last_update time stamp of the last time the tf tree was updated
    """
    
    tf_name_full = getFullTfName(user_id, tf_name)
    if tf.frameExists(tf_name_full) and tf.frameExists(world_name):   
        try:                
            time = tf.getLatestCommonTime(tf_name_full, world_name)
            pos, quat = tf.lookupTransform(world_name, tf_name_full, time)
        except:
            return last_updated, None
        last_updated = time.secs if last_updated <= time.secs else last_updated
        r = transformations.quaternion_matrix(quat)
        skel_trans = transformations.identity_matrix()
        skel_trans[0:4,0:4] = r
        skel_trans[0:3, 3] = pos    
        return last_updated, skel_trans
    else:
        return last_updated, None
    
def humanList(tf):
    """
    return the list of the human id that are in the tf tree 
    
    @param tf 
    """
    import re
    matcher = re.compile('.*user_(\\d+).*')
    all_tfs = tf.getFrameStrings()
    all_human_ids = []
    for frame_name in all_tfs:
        match = matcher.match(frame_name)
        if match is not None:
            all_human_ids.append(match.groups()[0])
            
    return all_human_ids

def removeHumans(all_human_ids, humans, env):
    """
    Update the list of the human in the env in the basis of all_human_ids
    
    @param all_human_ids list of the human id that are in the tf tree 
    @param humans list of the humans represented in the environment
    @param env environment
    """
    # removing
    humans_to_remove = [x for x in humans if not humanInList(x, all_human_ids)]
    for human in humans_to_remove:
        with env:
            env.RemoveKinBody(human);
        human.destroy()
        humans.remove(human)
