PACKAGE = 'humanpy'
from tf import transformations, LookupException, ConnectivityException, ExtrapolationException


def constrain(x, x_min, x_max):
    return min(max(x, x_min), x_max)

def humanInList(human, ids):
    for id in ids:
        if human.id == 'user_' + id: return True
    return False
        
def define_pose_from_eetransform(ee_pose):
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
    with env:
        body.SetVisible(False)
        body.Enable(False)
    return False

def show(env, body):
    with env:
        body.SetVisible(True)
        body.Enable(True)
    return True

def getFullTfName(id, link_name):
    return '/skel/' + id + '/' + link_name
            
def checkArg(arg):
    if arg > 1.0: arg = 1.0
    if arg < -1.0: arg = -1.0
    return arg

def getSkeletonTransformation(user_id, tf, tf_name, world_name, last_updated):
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
    # removing
    humans_to_remove = [x for x in humans if not humanInList(x, all_human_ids)]
    for human in humans_to_remove:
        with env:
            env.RemoveKinBody(human);
        human.destroy()
        humans.remove(human)
