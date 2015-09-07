import prpy
import logging
import openravepy
from prpy.action import ActionMethod

logger = logging.getLogger('humanpy')

@ActionMethod
def Grasp(robot, obj, manip=None, preshape=0.5, 
                  tsrlist=None, render=True, **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param manip The manipulator to perform the grasp with 
                    (if None active manipulator is used)
    @param preshape The grasp preshape for the hand
    @param tsrlist A list of TSRChain objects to use for planning to grasp pose
                    (if None, the 'grasp' tsr from tsrlibrary is used)
    @param render Render tsr samples and push direction vectors during planning
    """
    HumanGrasp(robot, obj,  manip=manip, preshape=preshape, 
                    tsrlist=tsrlist, render=render)

@ActionMethod
def PushGrasp(robot, obj, push_distance=0.1, manip=None, 
                preshape=0.5, push_required=True, 
                tsrlist=None, render=True, **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param distance The distance to push before grasping
    @param manip The manipulator to perform the grasp with 
                    (if None active manipulator is used)
    @param push_required If true, throw exception if a plan for the pushing 
                        movement cannot be found. If false, continue with grasp even if push 
                        cannot be executed.
    @param preshape The grasp preshape for the hand
    @param tsrlist A list of TSRChain objects to use for planning to grasp pose
                    (if None, the 'grasp' tsr from tsrlibrary is used)
    @param render Render tsr samples and push direction vectors during planning
    """
    if tsrlist is None:
        tsrlist = robot.tsrlibrary(obj, 'push_grasp', push_distance=push_distance)

    HumanGrasp(robot, obj, manip=manip, preshape=preshape, 
               push_distance=push_distance,
               tsrlist=tsrlist, render=render)

def HumanGrasp(robot, obj, push_distance=None, manip=None, 
                preshape=0.5, push_required=False, 
                tsrlist=None, render=True, **kw_args):
    """
    @param robot The robot performing the push grasp
    @param obj The object to push grasp
    @param distance The distance to push before grasping (if None, no pushing)
    @param manip The manipulator to perform the grasp with 
                    (if None active manipulator is used)
    @param preshape The grasp preshape for the hand
    @param push_required If true, throw exception if a plan for the pushing 
                            movement cannot be found. If false, continue with grasp even if push 
                            cannot be executed. (only used if distance is not None)
    @param render Render tsr samples and push direction vectors during planning
    """
    if manip is None:
        manip = robot.GetActiveManipulator()

    # Move the hand to the grasp preshape
    manip.hand.OpenHand()

    # Get the grasp tsr
    if tsrlist is None:
        tsrlist = robot.tsrlibrary(obj, 'grasp')

    # Plan to the grasp
    with prpy.viz.RenderTSRList(tsrlist, robot.GetEnv(), render=render):
        manip.PlanToTSR(tsrlist, execute=True)
    
    if push_distance is not None:
        ee_in_world = manip.GetEndEffectorTransform()
        push_direction = ee_in_world[:3,2]

        # Move the object into the hand
        env = robot.GetEnv()
        with env:
            obj_in_world = obj.GetTransform()

            # First move back until collision
            stepsize = 0.01
            total_distance = 0.0
            while not env.CheckCollision(robot, obj) and total_distance <= push_distance:
                obj_in_world[:3,3] -= stepsize*push_direction
                total_distance += stepsize
                obj.SetTransform(obj_in_world)

            # Then move forward until just out of collision
            stepsize = 0.001
            while env.CheckCollision(robot, obj):
                obj_in_world[:3,3] += stepsize*push_direction
                obj.SetTransform(obj_in_world)

        # Manipulator must be active for grab to work properly
        p = openravepy.KinBody.SaveParameters
        with robot.CreateRobotStateSaver(p.ActiveManipulator):
            robot.SetActiveManipulator(manip)
            robot.Grab(obj)

        # Now execute the straight line movement
        with prpy.viz.RenderVector(ee_in_world[:3,3], push_direction,
                                   push_distance, robot.GetEnv(), render=render):
            try:
                manip.PlanToEndEffectorOffset(direction = push_direction,
                                              distance = push_distance,
                                              **kw_args)
            except PlanningError, e:
                if push_required:
                    raise
                else:
                    logger.warn('Could not find a plan for straight line push. Ignoring.')
        robot.Release(obj)
    

    # Now close the hand to grasp
    manip.hand.CloseHand()

    # Manipulator must be active for grab to work properly
    p = openravepy.KinBody.SaveParameters
    with robot.CreateRobotStateSaver(p.ActiveManipulator):
        robot.SetActiveManipulator(manip)
        robot.Grab(obj)
