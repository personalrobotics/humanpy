PACKAGE = 'humanpy'
import logging
import os

from openravepy import Environment, IkParameterizationType, RaveCreateController, RaveCreateModule, openrave_exception, RaveCreateMultiController
from openravepy.databases.inversekinematics import InverseKinematicsModel

from humanhand import HumanHand
import humanpy.action # register actions
import humanpy.tsr # register TSR libraries
import prpy
from prpy.base.endeffector import EndEffector
from prpy.base.manipulator import Manipulator
from prpy.base.robot import Robot
from prpy.bind import bind_subclass
from prpy.planning.base import Sequence
from prpy.planning.cbirrt import CBiRRTPlanner
from prpy.planning.vectorfield import VectorFieldPlanner
from prpy.planning.snap import SnapPlanner 
from prpy.planning.retimer import HauserParabolicSmoother
import numpy

logger = logging.getLogger('humanpy')

def initialize(attach_viewer=False, sim=True, user_id='human', env=None):
    """Initialize the Human Robot"""
    
    prpy.logger.initialize_logging()
    
    if not env:
        env = Environment()

    #Setup Manipulators
    with env:
        robot = env.ReadKinBodyXMLFile('robots/man1.dae')
        robot.SetName(user_id)              #needed in order to have different humans in the same env
        env.AddKinBody(robot) 

        robot.left_arm = robot.GetManipulator('leftarm')
        robot.left_arm.hand = robot.left_arm.GetEndEffector()
        robot.left_hand = robot.left_arm.hand

        robot.right_arm = robot.GetManipulator('rightarm')
        robot.right_arm.hand = robot.right_arm.GetEndEffector()
        robot.right_hand = robot.right_arm.hand

        bind_subclass(robot, Robot, robot_name=user_id)
        bind_subclass(robot.left_arm, Manipulator)
        bind_subclass(robot.right_arm, Manipulator)
    
        bind_subclass(robot.left_arm.hand, HumanHand, manipulator=robot.left_arm, sim=True)
        bind_subclass(robot.right_arm.hand, HumanHand, manipulator=robot.right_arm, sim=True)
        
        #Setup Controller
        args = 'IdealController'         
        affine_dofs=0
        
        delegate_controller = RaveCreateController(env, args)
        if delegate_controller is None:
            message = 'Creating controller {0:s} of type {1:s} failed.'.format(robot.right_arm.GetName(), args)
            raise openrave_exception(message)
        robot.multicontroller.AttachController(delegate_controller, robot.right_arm.GetArmIndices(), affine_dofs)
        
        delegate_controller = RaveCreateController(env, args)
        if delegate_controller is None:
            message = 'Creating controller {0:s} of type {1:s} failed.'.format(robot.left_arm.GetName(), args)
            raise openrave_exception(message)
        robot.multicontroller.AttachController(delegate_controller, robot.left_arm.GetArmIndices(), affine_dofs)

        #Setup IK
        ikmodel_left = InverseKinematicsModel(
                robot,
                iktype=IkParameterizationType.Transform6D,
                manip=robot.left_arm)
        if not ikmodel_left.load():
            ikmodel_left.autogenerate()

        ikmodel_right = InverseKinematicsModel(
                robot,
                iktype=IkParameterizationType.Transform6D,
                manip=robot.right_arm)
        
        if not ikmodel_right.load():            
            ikmodel_right.autogenerate()

        #Setup planning pipeline
        robot.planner = Sequence(
                SnapPlanner(),
                VectorFieldPlanner(),
                CBiRRTPlanner())
        robot.simplifier = None
        robot.retimer = HauserParabolicSmoother() # hack
        robot.smoother = HauserParabolicSmoother()
        
        robot.actions = prpy.action.ActionLibrary() 

    #Setup viewer. Default to load rviz
    if env.GetViewer() is None:
        if attach_viewer == True:
            attach_viewer = 'rviz'
            env.SetViewer(attach_viewer)

            #Fallback on qtcoin if rviz couldnt load
            if env.GetViewer is None:
                logger.warning('Loading the Rviz viewer failed. Falling back on qtcoin')
                attach_viewer = 'qtcoin'

        if attach_viewer and env.GetViewer() is None:
            env.SetViewer(attach_viewer)
            if env.GetViewer() is None:
                raise Exception('Failed creating viewer of type "{0:s}"'.format(
                attach_viewer))

    #Remove ROS Logging since loading Rviz might have added it
    prpy.logger.remove_ros_logger()
        
    #changing orientation
    with env:
        robotLocation = numpy.array([[ -1. ,  0. ,  0. ,   0.],
                                    [ 0. ,  0. ,  1. ,  0.],
                                    [ 0. ,  1. ,  0. ,   0.],
                                    [ 0. ,  0. ,  0. ,   1. ]])
        robot.SetTransform(robotLocation)

    return env, robot
