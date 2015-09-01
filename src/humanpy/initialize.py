PACKAGE = 'humanpy'
import logging
import os

from catkin.find_in_workspaces import find_in_workspaces
from openravepy import Environment, IkParameterizationType, RaveCreateController, RaveCreateModule
from openravepy.databases.inversekinematics import InverseKinematicsModel

#import humanpy.action # register actions
#import humany.tsr # register TSR libraries
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

logger = logging.getLogger('humanpy')

def initialize(attach_viewer=False):
    """Initialize the Human Robot"""
    
    prpy.logger.initialize_logging()

    env = Environment()

    #Setup Manipulators
    with env:
        robot = env.ReadKinBodyXMLFile('robots/man1.zae')
        env.AddKinBody(robot)

        robot.left_arm = robot.GetManipulator('leftarm')
        robot.left_arm.hand = robot.left_arm.GetEndEffector()
        robot.left_hand = robot.left_arm.hand

        robot.right_arm = robot.GetManipulator('rightarm')
        robot.right_arm.hand = robot.right_arm.GetEndEffector()
        robot.right_hand = robot.right_arm.hand

        bind_subclass(robot, Robot, robot_name='human')
        bind_subclass(robot.left_arm, Manipulator)
        bind_subclass(robot.right_arm, Manipulator)
        #subclasses for hand?

    #Setup Controller
    with env:
        controller_dof_indices = []
        controller_dof_indices.extend(robot.left_arm.GetArmIndices())
        controller_dof_indices.extend(robot.right_arm.GetArmIndices())

        controller = RaveCreateController(env, 'idealcontroller')
        robot.multicontroller.AttachController(
            controller, controller_dof_indices, 0)

    #Setup IK
    with env:
        ikmodel_left = InverseKinematicsModel(
                robot,
                iktype=IkParameterizationType.Transform6D,
                manip=robot.left_arm
                )
        if not ikmodel_left.load():
            ikmodel_left.autogenerate()

        ikmodel_right = InverseKinematicsModel(
                robot,
                iktype=IkParameterizationType.Transform6D,
                manip=robot.right_arm
        )
        if not ikmodel_right.load():
            ikmodel_right.autogenerate()

    #Setup planning pipeline
    robot.planner = Sequence(
            SnapPlanner(),
            VectorFieldPlanner(),
            CBiRRTPlanner(),
    )
    robot.simplifier = None
    robot.retimer = HauserParabolicSmoother() # hack
    robot.smoother = HauserParabolicSmoother()

    #robot.actions = prpy.action.ActionLibrary()

    #Setup viewer. Default to load rviz
    if attach_viewer == True:
        attach_viewer = 'rviz'
        env.SetViewer('attach_viewer')

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

    return env, robot
