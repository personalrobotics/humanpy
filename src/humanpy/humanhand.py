import numpy
from openravepy import KinBody, PlannerStatus, Robot
from prpy import util
from prpy.base import EndEffector
from prpy.exceptions import PrPyException

#TODO how to compute the reducing factor and is the rest correct
#TODO easiest interface for controlling complex hand
def compute_finger_ratios(primary_angle, reducing_factor=0.8):
    reduced_angle = primary_angle * reducing_factor
    return [primary_angle, reduced_angle, reduced_angle]

class HumanHand(EndEffector):
    def __init__(self, sim, manipulator):
        EndEffector.__init__(self, manipulator)

        robot = manipulator.GetRobot()
        env = robot.GetEnv()

        self.simulated = sim

        with env:
            accel_limits = robot.GetDOFAccelerationLimits()
            accel_limits[self.GetIndices()] = 1.0
            robot.SetDOFAccelerationLimits(accel_limits)

        if sim:
            self.controller = robot.AttachController(
                    name=self.GetName(),
                    args='',
                    dof_indices=self.GetIndices(),
                    affine_dofs=0,
                    simulated=True)
        else:
            raise NotImplementedError('Non-simulation mode not implemented')

    def CloneBinding(self, parent):
        super(HumanHand, self).CloneBindings(parent)
        self.simulated = True

    def MoveHand(self, f1=None, f2=None, f3=None, f4=None, f5=None, timeout=None):
        """ Change the hand preshape.
        
        This function blocks until trajectory
        execution finishes. This can be changed by changing the timeout 
        parameter to a maximum number of seconds. Pass zero to return
        instantly. 

        @param f1 thumb 
        @param f2 pinky finger
        @param f3 ring finger
        @param f4 middle finger
        @param f5 index finger
        @param timeout blocking execution timeout
        """

        robot = self.GetParent()

        with robot.GetEnv():
            sp = Robot.SaveParameters
            with robot.CreateRobotStateSaver(sp.ActiveDOF):
                robot.SetActiveDOFs(self.GetIndices())
                cspec = robot.GetActiveConfigurationSpecification('linear')
                current_preshape = self.GetDOFValues()

            desired_preshape = current_preshape.copy()
            if f1 is not None:
                angles = compute_finger_ratios(f1)
                desired_preshape[0:2] = angles[0:2]
                desired_preshape[4] = angles[2]
            if f2 is not None:
                angles = compute_finger_ratios(f2)
                desired_preshape[5:8] = angles[0:]
            if f3 is not None:
                angles = compute_finger_ratios(f3)
                desired_preshape[8:11] = angles[0:]
            if f4 is not None:
                angles = compute_finger_ratios(f4)
                desired_preshape[11] = angles[0]
                desired_preshape[2:4] = angles[1:]

        self.controller.SetDesired(desired_preshape)
        util.WaitForControllers([ self.controller ], timeout=timeout)

    def OpenHand(self, timeout=None):
        """Open the hand completely.

        @param timeout blocking execution timeout
        """
        self.MoveHand(f1=0, f2=0, f3=0, f4=0, f5=0, timeout=timeout)

    def CloseHand(self, tightness=1.0, timeout=None):
        """Close the hand.

        @param tightness angle of how tight to close hand
        @param timeout block execution of timeout
        """

        if tightness > 1.7: tightness = 1.7
        self.MoveHand(f1=tightness, f2=tightness, f3=tightness, f4=tightness, f5=tightness, timeout=timeout)
