HumanPy
======

HumanPy is a Python library for interacting with an OpenRave Model of a Human through OpenRAVE. HumanPy expands the robot-agnostic helper library [PrPy]
(https://github.com/personalrobotics/prpy) by wrapping Human-specific functionality. This software was created by the [Personal Robotics Lab] (https://personalrobotics.ri.cmu.edu) at [Carnegie Mellon University] (http://www.cmu.edu). 

## Running HumanPy ##
You use HumanPy in your script by simply calling the ``initialize`` function:

```python
env, robot = humanpy.initialize(attach_viewer=False, sim=True, user_id='human', env=None)
```

This function have three paremeters: sim, user_id and env
``sim=True``: 
it allows the use of the human in simulation, thus as a robot

``sim=False``:
it allows the use of the human as a representative of a real human that is present in the environment. The Virtual human is moved on the basis fo the data coming from the kinect

``user_id=id`` :
it is required in order to have different humans in the same environment. It has to be defined both for ``sim=True`` and ``sim=False``
env is the environment. It allows to use of an environmet containing herb. If env is not set, a new environment is generated.

You can also optionally attach a viewer to the OpenRave environment by passing ``attach_viewer=True``.


