HumanPy
======

HumanPy is a Python library for interacting with an OpenRave Model of a Human through OpenRAVE. HumanPy expands the robot-agnostic helper library [PrPy]
(https://github.com/personalrobotics/prpy) by wrapping Human-specific functionality. This software was created by the [Personal Robotics Lab] (https://personalrobotics.ri.cmu.edu) at [Carnegie Mellon University] (http://www.cmu.edu). 

## Running HumanPy ##
You use HumanPy in your script by simply calling the ``initialize`` function:

```python
env, robot = humanpy.initialize()
```

Humanpy has not yet been connected up to a real human so this functionality is only available in simulation, with the option ``sim=True``. You can also optionally attach a viewer to the OpenRave environment by passing ``attach_viewer=True``.

See ``humanpy.initialize()`` for the full list of initialization options.
