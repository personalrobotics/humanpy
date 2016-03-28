HumanPy
======

HumanPy is a Python library for interacting with an OpenRave Model of a Human through OpenRAVE. HumanPy expands the robot-agnostic helper library [PrPy]
(https://github.com/personalrobotics/prpy) by wrapping Human-specific functionality. This software was created by the [Personal Robotics Lab] (https://personalrobotics.ri.cmu.edu) at [Carnegie Mellon University] (http://www.cmu.edu). 

## Running HumanPy ##
You use HumanPy in your script by simply calling the ``initialize`` function:

```python
env, robot = humanpy.initialize(attach_viewer=False, sim=True, user_id='human', env=None)
```

This function have four paremeters: attach_viewer, sim, user_id and env.
`attach_viewer=True`: is used to optionally attach a viewer to the OpenRave environment 

`sim=True`: loads the human into the simualtion environment as a robot. Use the available motion planner to move the human.

`sim=False`: the human is loaded in the simulated environment and moved on the basis of the information comimg from the kinect. 

`user_id=id` : it is required in order to have different humans in the same environment. It has to be defined both for `sim=True` and `sim=False`
env is the environment. 

`env`: allows the use of an environmet in which also herb is loaded. If env is not set or is equal to None, a new environment is generated.




## Using humanpy with herb ##
`hum_herb.py` shows how to load the human and herb in the same environment. The human acts as a robot.

## Using humanpy with kinect 1 ##
`humtrackkinect1_herb.py` contains an example on how to use the data coming from the kinect 1 (through `openni2_tracker` - https://github.com/personalrobotics/openni2-tracker).
The example call the herb initialization function and than load the human robot. The human is moved accordingly to the data coming from the kinect.


## Using humanpy with kinect 2 ##
`humtrackkinect2_herb.launch` lanches the script `humtrackkinect2_herb.py` that contains an example on how to use the data coming from the kinect 2 (through `k2_client` - https://github.com/personalrobotics/k2_client and `k2_client_vis` - https://github.com/personalrobotics/k2_client_vis).
The example calls herb initialization function and than loads the human. The human is moved accordingly to the data coming from the kinect.
If herb localization is active (segway simulation is False), set the param seg_sim:=False:

    'roslaunch humanpy humtrackkinect2_herb.launch seg_sim:=False'
    
