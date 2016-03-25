HumanPy
======
Explaination of script files.

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
    

    
