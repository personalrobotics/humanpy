HumanPy
======
Explaination of script files.

## Using humanpy with herb ##
`hum_herb.py` shows how to load the human and herb in the same environment. The human acts as a robot.

## Using humanpy with kinect 1 (deprecated) ##
`humtrackkinect1_herb.py` contains an example on how to use the data coming from the kinect 1 (through `openni2_tracker` - https://github.com/personalrobotics/openni2-tracker).
The example call the herb initialization function and than load the human robot. The human is moved accordingly to the data coming from the kinect.
As of June 2016, the Kinect on HERB does not use openni2-tracker. Therefore, the user should follow the instructions for Kinect 2 below. 



## Using humanpy with kinect 2 ##
`humtrackkinect2_herb.launch` lanches the script `humtrackkinect2_herb.py` that contains an example on how to use the data coming from the kinect 2 (through `k2_client` - https://github.com/personalrobotics/k2_client and `k2_client_vis` - https://github.com/personalrobotics/k2_client_vis).
The example calls herb initialization function and than loads the human. The human is moved accordingly to the data coming from the kinect.
If herb localization is active (segway simulation is False), set the param seg_sim:=False:
`roslaunch humanpy humtrackkinect2_herb.launch seg_sim:=False`
The process to launch the script is:

Physically connect a monitor to herb3, which is a small PC on top of herb0. 
Login with pr-demo and usual password. The OS of that PC is windows. Click on the kinect shortcut, to start the kinect server. A small icon on the bottom-right-hand side of the screen should appear, showing a small kinect running in green color. 

Switch to your machine. In your catking woskpace, do `rosmasterherb` then launch the launch file

On a new terminal, do `rosmasterherb` and then `rosrun rviz rviz`. In the rviz viewer, do add, by topic, and then `InteractiveMarkers`. HERB should appear. Stand in front of the Kinect with your hands extended to the sides. A human figure should appear in the viewer, following your motions. 



    
