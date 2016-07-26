PACKAGE = 'humanpy'
import logging
import os

PACKAGE = 'humanpy'
import logging
import os
import rospy
from tf import TransformListener, transformations

logger = logging.getLogger('humanpy')

def DetectHuman(env, orhuman='kin2_or', load_hum=True,                           
                enable_legs=True, segway_sim=True, 
                base_frame='/map' , kin_frame='/head/skel_depth_frame2',
                hrc=False, herb_sim=True, 
                continuous=True, action='stamp'):
        """
        Use the data coming from the kinect in order to add a human 
        in the simulation environment. The parameters herb_sim, 
        continuous and action have to be set only if hrc=True.
        
        The function enters an infinite loop.
        
        @param env simulation environment
        @param orhuman equal to kin1_skel, if the data are coming from the kinect1 
                        and a skeleton is used to visualize the human
                       equal kin1_or, if the data are coming from the kinect1 
                        and the openrave human is used for visualization 
                       equal kin2_or, if the data are coming from the kinect2
                        and the openrave human is used for visualization 
        @param load_hum equal to True if the human is loaded in the simulated env
        @param enable_legs equal to False if the legs of the human cannot be tracked,
                           e.g. the legs are covered by a table     
        @param segway_sim equal to False if Herb localization is needed        
        @param hrc equal to True, if the package hrc is used
        @param herb_sim equal to True, if herb is simulated
        @param continuous equal to True, if human precition on goal is continuous.
                          equal to False, if human prediction stops when a certain 
                          threshold is reached.
        @param action if hrc=True, the kind of action herb is going to perform 
                      between stamping ('stamp') and grasping ('grasp')
        """

        if load_hum:
          if orhuman=='kin1_skel': 
            #kin 1 - load a skeleton
            import or_skeletons.load_skeletons as sk
            humans = []
            logger.info('Humans_tracking')
          elif orhuman=='kin1_or':
            #kin 1- load the human from openrave
            import humanpy.humantracking_kinect1 as sk
            humans = []
            logger.info('Humans_tracking')
          elif orhuman=='kin2_or':
            #kin 2 - load the human from openrave
            import humanpy.humantracking_kinect2 as sk
            humans = []
            logger.info('Humans_tracking')
          else:
            raise Exception('orhuman param is not correct')
           
        if orhuman=='kin2_or' and hrc:
          #kin 2 - load the human from openrave
          from hrc import assistance                 
          humans_hrc = []
          logger.info('HRC')
           
        try:  
            tf = TransformListener()  
            humanadded = False
            
            while not rospy.is_shutdown():   
                if load_hum:
                    sk.addRemoveHumans(tf, humans, env,
                                       enable_legs=enable_legs,
                                       segway_sim=segway_sim,
                                       base_frame=base_frame,
                                       kin_frame=kin_frame)
                    for human in humans:
                        human.update(tf, 1)       
                if hrc:
                    if not humanadded:
                        humanadded = assistance.addHumansPred(tf, humans_hrc, env,
                                                                continuous=continuous,
                                                                herb_sim=herb_sim,
                                                                segway_sim=segway_sim,
                                                                action=action)
                    for human in humans_hrc:
                        human.update(tf)                          
      
        except Exception, e:  
            logger.error('Detection failed update: %s' % str(e))
            raise