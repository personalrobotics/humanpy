cmake_minimum_required(VERSION 2.8.3)
project(humanpy)




find_package(catkin REQUIRED COMPONENTS
   #roscpp
   rospy
   #std_msgs
   #geometry_msgs
   #message_generation
)
catkin_python_setup()

# add_message_files(
#   FILES
#   PoseArrays.msg
# )

# generate_messages(
#     DEPENDENCIES
#         std_msgs
#         geometry_msgs
# )

catkin_package(
    #CATKIN_DEPENDS
        #roscpp
        #std_msgs
        #geometry_msgs
        #message_runtime
)





