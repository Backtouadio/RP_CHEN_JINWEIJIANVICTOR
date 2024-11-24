Launch all necessary nodes before game_node 

# without pygame
roscore
rosrun RP_CHEN_JINWEIJIANVICTOR result_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR control_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR info_user.py
rosrun RP_CHEN_JINWEIJIANVICTOR game_node.py

# with pygame
roscore
rosrun RP_CHEN_JINWEIJIANVICTOR result_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR control_py_node.py
rosrun RP_CHEN_JINWEIJIANVICTOR info_user.py
rosrun RP_CHEN_JINWEIJIAN game_node.py