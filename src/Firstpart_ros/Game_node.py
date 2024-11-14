#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import rospkg
import time
from std_msgs.msg import String, Bool
from robotinfo_msgs.msg import Robotinfomsg 
from robotinfo_msgs.msg import user_msg



class Game_node(object):
    def __init__(self):
        """
        Init method.
        """
        # Class variables

        self.user_information_sub = rospy.Subscriber("user_information", user_msg, self.user_info)
        self.keyboard_control_sub = rospy.Subscriber("keyboard_control", String, self.keyboard_control)




        #no me queda claro si asi esta bien
        self.result_information = rospy.Publisher("result_information", int, queue_size=10)



        time.sleep(10)
        #Start the code of the node
        self.main()

    def main (self):

        self.result_information.publish("finalscore")
        
    def user_info(self,data):
        rospy.loginfo(" Hello userinfor subscriber" )
        rospy.loginfo(" The data is [%user_msg]", data)


    def keyboard_control(self,data):
        rospy.loginfo(" Hello keyboard subscriber" )
        rospy.loginfo(" The data is [%s]", data)




if __name__ == '__main__':
    try:
        name_node = "Game_node"
        # start the node
        rospy.init_node(name_node)
        rospy.loginfo("Node %s has started", name_node)

        # create and spin the node
        node = Game_node()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass