#!/usr/bin/python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import rospkg
from RP_CHEN_JINWEIJIANVICTOR.msg import user_msg
from std_msgs.msg import String, Int64
from RP_CHEN_JINWEIJIANVICTOR.srv import GetUserScore

class ResultNode:
    def __init__(self):
        """Initialize the result node"""
        rospy.init_node('result_node')
        
        # Store user information
        self.username = None
        
        # Create subscribers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.score_sub = rospy.Subscriber('result_information', Int64, self.score_callback)
        
        # Wait for the user_score service
        rospy.wait_for_service('user_score')
        self.get_score = rospy.ServiceProxy('user_score', GetUserScore)
        
        rospy.loginfo("Result node has started")
        
    def user_callback(self, data):
        """Callback for receiving user information"""
        self.username = data.username
        rospy.loginfo(f"Received user information for: {self.username}")
        
    def get_user_score_percentage(self):
        """Request score through service"""
        try:
            response = self.get_score(self.username)
            rospy.loginfo(f"User {self.username} score percentage: {response.score}%")
            print(f"\nScore Report:")
            print(f"Player: {self.username}")
            print(f"Score Percentage: {response.score}%")
            return response.score
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
        
    def score_callback(self, data):
        """Callback for receiving score information"""
        if self.username:
            score = data.data
            rospy.loginfo(f"Final Score for {self.username}: {score}")
            print(f"\nGame Over!")
            print(f"Player: {self.username}")
            print(f"Final Score: {score}")
            
            # Now that game is over, get the score percentage
            percentage = self.get_user_score_percentage()
            if percentage is not None:
                rospy.loginfo(f"Final score percentage: {percentage}%")
        
    def run(self):
        """Main node logic"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # You could add periodic checks or operations here if needed
            rate.sleep()
            
        rospy.spin()

if __name__ == '__main__':
    try:
        result_node = ResultNode()
        result_node.run()
    except rospy.ROSInterruptException:
        pass