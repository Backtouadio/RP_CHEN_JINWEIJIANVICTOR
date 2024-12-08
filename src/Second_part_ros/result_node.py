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
        
        # Store user and game information
        self.username = None
        self.name = None
        self.current_phase = "phase1"
        self.high_score = 0  # Track high score
        self.games_played = 0  # Track number of games played
        
        # Create subscribers
        self.user_sub = rospy.Subscriber('user_information', user_msg, self.user_callback)
        self.score_sub = rospy.Subscriber('result_information', Int64, self.score_callback)
        self.phase_sub = rospy.Subscriber('game_phase', String, self.phase_callback)
        
        # Wait for the user_score service
        rospy.wait_for_service('user_score')
        self.get_score = rospy.ServiceProxy('user_score', GetUserScore)
        
        rospy.loginfo("Result node has started")
        
    def user_callback(self, data):
        """Callback for receiving user information"""
        self.username = data.username
        self.name = data.name
        rospy.loginfo(f"Received user information for: {self.name} ({self.username})")
        
    def phase_callback(self, data):
        """Track game phase changes"""
        new_phase = data.data
        if new_phase != self.current_phase:
            self.current_phase = new_phase
            if new_phase == "phase2":
                rospy.loginfo("Game started!")
            elif new_phase == "phase3":
                rospy.loginfo("Game ended!")
        
    def calculate_performance_metrics(self, score):
        """Calculate various performance metrics"""
        # Update high score
        if score > self.high_score:
            self.high_score = score
        
        # Update games played
        self.games_played += 1
        
        # Calculate average score
        average = self.high_score / self.games_played if self.games_played > 0 else 0
        
        return {
            'high_score': self.high_score,
            'games_played': self.games_played,
            'average_score': average
        }
        
    def get_user_score_percentage(self):
        """Request score through service and calculate percentage"""
        try:
            response = self.get_score(self.username)
            score_percentage = response.score
            
            rospy.loginfo(f"\nScore Analysis for {self.username}:")
            rospy.loginfo(f"Score Percentage: {score_percentage}%")
            
            # Print to terminal for user
            print(f"\nScore Report:")
            print(f"Player: {self.name} ({self.username})")
            print(f"Score Percentage: {score_percentage}%")
            
            return score_percentage
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
        
    def score_callback(self, data):
        """Callback for receiving score information"""
        if self.username and self.current_phase == "phase3":
            final_score = data.data
            
            # Calculate performance metrics
            metrics = self.calculate_performance_metrics(final_score)
            
            # Get score percentage
            score_percentage = self.get_user_score_percentage()
            
            # Display comprehensive game report
            print("\nGame Over! Final Report:")
            print("------------------------")
            print(f"Player: {self.name} ({self.username})")
            print(f"Final Score: {final_score}")
            if score_percentage is not None:
                print(f"Performance: {score_percentage}%")
            print(f"High Score: {metrics['high_score']}")
            print(f"Games Played: {metrics['games_played']}")
            print(f"Average Score: {metrics['average_score']:.2f}")
            print("------------------------")
            
            rospy.loginfo(f"Final Score for {self.username}: {final_score}")
        
    def run(self):
        """Main node logic"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        result_node = ResultNode()
        result_node.run()
    except rospy.ROSInterruptException:
        pass