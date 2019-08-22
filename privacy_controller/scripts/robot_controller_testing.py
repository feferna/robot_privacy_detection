#!/usr/bin/env python

import rospy
import sys
import math
import time
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
#from move_base_msgs.msg import MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray

humanLocation = -1
canEnterRoom = False
reachedGoal = False
ask = False

sensitiveLocationReceived = False

waitingResponse = True
enteredRoom = False

goal_point = PoseStamped()

goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
voice_pub = rospy.Publisher("/robot_voice", String, queue_size=1)

def linearDistance(pose1, pose2):
    x = pose1.position.x - pose2.position.x
    y = pose1.position.y - pose2.position.y
    
    return math.sqrt(x**2 + y**2)

def setRobotGoal():
    global goal_point
    global reachedGoal
    global ask
    global waitingResponse
    global enteredRoom

    if(humanLocation == -1):
        goal_point.pose.position.x = -1.002
        goal_point.pose.position.y = 1.321
       	goal_point.pose.position.z = 0.0
        goal_point.pose.orientation.x = 0.0
       	goal_point.pose.orientation.y = 0.0
        goal_point.pose.orientation.z = -0.699
      	goal_point.pose.orientation.w = 0.715
        goal_pub.publish(goal_point)
        #ask = False
        reachedGoal = False

    if(humanLocation == 5 and not canEnterRoom and waitingResponse):
        goal_point.pose.position.x = 0.501
        goal_point.pose.position.y = -0.977
        goal_point.pose.position.z = 0.0
        goal_point.pose.orientation.x = 0.0
        goal_point.pose.orientation.y = 0.0
        goal_point.pose.orientation.z = 0.0
        goal_point.pose.orientation.w = 1.0
        goal_pub.publish(goal_point)

        time.sleep(12)

        goal_point.pose.position.x = -1.189
        goal_point.pose.position.y = -1.014
       	goal_point.pose.position.z = 0.0
        goal_point.pose.orientation.x = 0.0
       	goal_point.pose.orientation.y = 0.0
        goal_point.pose.orientation.z = -0.007
      	goal_point.pose.orientation.w = 1.000
        goal_pub.publish(goal_point)
        print("Human is in the Bedroom, I cannot enter the room")
        reachedGoal = False
        waitingResponse = False

    if(humanLocation == 5 and canEnterRoom and not enteredRoom):
        goal_point.pose.position.x = 0.501
        goal_point.pose.position.y = -0.977
        goal_point.pose.position.z = 0.0
        goal_point.pose.orientation.x = 0.0
        goal_point.pose.orientation.y = 0.0
        goal_point.pose.orientation.z = 0.70711
        goal_point.pose.orientation.w = 0.70711
        goal_pub.publish(goal_point)
        reachedGoal = False
        enteredRoom = True

    if(humanLocation == 100 and not canEnterRoom):
        goal_point.pose.position.x = -1.026
        goal_point.pose.position.y = -1.032
        goal_point.pose.position.z = 0.0
        goal_point.pose.orientation.x = 0.0
        goal_point.pose.orientation.y = 0.0
        goal_point.pose.orientation.z = 0.003
        goal_point.pose.orientation.w = 1.000
        goal_pub.publish(goal_point)
        reachedGoal = False


def pirCallback(data):
    global humanLocation
    global canEnterRoom
    global sensitiveLocationReceived    

    if(not sensitiveLocationReceived):
        humanLocation = data.data
        setRobotGoal()
        if(humanLocation == 5):
            sensitiveLocationReceived = True

def decideEnterRoom():
    global ask
    global canEnterRoom
    
    #print("reached Goal: " + str(reachedGoal))
    #print("Human Location: " + str(humanLocation))
    #print("Can Enter room? " + str(canEnterRoom))

    if(reachedGoal and humanLocation == 5 and not canEnterRoom and not ask):
        ask = True
        voice = "Can I enter the room?"
        print(voice)
        voice_pub.publish(voice)

        time.sleep(4)

        voice = "Okay, I will wait here."
        print(voice)
        voice_pub.publish(voice)

        time.sleep(4)

        voice = "Can I enter the room now?"
        print(voice)
        voice_pub.publish(voice)

        time.sleep(4)

        canEnterRoom = True
    
    #if(reachedGoal and humanLocation == 5 and canEnterRoom):
        voice = "I am entering the room."
        print(voice)
        voice_pub.publish(voice)
        
        setRobotGoal()
        canEnterRoom = False
        

def navStatusCallback(data):
    global reachedGoal

    #currentPose = data.feedback.base_position.pose
    
    #linearDist = linearDistance(currentPose, goal_point.pose)

    #if(linearDist < 0.3 and humanLocation != -1):
    #    reachedGoal = True
    if(data.status_list[0].status == 3):
        reachedGoal = True
        
def voiceCallback(data):
    global canEnterRoom
    global humanLocation

    print("received: " + str(data.data))

    if(data.data == "yes"):
        print("I can enter the room")
        canEnterRoom = True
        
def nakednessCallback(data):
    global humanLocation
    global nakedCounter
    if(data.data == "half naked"):
        humanLocation = -1
        voice = "Oops, I am going back."
        print(voice)
        voice_pub.publish(voice)
        setRobotGoal()

def main(args):
    global goal_point
    try:
        rospy.init_node('robot_privacy_controller')

        nakedness_sub = rospy.Subscriber("/nakedness_detection", String, nakednessCallback)
        pir_sub = rospy.Subscriber("/PIR_MotionDetection", Int8, pirCallback)
        nav_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, navStatusCallback)
        voice_sub = rospy.Subscriber("/recognizer/output", String, voiceCallback)
        
        goal_point.header.frame_id = "map"        
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            decideEnterRoom()
            rate.sleep()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main(sys.argv)
