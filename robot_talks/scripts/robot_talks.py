#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

soundhandle = SoundClient()
voice = 'voice_kal_diphone'

def callback(msg):
    s = msg.data

    print 'Saying: %s' % s
    print 'Voice: %s' % voice
    
    soundhandle.say(s, voice)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robot_talks')
    rospy.Subscriber("/robot_voice", String, callback)

    soundhandle.say("Voice node started", voice)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
