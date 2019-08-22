#!/usr/bin/env python

# Receive PIR data and do coarse human localization

import roslib;
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String,Int8
import serial

PIR_data = [0,0,0,0,0,0]

room_dict = {'2':"living room", '1': "dining table", '4':"kitchen", '3':"bathroom", '5':"bedroom" }

PIR_dict = {49:['4', "kitchen"], 50:['4',"kitchen"], 51:['3', "bathroom"],
            52:['2',"Living room"], 53:['5', "bedroom"], 55:['1', "dining table"], 54:['2',"living room"]}

def serialInit():
    # Open Serial Port and configure the baud rate
    port = "/dev/ttyUSB0"
    baudrate = 57600
    #Opening Serial port
    SerialPortHandler = serial.Serial(port, baudrate)
    print "Serial Port Connected"
    return SerialPortHandler

def talker(SerialPortHandler):
    pub = rospy.Publisher('/PIR_MotionDetection', Int8, queue_size=100)
    rospy.init_node('PIR_Receiver',anonymous=True)
    rospy.sleep(0.1)
    new_data = True
    id_read = False
    current_position = 0
    print "Wait to receive IR sensor Data ..."
    while not rospy.is_shutdown():
        #Read the serial Data
        sensorData = SerialPortHandler.read()
        # Receive One-by-One: (ID|on/off|Return Key|New Line)
	    #Get ASCII code of the Value
        sensorData = ord(sensorData[0])
        if new_data:
            sensorID = sensorData;
            new_data = False
            id_read = True
        else:
            if id_read: # sensor status read
                id_read = False
                if (sensorData - 48 == 1):
                    PIR = PIR_dict.get(sensorID)
                    if PIR:
                        if PIR[0]!= current_position:
                            current_position = PIR[0]
                            rospy.loginfo("The person is located at " + room_dict[current_position] + ".")
                            pub.publish(int(current_position))
                    else:
                        print sensorID
        
        rospy.sleep(0.01)
        if sensorData==10:
            new_data = True

if __name__ == '__main__':
    try:
        SerialPortHandler = serialInit()
        talker(SerialPortHandler)

    except rospy.ROSInterruptException:
        SerialPortHandler.close()
        pass
