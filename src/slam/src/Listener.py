import rospy
import json
from std_msgs.msg import String

class Listener:
    def __init__(self , publisher_name, dataStruct, timeTick = 20):
        self.pubName = publisher_name
        self.dataStruct = dataStruct
        self.sub = rospy.Subscriber(self.pubName, String, lambda message: self.callback(message))

    def callback(self,message):
        objects = json.loads(message.data)
        if len(self.dataStruct) <= 100:
            self.dataStruct.append(objects)
