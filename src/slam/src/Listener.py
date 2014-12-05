import rospy
import json
from std.msgs.msg import String

class Listener:
    def __init__(self,node_name,publisher_name, dataStruct, timeTick = 25):
        self.pubName = publisher_name
        self.dataStruct = dataStruct
        self.nodeName = node_name
        rospy.init_node(node_name)
        self.sub = rospy.Subscriber(self.pubName, String, lambda message: self.callback(message))
        rospy.spin()

    def callback(self,message):
        objects = json.loads(message)
        dataStruct.append(objects)
