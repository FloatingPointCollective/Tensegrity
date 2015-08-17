import paho.mqtt.client as mqtt
import json
import rospy
from std_msgs.msg import Float32MultiArray
rospy.init_node("tensegrity")

pub = rospy.Publisher("raw_data",Float32MultiArray)
client = mqtt.Client()
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("#")
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    try:
    	data = json.loads(msg.payload)
    	ros_msg = Float32MultiArray()
    	ros_msg.data = [data["acc_X"]/512.,data["acc_Y"]/512.,data["acc_Z"]/512.]
    	for i in xrange(12):
    		ros_msg.data.append(data["touch"][i]/3.)
    	pub.publish(ros_msg)
    except:
    	print "Couldn't parse JSON"
    
client.on_connect = on_connect
client.on_message = on_message
client.connect('127.0.0.1',8080,60)
client.loop_forever()
