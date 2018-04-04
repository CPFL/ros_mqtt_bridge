#!/usr/bin/env python
# coding: utf-8

import json

import rospy
import paho.mqtt.client as mqtt

from attr_dict import AttrDict


class MqttToROS(object):

    DEFAULT_NODE_NAME = "mqtt_to_ros"

    def __init__(
            self, host, port, from_topic, to_topic, message_module_name, message_class_name,
            node_name=None, keepalive=60, queue_size=1, rospy_rate=10
    ):
        self.__from_topic = from_topic
        self.__to_topic = to_topic
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.connect(host, port=port, keepalive=keepalive)

        message_module = __import__(message_module_name)
        message_class = eval("message_module." + message_class_name)
        self.__ros_publisher = rospy.Publisher(to_topic, message_class, queue_size=queue_size)
        if node_name is None:
            rospy.init_node(MqttToROS.DEFAULT_NODE_NAME, anonymous=True)
        else:
            rospy.init_node(node_name)
        self.__rospy_rate = rospy.Rate(rospy_rate)

    def __del__(self):
        self.__client.disconnect()

    def __on_connect(self, _client, _userdata, _flags, response_code):
        if response_code == 0:
            self.__client.subscribe(self.__from_topic)
        else:
            print('connect status {0}'.format(response_code))

    def __on_message(self, _client, _user_data, message_data):
        message_dict = json.loads(message_data.payload.decode("utf-8"))
        message_attrdict = AttrDict.set_recursively(message_dict)
        self.__ros_publisher.publish(**message_attrdict)
        self.__rospy_rate.sleep()

    def start(self, dt=1.0):
        self.__client.loop_start()
        rospy.spin()


from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="mqtt broker host")
parser.add_argument("-P", "--port", type=int, default=1883, help="mqtt broker port")
parser.add_argument("-FT", "--from_topic", type=str, required=True, help="from topic")
parser.add_argument("-TT", "--to_topic", type=str, required=True, help="to topic")
parser.add_argument("-MMN", "--message_module_name", type=str, required=True, help="message module name")
parser.add_argument("-MCN", "--message_class_name", type=str, required=True, help="message class name")
args = parser.parse_args()


if __name__ == '__main__':
    mqtt_to_ros = MqttToROS(
        args.host, args.port, args.from_topic, args.to_topic, args.message_module_name, args.message_class_name)
    mqtt_to_ros.start()
