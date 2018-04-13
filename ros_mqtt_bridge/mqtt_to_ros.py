#!/usr/bin/env python
# coding: utf-8

import json

import rospy
import paho.mqtt.client as mqtt

from ros_mqtt_bridge.attr_dict import AttrDict


class MQTTToROS(object):

    DEFAULT_NODE_NAME = "mqtt_to_ros"

    def __init__(
            self, host, port, from_topic, to_topic, message_module_name, message_class_name,
            node_name=None, keepalive=60, queue_size=1, rospy_rate=10
    ):
        self.__from_topic = from_topic
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.connect(host, port=port, keepalive=keepalive)

        message_module = __import__(message_module_name)
        message_class = eval("message_module." + message_class_name)
        self.__ros_publisher = rospy.Publisher(to_topic, message_class, queue_size=queue_size)
        if node_name is None:
            rospy.init_node(MQTTToROS.DEFAULT_NODE_NAME, anonymous=True)
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

    def start(self):
        self.__client.loop_start()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
