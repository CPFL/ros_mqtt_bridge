#!/usr/bin/env python
# coding: utf-8

import json

import rospy
import paho.mqtt.client as mqtt

from ros_mqtt_bridge.args_setters import ArgsSetters
from ros_mqtt_bridge.attr_dict import AttrDict


class MQTTToROS(ArgsSetters):

    def __init__(self, from_topic, to_topic, message_type):
        super(MQTTToROS, self).__init__(message_type)

        self.__mqtt_client = None
        self.__ros_publisher = None

        self.args["mqtt"]["subscribe"]["topic"] = from_topic
        self.args["ros"]["publisher"]["name"] = to_topic
        self.args["ros"]["publisher"]["data_class"] = self.args["ros"]["data_class"]

    def __on_connect(self, _client, _userdata, _flags, response_code):
        if response_code == 0:
            self.__mqtt_client.subscribe(**self.args["mqtt"]["subscribe"])
        else:
            print('connect status {0}'.format(response_code))

    def __on_message(self, _client, _user_data, message_data):
        message_dict = json.loads(message_data.payload.decode("utf-8"))
        message_attrdict = AttrDict.set_recursively(message_dict)
        self.__ros_publisher.publish(**message_attrdict)

    def connect_ros(self):
        if "name" not in self.args["ros"]["init_node"]:
            self.args["ros"]["init_node"]["name"] = "ros_mqtt_bridge"
            self.args["ros"]["init_node"]["anonymous"] = True
        self.__ros_publisher = rospy.Publisher(**self.args["ros"]["publisher"])
        rospy.init_node(**self.args["ros"]["init_node"])

    def connect_mqtt(self):
        self.__mqtt_client = mqtt.Client(**self.args["mqtt"]["client"])
        if self.args["mqtt"]["tls"] is not None:
            self.set_mqtt_tls()
        self.__mqtt_client.on_connect = self.__on_connect
        self.__mqtt_client.on_message = self.__on_message
        self.__mqtt_client.connect(**self.args["mqtt"]["connect"])

    def set_mqtt_tls(self):
        self.__mqtt_client.tls_set(**self.args["mqtt"]["tls"])
        self.__mqtt_client.tls_insecure_set(True)

    def start(self):
        self.connect_ros()
        self.connect_mqtt()
        self.__mqtt_client.loop_start()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
