#!/usr/bin/env python
# coding: utf-8

import json

import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from ros_mqtt_bridge.args_setters import RospyArgsSetter, AWSIoTArgsSetter
from ros_mqtt_bridge.attr_dict import AttrDict


class AWSIoTToROS(RospyArgsSetter, AWSIoTArgsSetter):

    def __init__(self, from_topic, to_topic, message_type, qos=0):
        self.args = {"ros": {}, "mqtt": {}}
        super(AWSIoTToROS, self).__init__(message_type)

        self.__mqtt_client = None
        self.__ros_publisher = None

        self.args["mqtt"]["subscribe"] = {
            "topic": from_topic,
            "QoS": qos,
            "callback": self.__on_message
        }
        self.args["ros"]["publisher"]["name"] = to_topic
        self.args["ros"]["publisher"]["data_class"] = self.args["ros"]["data_class"]

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
        self.__mqtt_client = AWSIoTMQTTClient(**self.args["mqtt"]["aws_iot_mqtt_client"])
        self.__mqtt_client.configureEndpoint(**self.args["mqtt"]["configure_endpoint"])

        if "configure_credentials" in self.args["mqtt"].keys():
            self.__mqtt_client.configureCredentials(**self.args["mqtt"]["configure_credentials"])

        if "configure_last_will" in self.args["mqtt"].keys():
            self.__mqtt_client.configureLastWill(**self.args["mqtt"]["configure_last_will"])

        self.__mqtt_client.connect(**self.args["mqtt"]["connect"])
        self.__mqtt_client.subscribe(**self.args["mqtt"]["subscribe"])

    def start(self):
        self.connect_ros()
        self.connect_mqtt()
        self.__mqtt_client.loop_start()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
