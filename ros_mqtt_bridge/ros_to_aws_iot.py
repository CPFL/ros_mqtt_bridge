#!/usr/bin/env python
# coding: utf-8

import yaml
import json

import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from ros_mqtt_bridge.args_setters import RospyArgsSetter, AWSIoTArgsSetter


class ROSToAWSIoT(RospyArgsSetter, AWSIoTArgsSetter):

    def __init__(self, from_topic, to_topic, message_type, qos=0):
        self.args = {"ros": {}, "mqtt": {}}
        super(ROSToAWSIoT, self).__init__(message_type)

        self.__mqtt_client = None

        self.args["mqtt"]["publish"] = {
            "topic": to_topic,
            "QoS": qos,
            "payload": None
        }
        self.args["ros"]["wait_for_message"]["topic"] = from_topic
        self.args["ros"]["wait_for_message"]["topic_type"] = self.args["ros"]["data_class"]

    def connect_ros(self):
        if "name" not in self.args["ros"]["init_node"]:
            self.args["ros"]["init_node"]["name"] = "ros_mqtt_bridge"
            self.args["ros"]["init_node"]["anonymous"] = True
        rospy.init_node(**self.args["ros"]["init_node"])

    def connect_mqtt(self):
        # print("create client", self.args["mqtt"]["aws_iot_mqtt_client"])
        self.__mqtt_client = AWSIoTMQTTClient(**self.args["mqtt"]["aws_iot_mqtt_client"])
        # print("configure endpoint", self.args["mqtt"]["configure_endpoint"])
        self.__mqtt_client.configureEndpoint(**self.args["mqtt"]["configure_endpoint"])

        if "configure_credentials" in self.args["mqtt"].keys():
            # print("configure credentials", self.args["mqtt"]["configure_credentials"])
            self.__mqtt_client.configureCredentials(**self.args["mqtt"]["configure_credentials"])
        # elif "configure_iam_credentials" in self.args["mqtt"].keys():
        #     print("configure iam credentials", self.args["mqtt"]["configure_iam_credentials"])
        #     self.__mqtt_client.configureIAMCredentials(**self.args["mqtt"]["configure_iam_credentials"])
        if "configure_last_will" in self.args["mqtt"].keys():
            # print("configure last will", self.args["mqtt"]["configure_last_will"])
            self.__mqtt_client.configureLastWill(**self.args["mqtt"]["configure_last_will"])

        # print("connect", self.args["mqtt"]["connect"])
        self.__mqtt_client.connect(**self.args["mqtt"]["connect"])

    def start(self):
        self.connect_mqtt()
        self.connect_ros()
        self.__rospy_rate = rospy.Rate(**self.args["ros"]["rate"])
        while not rospy.is_shutdown():
            try:
                message_yaml = str(rospy.wait_for_message(**self.args["ros"]["wait_for_message"]))
                self.args["mqtt"]["publish"]["payload"] = json.dumps(yaml.load(message_yaml))
                # print(self.args["mqtt"]["publish"])
                self.__mqtt_client.publish(**self.args["mqtt"]["publish"])
                self.__rospy_rate.sleep()
            except rospy.ROSException:
                pass
            except rospy.ROSInterruptException:
                break
