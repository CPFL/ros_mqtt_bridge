#!/usr/bin/env python
# coding: utf-8

import yaml
import json

import rospy
import paho.mqtt.client as mqtt

from ros_mqtt_bridge.args_setters import ArgsSetters


class ROSToMQTT(ArgsSetters):

    def __init__(self, from_topic, to_topic, message_type):
        super(ROSToMQTT, self).__init__(message_type)

        self.__mqtt_client = None

        self.args["mqtt"]["publish"]["topic"] = to_topic
        self.args["ros"]["wait_for_message"]["topic"] = from_topic
        self.args["ros"]["wait_for_message"]["topic_type"] = self.args["ros"]["data_class"]

    def connect_ros(self):
        if "name" not in self.args["ros"]["init_node"]:
            self.args["ros"]["init_node"]["name"] = "ros_mqtt_bridge"
            self.args["ros"]["init_node"]["anonymous"] = True
        rospy.init_node(**self.args["ros"]["init_node"])

    def connect_mqtt(self):
        self.__mqtt_client = mqtt.Client(**self.args["mqtt"]["client"])
        if self.args["mqtt"]["tls"] is not None:
            self.set_mqtt_tls()
        self.__mqtt_client.connect(**self.args["mqtt"]["connect"])

    def set_mqtt_tls(self):
        self.__mqtt_client.tls_set(**self.args["mqtt"]["tls"])
        self.__mqtt_client.tls_insecure_set(True)

    def start(self):
        self.connect_mqtt()
        self.connect_ros()
        self.__rospy_rate = rospy.Rate(**self.args["ros"]["rate"])
        while not rospy.is_shutdown():
            try:
                message_yaml = str(rospy.wait_for_message(**self.args["ros"]["wait_for_message"]))
                self.args["mqtt"]["publish"]["payload"] = json.dumps(yaml.load(message_yaml))
                self.__mqtt_client.publish(**self.args["mqtt"]["publish"])
                self.__rospy_rate.sleep()
            except rospy.ROSException:
                pass
            except rospy.ROSInterruptException:
                break
