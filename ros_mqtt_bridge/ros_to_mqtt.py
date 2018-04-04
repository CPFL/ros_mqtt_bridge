#!/usr/bin/env python
# coding: utf-8

import yaml
import json

import rospy
import paho.mqtt.client as mqtt


class ROSToMqtt(object):

    DEFAULT_NODE_NAME = "ros_to_mqtt"

    def __init__(
            self, host, port, from_topic, to_topic, message_module_name, message_class_name,
            node_name=None, keepalive=3600, rospy_rate=10, qos=0
    ):
        self.__from_topic = from_topic
        self.__to_topic = to_topic
        self.__qos = qos

        message_module = __import__(message_module_name)
        self.__message_class = eval("message_module." + message_class_name)

        if node_name is None:
            rospy.init_node(ROSToMqtt.DEFAULT_NODE_NAME, anonymous=True)
        else:
            rospy.init_node(node_name)
        self.__rospy_rate = rospy.Rate(rospy_rate)

        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.connect(host, port=port, keepalive=keepalive)

    def start(self):
        while not rospy.is_shutdown():
            try:
                message_yaml = str(rospy.wait_for_message(self.__from_topic, self.__message_class))
                payload = json.dumps(yaml.load(message_yaml))
                self.__client.publish(self.__to_topic, payload=payload, qos=self.__qos)
            except rospy.ROSException:
                pass
            except rospy.ROSInterruptException:
                break

        self.__rospy_rate.sleep()


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
    ros_to_mqtt = ROSToMqtt(args.host, args.port, args.from_topic, args.to_topic, args.message_module_name, args.message_class_name)
    ros_to_mqtt.start()
