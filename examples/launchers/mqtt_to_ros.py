#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from ros_mqtt_bridge import MQTTToROS

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="mqtt broker host")
parser.add_argument("-P", "--port", type=int, default=1883, help="mqtt broker port")
parser.add_argument("-FT", "--from_topic", type=str, required=True, help="from topic")
parser.add_argument("-TT", "--to_topic", type=str, required=True, help="to topic")
parser.add_argument("-MT", "--message_type", type=str, required=True, help="message type")
args = parser.parse_args()


if __name__ == '__main__':
    mqtt_to_ros = MQTTToROS(args.from_topic, args.to_topic, args.message_type)
    mqtt_to_ros.set_mqtt_connect_args(host=args.host, port=args.port)
    mqtt_to_ros.start()
