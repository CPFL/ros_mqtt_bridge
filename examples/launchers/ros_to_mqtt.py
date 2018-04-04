#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from ros_mqtt_bridge import ROSToMQTT

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="mqtt broker host")
parser.add_argument("-P", "--port", type=int, default=1883, help="mqtt broker port")
parser.add_argument("-FT", "--from_topic", type=str, required=True, help="from topic")
parser.add_argument("-TT", "--to_topic", type=str, required=True, help="to topic")
parser.add_argument("-MMN", "--message_module_name", type=str, required=True, help="message module name")
parser.add_argument("-MCN", "--message_class_name", type=str, required=True, help="message class name")
args = parser.parse_args()


if __name__ == '__main__':
    ros_to_mqtt = ROSToMQTT(
        args.host, args.port, args.from_topic, args.to_topic, args.message_module_name, args.message_class_name)
    ros_to_mqtt.start()
