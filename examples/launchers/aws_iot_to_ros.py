#!/usr/bin/env python
# coding: utf-8

from uuid import uuid4 as uid
from argparse import ArgumentParser
from ros_mqtt_bridge import AWSIoTToROS

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="mqtt broker host")
parser.add_argument("-P", "--port", type=int, default=8883, help="mqtt broker port")

parser.add_argument("-CAP", "--ca_file_path", type=str, default=None, help="./secrets/root-ca.crt")
parser.add_argument("-KP", "--key_file_path", type=str, default="", help="./secrets/private.key")
parser.add_argument("-CP", "--certificate_file_path", type=str, default="", help="./secrets/cert.pem")

parser.add_argument("-FT", "--from_topic", type=str, required=True, help="from topic")
parser.add_argument("-TT", "--to_topic", type=str, required=True, help="to topic")
parser.add_argument("-MT", "--message_type", type=str, required=True, help="message type")
args = parser.parse_args()


if __name__ == '__main__':
    aws_iot_to_ros = AWSIoTToROS(args.from_topic, args.to_topic, args.message_type)

    aws_iot_to_ros.set_aws_iot_AWSIoTMQTTClient(clientID=str(uid()))
    aws_iot_to_ros.set_aws_iot_configureEndpoint(args.host, args.port)

    aws_iot_to_ros.set_aws_iot_configureCredentials(
        args.ca_file_path, args.key_file_path, args.certificate_file_path)

    aws_iot_to_ros.set_aws_iot_connect()
    aws_iot_to_ros.start()
