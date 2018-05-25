#!/usr/bin/env python
# coding: utf-8

from uuid import uuid4 as uid
from argparse import ArgumentParser
from ros_mqtt_bridge import ROSToAWSIoT

parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="mqtt broker host")
parser.add_argument("-P", "--port", type=int, default=8883, help="mqtt broker port")

parser.add_argument("-CAP", "--ca_file_path", type=str, default=None, help="./secrets/root-ca.crt")
parser.add_argument("-KP", "--key_file_path", type=str, default="", help="./secrets/private.key")
parser.add_argument("-CP", "--certificate_file_path", type=str, default="", help="./secrets/cert.pem")

parser.add_argument("-AKI", "--access_key_id", type=str, default=None, help="access key id")
parser.add_argument("-SAK", "--secret_access_key", type=str, default=None, help="secret access key")
parser.add_argument("-ST", "--session_token", type=str, default="", help="session token")

parser.add_argument("-FT", "--from_topic", type=str, required=True, help="from topic")
parser.add_argument("-TT", "--to_topic", type=str, required=True, help="to topic")
parser.add_argument("-MT", "--message_type", type=str, required=True, help="message type")
args = parser.parse_args()


if __name__ == '__main__':
    ros_to_aws_iot = ROSToAWSIoT(args.from_topic, args.to_topic, args.message_type)

    ros_to_aws_iot.set_aws_iot_AWSIoTMQTTClient(clientID=str(uid()))
    ros_to_aws_iot.set_aws_iot_configureEndpoint(args.host, args.port)

    if args.ca_file_path is not None:
        ros_to_aws_iot.set_aws_iot_configureCredentials(
            args.ca_file_path, args.key_file_path, args.certificate_file_path)
    if None not in [args.access_key_id, args.secret_access_key]:
        ros_to_aws_iot.set_aws_iot_configureIAMCredentials(
            args.access_key_id, args.secret_access_key, args.session_token)

    ros_to_aws_iot.start()
