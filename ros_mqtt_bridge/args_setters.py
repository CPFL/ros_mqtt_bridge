#!/usr/bin/env python
# coding: utf-8

from copy import copy
from ssl import PROTOCOL_TLSv1_2
from paho.mqtt.client import MQTTv311

from ros_mqtt_bridge.const import MQTT_CLIENT


class ArgsSetters(object):

    DEFAULT_HOST = "localhost"

    def __init__(self, message_type):
        self.args = {
            "ros": {
                "data_class": self.get_data_class_from_message_type(message_type),
                "init_node": {},
                "publisher": {
                    "queue_size": 1
                },
                "wait_for_message": {},
                "rate": {
                    "hz": 2
                }
            },
            "mqtt": {
                "client": {},
                "connect": {
                    "host": ArgsSetters.DEFAULT_HOST,
                },
                "publish": {},
                "subscribe": {},
                "tls": None
            }
        }

    def set_ros_publisher_args(
            self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None,
            queue_size=None):
        self.args["ros"]["publisher"] = copy(locals())
        self.args["ros"]["publisher"].pop("self")

    def set_ros_init_node_args(
            self, name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False,
            disable_signals=False, xmlrpc_port=0, tcpros_port=0):
        self.args["ros"]["init_node"] = copy(locals())
        self.args["ros"]["init_node"].pop("self")

    def set_ros_wait_for_message_args(self, topic, topic_type, timeout=None):
        self.args["ros"]["wait_for_message"] = copy(locals())
        self.args["ros"]["wait_for_message"].pop("self")

    def set_ros_rate_args(self, hz, reset=False):
        self.args["ros"]["rate"] = copy(locals())
        self.args["ros"]["rate"].pop("self")

    def set_mqtt_client_args(
            self, client_id="", clean_session=True, userdata=None, protocol=MQTTv311, transport="tcp"):
        self.args["mqtt"]["client"] = copy(locals())
        self.args["mqtt"]["client"].pop("self")

    def set_mqtt_connect_args(self, host, port=1883, keepalive=60, bind_address=""):
        self.args["mqtt"]["connect"] = copy(locals())
        self.args["mqtt"]["connect"].pop("self")

    def set_mqtt_publish_args(self, topic, payload=None, qos=0, retain=False):
        self.args["mqtt"]["publish"] = copy(locals())
        self.args["mqtt"]["publish"].pop("self")

    def set_mqtt_subscribe_args(self, topic, qos=0):
        self.args["mqtt"]["subscribe"] = copy(locals())
        self.args["mqtt"]["subscribe"].pop("self")

    def set_mqtt_tls_args(
            self, ca_certs=None, certfile=None, keyfile=None, cert_reqs=None, tls_version=PROTOCOL_TLSv1_2,
            ciphers=None):
        self.args["mqtt"]["tls"] = copy(locals())
        self.args["mqtt"]["tls"].pop("self")

    @staticmethod
    def get_data_class_from_message_type(message_type):
        splitted_message_type = message_type.split("/")
        message_module_name = splitted_message_type[0] + ".msg"
        message_class_name = "msg." + splitted_message_type[1]
        message_module = __import__(message_module_name)
        return eval("message_module." + message_class_name)


class RospyArgsSetter(object):

    DEFAULT_HOST = "localhost"

    def __init__(self, message_type):
        self.args["ros"] = {
            "data_class": self.get_data_class_from_message_type(message_type),
            "init_node": {},
            "publisher": {
                "queue_size": 1
            },
            "wait_for_message": {},
            "rate": {
                "hz": 2
            }
        }

    def set_ros_publisher_args(
            self, name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None,
            queue_size=None):
        self.args["ros"]["publisher"] = copy(locals())
        self.args["ros"]["publisher"].pop("self")

    def set_ros_init_node_args(
            self, name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False,
            disable_signals=False, xmlrpc_port=0, tcpros_port=0):
        self.args["ros"]["init_node"] = copy(locals())
        self.args["ros"]["init_node"].pop("self")

    def set_ros_wait_for_message_args(self, topic, topic_type, timeout=None):
        self.args["ros"]["wait_for_message"] = copy(locals())
        self.args["ros"]["wait_for_message"].pop("self")

    def set_ros_rate_args(self, hz, reset=False):
        self.args["ros"]["rate"] = copy(locals())
        self.args["ros"]["rate"].pop("self")

    @staticmethod
    def get_data_class_from_message_type(message_type):
        splitted_message_type = message_type.split("/")
        message_module_name = splitted_message_type[0] + ".msg"
        message_class_name = "msg." + splitted_message_type[1]
        message_module = __import__(message_module_name)
        return eval("message_module." + message_class_name)


class AWSIoTArgsSetter(object):

    CONST = MQTT_CLIENT.BASE_CLIENTS.AWS_IOT_SDK

    def set_aws_iot_AWSIoTMQTTClient(
            self, clientID, protocolType=CONST.DEFAULT_PROTOCOL,
            useWebsocket=False, cleanSession=True):
        self.args["mqtt"]["aws_iot_mqtt_client"] = copy(locals())
        self.args["mqtt"]["aws_iot_mqtt_client"].pop("self")

    def set_aws_iot_configureEndpoint(self, hostName, portNumber):
        self.args["mqtt"]["configure_endpoint"] = copy(locals())
        self.args["mqtt"]["configure_endpoint"].pop("self")

    def set_aws_iot_configureCredentials(self, CAFilePath, KeyPath="", CertificatePath=""):
        self.args["mqtt"]["configure_credentials"] = copy(locals())
        self.args["mqtt"]["configure_credentials"].pop("self")

    def set_aws_iot_configureIAMCredentials(self, AWSAccessKeyID, AWSSecretAccessKey, AWSSessionToken=""):
        self.args["mqtt"]["configure_iam_credentials"] = copy(locals())
        self.args["mqtt"]["configure_iam_credentials"].pop("self")

    def set_aws_iot_configureLastWill(self, topic, payload, QoS, retain=False):
        self.args["mqtt"]["configure_last_will"] = copy(locals())
        self.args["mqtt"]["configure_last_will"].pop("self")

    def set_aws_iot_connect(self, keepAliveIntervalSecond=600):
        self.args["mqtt"]["connect"] = copy(locals())
        self.args["mqtt"]["connect"].pop("self")

