#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ssl import PROTOCOL_TLSv1_2
from paho.mqtt.client import MQTTv311


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
        self.args["ros"]["publisher"] = deepcopy(locals())
        self.args["ros"]["publisher"].pop("self")

    def set_ros_init_node_args(
            self, name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False,
            disable_signals=False, xmlrpc_port=0, tcpros_port=0):
        self.args["ros"]["init_node"] = deepcopy(locals())
        self.args["ros"]["init_node"].pop("self")

    def set_ros_wait_for_message_args(self, topic, topic_type, timeout=None):
        self.args["ros"]["wait_for_message"] = deepcopy(locals())
        self.args["ros"]["wait_for_message"].pop("self")

    def set_ros_rate_args(self, hz, reset=False):
        self.args["ros"]["rate"] = deepcopy(locals())
        self.args["ros"]["rate"].pop("self")

    def set_mqtt_client_args(
            self, client_id="", clean_session=True, userdata=None, protocol=MQTTv311, transport="tcp"):
        self.args["mqtt"]["client"] = deepcopy(locals())
        self.args["mqtt"]["client"].pop("self")

    def set_mqtt_connect_args(self, host, port=1883, keepalive=60, bind_address=""):
        self.args["mqtt"]["connect"] = deepcopy(locals())
        self.args["mqtt"]["connect"].pop("self")

    def set_mqtt_publish_args(self, topic, payload=None, qos=0, retain=False):
        self.args["mqtt"]["publish"] = deepcopy(locals())
        self.args["mqtt"]["publish"].pop("self")

    def set_mqtt_subscribe_args(self, topic, qos=0):
        self.args["mqtt"]["subscribe"] = deepcopy(locals())
        self.args["mqtt"]["subscribe"].pop("self")

    def set_mqtt_tls_args(
            self, ca_certs=None, certfile=None, keyfile=None, cert_reqs=None, tls_version=PROTOCOL_TLSv1_2,
            ciphers=None):
        self.args["mqtt"]["tls"] = deepcopy(locals())
        self.args["mqtt"]["tls"].pop("self")

    @staticmethod
    def get_data_class_from_message_type(message_type):
        splitted_message_type = message_type.split("/")
        message_module_name = splitted_message_type[0] + ".msg"
        message_class_name = "msg." + splitted_message_type[1]
        message_module = __import__(message_module_name)
        return eval("message_module." + message_class_name)
