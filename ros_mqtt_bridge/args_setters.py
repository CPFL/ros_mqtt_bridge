#!/usr/bin/env python
# coding: utf-8

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
        self.args["ros"]["publisher"]["name"] = name
        self.args["ros"]["publisher"]["data_class"] = data_class
        self.args["ros"]["publisher"]["subscriber_listener"] = subscriber_listener
        self.args["ros"]["publisher"]["tcp_nodelay"] = tcp_nodelay
        self.args["ros"]["publisher"]["latch"] = latch
        self.args["ros"]["publisher"]["headers"] = headers
        self.args["ros"]["publisher"]["queue_size"] = queue_size

    def set_ros_init_node_args(
            self, name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False,
            disable_signals=False, xmlrpc_port=0, tcpros_port=0):
        self.args["ros"]["init_node"]["name"] = name
        self.args["ros"]["init_node"]["argv"] = argv
        self.args["ros"]["init_node"]["anonymous"] = anonymous
        self.args["ros"]["init_node"]["log_level"] = log_level
        self.args["ros"]["init_node"]["disable_rostime"] = disable_rostime
        self.args["ros"]["init_node"]["disable_rosout"] = disable_rosout
        self.args["ros"]["init_node"]["disable_signals"] = disable_signals
        self.args["ros"]["init_node"]["xmlrpc_port"] = xmlrpc_port
        self.args["ros"]["init_node"]["tcpros_port"] = tcpros_port

    def set_ros_wait_for_message(self, topic, topic_type, timeout=None):
        self.args["ros"]["wait_for_message"]["topic"] = topic
        self.args["ros"]["wait_for_message"]["topic_type"] = topic_type
        self.args["ros"]["wait_for_message"]["timeout"] = timeout

    def set_ros_rate(self, hz, reset=False):
        self.args["ros"]["rate"]["hz"] = hz
        self.args["ros"]["rate"]["reset"] = reset

    def set_mqtt_client_args(
            self, client_id="", clean_session=True, userdata=None, protocol=MQTTv311, transport="tcp"):
        self.args["mqtt"]["client"]["client_id"] = client_id
        self.args["mqtt"]["client"]["clean_session"] = clean_session
        self.args["mqtt"]["client"]["userdata"] = userdata
        self.args["mqtt"]["client"]["protocol"] = protocol
        self.args["mqtt"]["client"]["transport"] = transport

    def set_mqtt_connect_args(self, host, port=1883, keepalive=60, bind_address=""):
        self.args["mqtt"]["connect"]["host"] = host
        self.args["mqtt"]["connect"]["port"] = port
        self.args["mqtt"]["connect"]["keepalive"] = keepalive
        self.args["mqtt"]["connect"]["bind_address"] = bind_address

    def set_mqtt_publish_args(self, topic, payload=None, qos=0, retain=False):
        self.args["mqtt"]["publish"]["topic"] = topic
        self.args["mqtt"]["publish"]["payload"] = payload
        self.args["mqtt"]["publish"]["qos"] = qos
        self.args["mqtt"]["publish"]["retain"] = retain

    def set_mqtt_subscribe_args(self, topic, qos=0):
        self.args["mqtt"]["subscribe"]["topic"] = topic
        self.args["mqtt"]["subscribe"]["qos"] = qos

    def set_mqtt_tls_args(
            self, ca_certs=None, certfile=None, keyfile=None, cert_reqs=None, tls_version=PROTOCOL_TLSv1_2,
            ciphers=None):
        self.args["mqtt"]["tls"] = {}
        self.args["mqtt"]["tls"]["ca_certs"] = ca_certs
        self.args["mqtt"]["tls"]["certfile"] = certfile
        self.args["mqtt"]["tls"]["keyfile"] = keyfile
        self.args["mqtt"]["tls"]["cert_reqs"] = cert_reqs
        self.args["mqtt"]["tls"]["tls_version"] = tls_version
        self.args["mqtt"]["tls"]["ciphers"] = ciphers

    def get_data_class_from_message_type(self, message_type):
        splitted_message_type = message_type.split("/")
        message_module_name = splitted_message_type[0] + ".msg"
        message_class_name = "msg." + splitted_message_type[1]
        message_module = __import__(message_module_name)
        return eval("message_module." + message_class_name)
