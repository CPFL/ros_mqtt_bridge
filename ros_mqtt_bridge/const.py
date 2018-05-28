#!/usr/bin/env python
# coding: utf-8

import pkgutil
import importlib
from collections import namedtuple


def get_namedtuple_from_dict(typename, _dict):
    values = []
    for key, value in _dict.items():
        if isinstance(value, dict):
            values.append(get_namedtuple_from_dict(key, value))
        else:
            values.append(value)
    return namedtuple(typename, list(_dict.keys()))(*values)

base_clients = {}

if pkgutil.find_loader("paho.mqtt.client") is not None:
    base_clients["PAHO_MQTT"] = {
        "MODULE_NAME": "paho.mqtt.client",
        "DEFAULT_PROTOCOL": importlib.import_module("paho.mqtt.client").MQTTv311,
    }

if pkgutil.find_loader("AWSIoTPythonSDK.MQTTLib") is not None:
    base_clients["AWS_IOT_SDK"] = {
        "MODULE_NAME": "AWSIoTPythonSDK.MQTTLib",
        "DEFAULT_PROTOCOL": importlib.import_module("AWSIoTPythonSDK.MQTTLib").MQTTv3_1_1
    }

if pkgutil.find_loader("boto3") is not None:
    base_clients["AWS_IOT_BOTO3"] = {
        "MODULE_NAME": "boto3",
        "CLIENT_TYPE": "iot-data",
    }

MQTT_CLIENT = get_namedtuple_from_dict("CONST", {
    "BASE_CLIENTS": base_clients
})
