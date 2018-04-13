# ROS-MQTT-BRIDGE [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) example

## Requirements

- mosquitto(localhost:1883)

## Run

launch bridges
```
$ python std_msgs_string.py
```

launch subscribers
```
$ rostopic echo /ros/test/std_msgs_string
```

```
$ mosquitto_sub -d -t /mqtt/test/std_msgs_string -v
```

publish
```
$ rostopic pub -1 /ros/test/std_msgs_string std_msgs/String "data: 'test text from ros'"
```

```
$ mosquitto_pub -d -t /mqtt/test/std_msgs_string -m '{"data": "test text from mqtt"}'
```
