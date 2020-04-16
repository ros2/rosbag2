# Overriding topic QoS profiles

## Recording

When starting a recording workflow, you can pass a yaml file that contains QoS profile settings for a specific topic.
The yaml file should have following structure:

```yaml
<topic1_name>:
  <setting_1>: value
  <time_setting>:
    sec: 0
    nsec: 0
...
<topic2_name>:
  <setting_2>: value
...
```

See [`qos_profile.yaml`](../ros2bag/test/resources/qos_profile.yaml) for a complete example.

## Resources

* [About QoS settings](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)
* [ROS2 QoS policies design doc](https://design.ros2.org/articles/qos.html)
