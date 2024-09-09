# idmind_teleop

This package will transform joystick/controller commands into velocity commands. 

## Requirements
- joy_node package: http://wiki.ros.org/joy
- Tested with Logitech Wireless Gamepad F710 and NPlay MB-8140
- udev rule that places joystick device in /dev/idmind_joystick is recommended

## Usage
- Launch idmind_teleop.launch to use default values and publish velocity commands
- Left Joystick controls X and Y velocities, Right Joystick controls angular velocity.
- Top left Button is a dead-man switch, 

## API
### Topics
- Subscribes:
    - /joy (sensor_msgs/Joy)
- Publishes:
    - idmind_logging (idmind_msgs/Log)
    - idmind_teleop/set_velocities (geometry_messages/Twist)

## Services
- Calls /idmind_navigation/toggle_joystick (if available)
- Calls /idmind_leds/toggle_display (if available)

## Parameters
- /ns/kinematics
- /ns/max_vel
- /ns/max_rot

where ns can be *bot*, *motors* or *teleop* (the first that exists).

##Contacts
Carlos Neves - cneves@idmind.pt
Henrique Póvoa - hpovoa@idmind.pt