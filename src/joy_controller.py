#!/usr/bin/env python3

import ast
import rospy
import subprocess
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import dynamic_reconfigure.client

LOGS = 7
VERBOSE = 7

class Teleop:
    """
    Class responsible for interpreting joystick commands
    """
    def __init__(self):

        # Robot parameters
        self.control_freq = 20

        self.max_vel = 1.0
        self.max_rot = 1.0
        self.curr_max_vel = 1.0
        self.curr_max_rot = 1.0
        self.dynamic_client = None
        self.controller_list = ["Joystick", "Autonomous", "Interface"]

        # Joystick mapping
        self.change_controller = False
        self.joy = rospy.get_param("sewer_teleop_joy/key_mapping")

        self.publish_vel = False

        ################
        #  Navigation  #
        ################
        teleop_topic = rospy.get_param("~cmd_vel_topic", "sewer_teleop/set_velocities")
        self.twist = Twist()
        self.last_joy = Joy()
        self.twist_pub = rospy.Publisher(teleop_topic, Twist, queue_size=10)

        self.change_max_speed = False
        rospy.Subscriber("joy", Joy, self.update_joy)

    ###############
    #  CALLBACKS  #
    ###############
    def update_params(self, config):
        """ Callback to an update in the dynamic parameters of the platform """
        self.max_vel = config.max_vel
        self.max_rot = config.max_rot
        self.controller = config.controller

    def update_joy(self, msg):
        # Define which mode the joystick is using
        if len(msg.axes) == 8:
            mode = "x"
        elif len(msg.axes) == 6:
            mode = "d"
        else:
            mode = "d"  # This is for the NPlay  remote
            return

        # Translate command to robot velocities - Added a "dead man switch"
        new_vel = Twist()

        if msg.buttons[self.joy[mode]["buttons"]["LB"]] :
            self.publish_vel = not self.publish_vel
        if msg.buttons[self.joy[mode]["buttons"]["Y"]]:
            self.change_max_speed = True
        if msg.buttons[self.joy[mode]["buttons"]["RB"]]:
            self.change_controller = True
        # Compute rotation velocity
        new_vel.linear.x = msg.axes[self.joy[mode]["axis"]["YLJoy"]] * self.curr_max_vel
        new_vel.linear.y = msg.axes[self.joy[mode]["axis"]["XLJoy"]] * self.curr_max_vel
        new_vel.angular.z = msg.axes[self.joy[mode]["axis"]["XRJoy"]] * self.curr_max_rot

        self.twist = new_vel

        # Save the current msg in order to detect rise/fall events of buttons
        self.last_joy = msg

        if self.publish_vel:
            self.twist_pub.publish(self.twist)

    def start(self):
        r = rospy.Rate(self.control_freq)
        while not rospy.is_shutdown():
            try:
                if (self.dynamic_client is not None) and self.change_controller:
                    idx = self.controller_list.index(self.controller)
                    new_controller = self.controller_list[idx+1] if (idx + 1) < len(self.controller_list) else self.controller_list[0]
                    self.dynamic_client.update_configuration({"controller": new_controller})
                    self.change_controller = False
                if self.change_max_speed:
                    self.curr_max_vel = min(2 * self.curr_max_vel, self.max_vel)
                    self.curr_max_rot = min(2 * self.curr_max_rot, self.max_rot)
                    self.change_max_speed = False                          
                # self.twist_pub.publish(self.twist)
                r.sleep()

            except (KeyboardInterrupt, rospy.exceptions.ROSInterruptException):
                break


if __name__ == "__main__":
    rospy.init_node("sewer_teleop")

    
    t = Teleop()
    t.start()

    rospy.loginfo("{}: Shutting down.".format(rospy.get_name()))