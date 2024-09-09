#!/usr/bin/env python3

import ast
import rospy
import subprocess
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sewer_msgs.msg import Log
import dynamic_reconfigure.client

LOGS = 7
VERBOSE = 7

class Teleop:
    """
    Class responsible for interpreting joystick commands
    """
    def __init__(self, use_dynamic_params=True):

        # Robot parameters
        self.use_dynamic_params = use_dynamic_params
        self.control_freq = 20
        self.logging = rospy.Publisher("sewer_logging", Log, queue_size=10)
        if self.use_dynamic_params:
            for tries in range(0, 3):
                try:
                    # ROS Dynamic Reconfigure
                    self.max_vel = 0.0
                    self.max_rot = 0.0
                    self.curr_max_vel = 1.0
                    self.curr_max_rot = 1.0
                    self.controller_list = ["Joystick", "Autonomous", "Interface"]
                    # self.log("Connecting to Motors Dynamic Parameter server", 7)
                    self.dynamic_client = dynamic_reconfigure.client.Client("sewer_motors", timeout=10, config_callback=self.update_params)
                    break
                except Exception:
                    if tries == 2:
                        # self.log("No motor dynamic parameter server detected.", 5, alert="warn")
                        self.max_vel = 1.0
                        self.max_rot = 1.0
                        self.curr_max_vel = 1.0
                        self.curr_max_rot = 1.0
                        self.dynamic_client = None
                    else:
                        # self.log("Waiting for motor dynamic parameter server.", 5, alert="warn")
                        rospy.sleep(2)
        else:
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

        # self.log("Node is initialized", 5)

    #########################
    #  AUXILIARY FUNCTIONS  #
    #########################
    # def log(self, msg, msg_level, log_level=-1, alert="info"):
    #     """
    #     Log function that publish in screen and in topic
    #     :param msg: Message to be published
    #     :param msg_level: Message level (1-10, where 1 is most important)
    #     :param log_level: Message level for logging (1-10, optional, -1 uses the same as msg_level)
    #     :param alert: Alert level of message - "info", "warn" or "error"
    #     :return:
    #     """
    #     if VERBOSE >= msg_level:
    #         if alert == "info":
    #             rospy.loginfo("{}: {}".format(rospy.get_name(), msg))
    #         elif alert == "warn":
    #             rospy.logwarn("{}: {}".format(rospy.get_name(), msg))
    #         elif alert == "error":
    #             rospy.logerr("{}: {}".format(rospy.get_name(), msg))
    #     if LOGS >= (log_level if log_level != -1 else msg_level):
    #         self.logging.publish(rospy.Time.now().to_sec(), rospy.get_name(), msg)

    ###############
    #  CALLBACKS  #
    ###############
    def update_params(self, config):
        """ Callback to an update in the dynamic parameters of the platform """
        # self.log("Parameters have been updated", 2)
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
            # self.log("{}: Incorrect joy message type".format(rospy.get_name()), 2, alert="error")
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
                # self.log("Node shutting down", 3, alert="warn")
                break


if __name__ == "__main__":
    rospy.init_node("sewer_teleop")

    use_dynamic_params = rospy.get_param("~use_dynamic_params", True)  # Get the flag from a ROS parameter (default: True)
    
    t = Teleop(use_dynamic_params=use_dynamic_params)
    t.start()
    rospy.loginfo("{}: Shutting down.".format(rospy.get_name()))