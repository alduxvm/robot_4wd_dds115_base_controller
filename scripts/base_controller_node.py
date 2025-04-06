#!/usr/bin/env python3
import os
import sys
import rospy
import math
import time
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float32MultiArray

# Adjust the path so Python can locate the ddsm115 module.
this_dir = os.path.dirname(os.path.abspath(__file__))
ddsm115_path = os.path.abspath(os.path.join(this_dir, '..', 'ddsm115'))
if ddsm115_path not in sys.path:
    sys.path.insert(0, ddsm115_path)

import ddsm115

class BaseController:
    def __init__(self):
        rospy.init_node("base_controller", anonymous=True)

        # Load parameters
        self.wheel_names = rospy.get_param("~wheel_names", 
            ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"])
        self.wheel_ids = rospy.get_param("~wheel_ids", [1, 2, 3, 4])
        self.wheel_directions = rospy.get_param("~wheel_directions", 
            ["forward", "backward", "forward", "backward"])
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.1)      # in meters
        self.wheel_track = rospy.get_param("~wheel_track", 0.5)        # meters (distance between left & right wheels)
        self.wheel_base = rospy.get_param("~wheel_base", 0.6)          # meters (distance between front & rear wheels)
        self.feedback_rate = rospy.get_param("~feedback_rate", 100)    # Hz

        # New parameters for slight turning behavior.
        self.slight_turn_threshold = rospy.get_param("~slight_turn_threshold", 0.1)  # rad/s threshold for slight turn
        self.inside_reduction_factor = rospy.get_param("~inside_reduction_factor", 0.3)  # fraction to reduce inside wheel speed

        # Instantiate the motor control object from ddsm115.
        self.motor_control = ddsm115.MotorControl()

        # Set all motors to drive mode 2.
        for motor_id in self.wheel_ids:
            self.motor_control.set_drive_mode(motor_id, 2)

        # Initialize odometry state.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # Publishers for odometry, motor RPMs, and motor currents, and TF broadcaster.
        self.odom_pub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.motor_rpm_pub = rospy.Publisher("motor_rpms", Float32MultiArray, queue_size=10)
        self.motor_current_pub = rospy.Publisher("motor_currents", Float32MultiArray, queue_size=10)

        # Subscriber for /cmd_vel commands.
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        # Timer for the feedback loop.
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.feedback_rate), self.update)

    def cmd_vel_callback(self, msg):
        """
        Converts incoming /cmd_vel commands into motor RPM commands.

        For slight turns (angular velocity below slight_turn_threshold), instead of applying
        full differential drive offsets, the inside wheels are commanded at a reduced speed
        relative to the current forward speed.

        For larger rotations, the standard differential drive kinematics are used:
            left_vel  = v - (w * wheel_track/2)
            right_vel = v + (w * wheel_track/2)
        
        Linear velocities (m/s) are then converted to RPM.
        """
        v = msg.linear.x
        w = msg.angular.z

        conversion_factor = 60.0 / (2 * math.pi * self.wheel_radius)

        # Check if this is a slight turn.
        if abs(w) < self.slight_turn_threshold:
            # For a slight turn, reduce the speed of the inside wheels.
            if w > 0:
                # Turning left: left wheels are inside.
                left_vel = v * (1 - self.inside_reduction_factor)
                right_vel = v
            elif w < 0:
                # Turning right: right wheels are inside.
                left_vel = v
                right_vel = v * (1 - self.inside_reduction_factor)
            else:
                left_vel = v
                right_vel = v
        else:
            # Standard differential drive for larger rotations.
            left_vel = v - (w * self.wheel_track / 2.0)
            right_vel = v + (w * self.wheel_track / 2.0)

        # Convert linear velocities to RPM.
        left_rpm = left_vel * conversion_factor
        right_rpm = right_vel * conversion_factor

        # Command each motor with the appropriate RPM.
        for i, motor_id in enumerate(self.wheel_ids):
            rpm = left_rpm if i in [0, 2] else right_rpm

            # Adjust for wheel direction.
            if self.wheel_directions[i] == "backward":
                rpm = -rpm

            try:
                self.motor_control.send_rpm(motor_id, rpm)
            except Exception as e:
                rospy.logwarn("Error sending rpm to motor {}: {}".format(motor_id, e))

    def update(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        wheel_velocities = []
        motor_rpms = []
        motor_currents = []

        # Read feedback for each motor.
        for i, motor_id in enumerate(self.wheel_ids):
            try:
                feedback = self.motor_control.get_motor_feedback(motor_id)
                rpm = feedback[0]
                current = feedback[1]
            except Exception as e:
                rospy.logwarn("Error reading feedback for motor {}: {}".format(motor_id, e))
                rpm, current = 0, 0

            # Adjust RPM based on wheel direction.
            if self.wheel_directions[i] == "backward":
                rpm = -rpm

            motor_rpms.append(rpm)
            motor_currents.append(current)
            # Convert rpm to linear velocity (m/s): v = rpm * (2*pi*radius) / 60.
            v_conv = rpm * (2 * math.pi * self.wheel_radius) / 60.0
            wheel_velocities.append(v_conv)

        # For odometry, assume wheels 0 and 2 are on the left, and wheels 1 and 3 are on the right.
        v_left = (wheel_velocities[0] + wheel_velocities[2]) / 2.0
        v_right = (wheel_velocities[1] + wheel_velocities[3]) / 2.0

        linear_velocity = (v_left + v_right) / 2.0
        angular_velocity = (v_right - v_left) / self.wheel_track

        # Update the robot's pose (simple Euler integration).
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt

        # Publish the odometry message.
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*q)
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            q,
            current_time,
            "base_link",
            "odom"
        )

        # Publish motor RPM and current topics.
        rpm_msg = Float32MultiArray()
        rpm_msg.data = motor_rpms
        self.motor_rpm_pub.publish(rpm_msg)

        current_msg = Float32MultiArray()
        current_msg.data = motor_currents
        self.motor_current_pub.publish(current_msg)

if __name__ == '__main__':
    try:
        controller = BaseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
