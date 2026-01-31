#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import math


class ServoJointVelEncoder:
    """
    ROS node that converts joint velocities (rad/s) from MoveIt Servo
    into a formatted string for STM microcontroller communication.

    Output Format:
      V + sign(1) + magnitude(3) for each joint + F
      Example: V+120-045+000+999+012-300F (for 6 joints)
      
    Each joint velocity is normalized to a 0-999 range based on
    the maximum velocity limit for that specific joint.
    """

    def __init__(self):
        # Configuration parameters
        self.num_joints = 6  # 6-DOF robot arm

        # Joint order for STM encoding
        self.joint_order = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # Maximum velocity limits for each joint (rad/s)
        # These values should match the limits defined in joint_limits.yaml
        self.max_vel_rad = {
            "joint1": 1.5,
            "joint2": 1.5,
            "joint3": 1.5,
            "joint4": 2.0,
            "joint5": 2.0,
            "joint6": 2.5
        }

        # ROS topic names
        self.traj_topic = "/servo_server/joint_cmds"
        self.cmd_topic = "/velocity_commands"  # rk23_serial.py dinliyor

        # Initialize publisher and subscriber
        self.pub = rospy.Publisher(self.cmd_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(self.traj_topic,
                                    JointTrajectory,
                                    self.traj_cb,
                                    queue_size=10)

        rospy.loginfo("ServoJointVelEncoder node initialized successfully.")
        rospy.loginfo("Listening on topic: %s", self.traj_topic)
        rospy.loginfo("Publishing to topic: %s", self.cmd_topic)

    def encode_velocity(self, v, vmax):
        """
        Encodes a joint velocity into sign and magnitude format.

        Args:
            v (float): Joint velocity in rad/s
            vmax (float): Maximum velocity limit for this joint (rad/s)

        Returns:
            tuple: (sign, magnitude)
                - sign (str): '+' for positive, '-' for negative velocity
                - magnitude (int): Normalized value in range [0, 999]
        """
        if vmax <= 0.0:
            vmax = 1e-6  # Safety check to prevent division by zero

        v_abs = abs(v)

        # Apply velocity saturation
        if v_abs > vmax:
            v_abs = vmax

        # Normalize velocity: [0, vmax] -> [0, 999]
        mag_float = (v_abs / vmax) * 999.0
        mag = int(round(mag_float))

        # Clamp magnitude to valid range
        if mag < 0:
            mag = 0
        if mag > 999:
            mag = 999

        sign = '+' if v >= 0.0 else '-'
        return sign, mag

    def traj_cb(self, msg):
        """
        Callback function for JointTrajectory messages.
        
        Processes incoming trajectory messages from MoveIt Servo,
        encodes the joint velocities, and publishes the formatted string.
        
        Args:
            msg (JointTrajectory): Trajectory message containing joint velocities
        """
        # MoveIt Servo typically sends single-point trajectories
        if not msg.points:
            # Send stop command if message is empty
            stop_cmd = "V+000+000+000+000+000+000F"
            self.pub.publish(String(data=stop_cmd))
            rospy.logdebug("Empty trajectory, sending stop: %s", stop_cmd)
            return

        pt = msg.points[0]

        # Create dictionary: joint_name -> velocity
        vel_dict = dict(zip(msg.joint_names, pt.velocities))

        # Build encoded string in correct order
        buf = ["V"]
        for joint_name in self.joint_order:
            v = vel_dict.get(joint_name, 0.0)  # Default to 0 if missing
            vmax = self.max_vel_rad.get(joint_name, 1.0)
            sign, mag = self.encode_velocity(v, vmax)
            buf.append(sign)
            buf.append(f"{mag:03d}")

        buf.append("F")
        out_str = "".join(buf)

        msg_out = String(data=out_str)
        self.pub.publish(msg_out)

        rospy.logdebug("Encoded joint vel: %s", out_str)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("servo_joint_vel_encoder")
    node = ServoJointVelEncoder()
    node.spin()
