#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


class AutoInitialPose:
    def __init__(self):
        self.initial_pose_x = float(rospy.get_param("~initial_pose_x", 0.0))
        self.initial_pose_y = float(rospy.get_param("~initial_pose_y", 0.0))
        self.initial_pose_a = float(rospy.get_param("~initial_pose_a", 0.0))
        self.xy_stddev = float(rospy.get_param("~xy_stddev", 0.08))
        self.yaw_stddev = float(rospy.get_param("~yaw_stddev", 0.35))
        self.warmup_enabled = get_bool_param("~warmup_enabled", True)
        self.warmup_angular_speed = float(rospy.get_param("~warmup_angular_speed", 0.25))
        self.warmup_duration = float(rospy.get_param("~warmup_duration", 2.0))
        self.warmup_return_enabled = get_bool_param("~warmup_return_enabled", False)
        self.escape_forward_enabled = get_bool_param("~escape_forward_enabled", True)
        self.escape_forward_speed = float(rospy.get_param("~escape_forward_speed", 0.10))
        self.escape_forward_distance = float(rospy.get_param("~escape_forward_distance", 0.30))
        self.clear_costmaps = get_bool_param("~clear_costmaps", True)

        self.initial_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.done_pub = rospy.Publisher(
            "/auto_initial_pose_done", Bool, queue_size=1, latch=True
        )

    def build_initial_pose_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y

        half_yaw = self.initial_pose_a * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        msg.pose.covariance[0] = self.xy_stddev ** 2
        msg.pose.covariance[7] = self.xy_stddev ** 2
        msg.pose.covariance[35] = self.yaw_stddev ** 2
        return msg

    def publish_initial_pose(self):
        rospy.loginfo(
            "Publishing /initialpose: x=%.3f y=%.3f yaw=%.3f",
            self.initial_pose_x,
            self.initial_pose_y,
            self.initial_pose_a,
        )
        rate = rospy.Rate(5)
        for _ in range(5):
            if rospy.is_shutdown():
                return
            msg = self.build_initial_pose_msg()
            msg.header.stamp = rospy.Time.now()
            self.initial_pose_pub.publish(msg)
            rate.sleep()

    def wait_for_amcl_pose(self):
        rospy.loginfo("Waiting for /amcl_pose after initial pose publish")
        try:
            rospy.wait_for_message(
                "/amcl_pose", PoseWithCovarianceStamped, timeout=10.0
            )
        except rospy.ROSException as exc:
            rospy.logerr("Timed out waiting for /amcl_pose: %s", exc)
            return False
        rospy.loginfo("/amcl_pose received")
        return True

    def publish_twist_for_duration(self, twist, duration):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def stop_base(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        rospy.sleep(0.1)
        self.cmd_vel_pub.publish(stop)

    def warmup_amcl(self):
        if not self.warmup_enabled:
            rospy.loginfo("AMCL warmup rotation disabled")
            return
        if self.warmup_duration <= 0.0 or self.warmup_angular_speed == 0.0:
            rospy.loginfo("AMCL warmup rotation skipped")
            return

        rospy.loginfo(
            "Publishing /cmd_vel warmup rotation: angular_z=%.3f duration=%.3f return=%s",
            self.warmup_angular_speed,
            self.warmup_duration,
            self.warmup_return_enabled,
        )
        forward = Twist()
        forward.angular.z = self.warmup_angular_speed
        backward = Twist()
        backward.angular.z = -self.warmup_angular_speed

        try:
            if self.warmup_return_enabled:
                half_duration = self.warmup_duration * 0.5
                self.publish_twist_for_duration(forward, half_duration)
                self.publish_twist_for_duration(backward, half_duration)
            else:
                self.publish_twist_for_duration(forward, self.warmup_duration)
        finally:
            self.stop_base()

    def escape_forward(self):
        if not self.escape_forward_enabled:
            rospy.loginfo("Initial forward escape disabled")
            return
        if self.escape_forward_distance <= 0.0 or self.escape_forward_speed == 0.0:
            rospy.loginfo("Initial forward escape skipped")
            return

        duration = abs(self.escape_forward_distance / self.escape_forward_speed)
        twist = Twist()
        twist.linear.x = self.escape_forward_speed
        rospy.loginfo(
            "Publishing /cmd_vel initial forward escape: linear_x=%.3f distance=%.3f duration=%.3f",
            self.escape_forward_speed,
            self.escape_forward_distance,
            duration,
        )
        try:
            self.publish_twist_for_duration(twist, duration)
        finally:
            self.stop_base()

    def clear_move_base_costmaps(self):
        if not self.clear_costmaps:
            rospy.loginfo("move_base costmap clearing disabled")
            return

        rospy.loginfo("Calling /move_base/clear_costmaps")
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=5.0)
            clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            clear_costmaps()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Could not clear move_base costmaps: %s", exc)

    def run(self):
        rospy.sleep(0.5)
        self.publish_initial_pose()
        if not self.wait_for_amcl_pose():
            return False
        self.warmup_amcl()
        self.escape_forward()
        self.clear_move_base_costmaps()
        self.done_pub.publish(Bool(data=True))
        rospy.loginfo("Published /auto_initial_pose_done")
        return True


def main():
    rospy.init_node("auto_initial_pose")
    node = AutoInitialPose()
    if node.run():
        rospy.spin()


if __name__ == "__main__":
    main()
