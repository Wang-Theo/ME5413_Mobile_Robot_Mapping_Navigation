#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import SetBool, SetBoolResponse

class OdometryController:
    def __init__(self):
        rospy.init_node("odometry_controller")
        self.enable_odom_pub = rospy.get_param("~enable_odom_pub", False)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.odom_pub = rospy.Publisher("/odom_output", Odometry, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.goal_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_callback)

    def odom_callback(self, msg):
        if self.enable_odom_pub:
            self.odom_pub.publish(msg)

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        if linear_vel == 0.0 and angular_vel == 0.0:
            self.enable_odom_pub = False
        else:
            self.enable_odom_pub = True

    def goal_callback(self, msg):
        self.enable_odom_pub = True

if __name__ == "__main__":
    odomtry_controller = OdometryController()
    rospy.spin()

