import rospy
import numpy as np
import tf
import tf2_ros

from geometry_msgs.msg import Pose2D, Twist, TransformStamped
from visualization_msgs.msg import Marker
from robot_msgs.srv import ChangeColor, ChangeColorRequest, ChangeColorResponse

class  RobotSimulation:
  def __init__(self):
    self.robot_pose = Pose2D()
    self.robot_pose_br = tf2_ros.TransformBroadcaster()

    self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)

    self.time_frame = rospy.Time.now().to_sec()

  def cmd_vel_cb(self, data):
    new_time_frame = rospy.Time.now().to_sec()
    dt = new_time_frame - self.time_frame
    self.time_frame = new_time_frame

    dtheta = data.angular.z * dt
    dx = data.linear.x * dt * np.cos(self.robot_pose.theta + dtheta/2)
    dy = data.linear.x * dt * np.sin(self.robot_pose.theta + dtheta/2)

    self.robot_pose.x += dx
    self.robot_pose.y += dy
    self.robot_pose.theta += dtheta

    quaternion = tf.transformations.quaternion_from_euler(0, 0, self.robot_pose.theta)

    robot_transform_stamp = TransformStamped()
    robot_transform_stamp.header.stamp = rospy.Time.now()
    robot_transform_stamp.header.frame_id = "map"
    robot_transform_stamp.child_frame_id = "base_footprint"

    robot_transform_stamp.transform.translation.x = self.robot_pose.x
    robot_transform_stamp.transform.translation.y = self.robot_pose.y
    robot_transform_stamp.transform.translation.z = 0.0

    robot_transform_stamp.transform.rotation.x = quaternion[0]
    robot_transform_stamp.transform.rotation.y = quaternion[1]
    robot_transform_stamp.transform.rotation.z = quaternion[2]
    robot_transform_stamp.transform.rotation.w = quaternion[3]

    self.robot_pose_br.sendTransform(robot_transform_stamp)

  def  change_color_cb(self, req):
    self.robot_rviz.color = req.color
    res = ChangeColorResponse()
    res.success = True
    res.message = "OK"
    return res

def main():
  rospy.init_node("robot_simulation")
  robot_simulation= RobotSimulation()
  rospy.spin()
main()