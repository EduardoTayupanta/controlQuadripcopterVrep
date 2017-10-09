#!/usr/bin/env python
"""
Ros node for control quadricopter running in simulation using V-REP
"""

import rospy
import message_filters
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

__author__ = "Eduardo Tayupanta"
__email__ = "wetayupanta@gmail.com"


class Quadricopter():
    def __init__(self, name):
        # Name Robot
        self.robot = name
        # Parameters Parcles
        self.pParam = 2
        self.iParam = 0
        self.dParam = 0
        self.vParam = -2
        self.cumul = 0
        self.lastE = 0
        self.pAlphaE = 0
        self.pBetaE = 0
        self.psp2 = 0
        self.psp1 = 0
        self.prevEuler = 0
        self.particleVelocity = [0, 0, 0, 0]
        # Vision Sensor
        # Instantiate CvBridge
        self.bridge = CvBridge()
        self.frontResolution = []
        self.frontImage = []
        self.frongimg = []

    def control_quadricopter(self, odom_base_pub, odom_targetObj_pub, odom_heli_pub, odom_matrix_pub):
        """Follow the line using the IR sensors under the robot."""
        # Vertical control:
        targetPos = [odom_targetObj_pub.pose.pose.position.x, odom_targetObj_pub.pose.pose.position.y,
                     odom_targetObj_pub.pose.pose.position.z]
        pos = [odom_base_pub.pose.pose.position.x, odom_base_pub.pose.pose.position.y,
               odom_base_pub.pose.pose.position.z]
        l = [odom_heli_pub.twist.twist.linear.x, odom_heli_pub.twist.twist.linear.y, odom_heli_pub.twist.twist.linear.z]
        e = targetPos[2] - pos[2]
        self.cumul = self.cumul + e
        pv = self.pParam * e
        thrust = 5.335 + pv + self.iParam * self.cumul + self.dParam * (e - self.lastE) + l[2] * self.vParam
        self.lastE = e

        # Horizontal control:
        sp = [targetPos[0] - pos[0], targetPos[1] - pos[1], targetPos[2] - pos[2]]
        m = [odom_matrix_pub.pose.pose.position.x, odom_matrix_pub.pose.pose.position.y,
             odom_matrix_pub.pose.pose.position.z, odom_matrix_pub.pose.pose.orientation.x,
             odom_matrix_pub.pose.pose.orientation.y, odom_matrix_pub.pose.pose.orientation.z,
             odom_matrix_pub.twist.twist.linear.x, odom_matrix_pub.twist.twist.linear.y,
             odom_matrix_pub.twist.twist.linear.z, odom_matrix_pub.twist.twist.angular.x,
             odom_matrix_pub.twist.twist.angular.y, odom_matrix_pub.twist.twist.angular.z]
        vx = [m[0] + m[3], m[4] + m[7], m[8] + m[11]]
        vy = [m[1] + m[3], m[5] + m[7], m[9] + m[11]]
        alphaE = vy[2] - m[11]
        alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - self.pAlphaE)
        betaE = vx[2] - m[11]
        betaCorr = -0.25 * betaE - 2.1 * (betaE - self.pBetaE)
        self.pAlphaE = alphaE
        self.pBetaE = betaE
        alphaCorr = alphaCorr + sp[1] * 0.005 + 1 * (sp[1] - self.psp2)
        betaCorr = betaCorr - sp[0] * 0.005 - 1 * (sp[0] - self.psp1)
        self.psp2 = sp[1]
        self.psp1 = sp[0]

        # Rotational control:
        euler = [odom_base_pub.pose.pose.orientation.x - odom_targetObj_pub.pose.pose.orientation.x,
                 odom_base_pub.pose.pose.orientation.y - odom_targetObj_pub.pose.pose.orientation.y,
                 odom_base_pub.pose.pose.orientation.z - odom_targetObj_pub.pose.pose.orientation.z]
        rotCorr = euler[2] * 0.1 + 2 * (euler[2] - self.prevEuler)
        self.prevEuler = euler[2]

        # Decide of the motor velocities:
        self.particleVelocity[0] = thrust * (1 - alphaCorr + betaCorr + rotCorr)
        self.particleVelocity[1] = thrust * (1 - alphaCorr - betaCorr - rotCorr)
        self.particleVelocity[2] = thrust * (1 + alphaCorr - betaCorr + rotCorr)
        self.particleVelocity[3] = thrust * (1 + alphaCorr + betaCorr - rotCorr)

        # Never publish after the node has been shut down
        if not rospy.is_shutdown():
            # Actually publish the message
            for x in range(0, 4):
                self.set_speed(self.particleVelocity, x)

    def set_speed(self, particleVelocity, x):
        # Set particle velocity with a Float32 topic
        msg = Float32()
        msg.data = particleVelocity[x]
        # Actually publish the topic
        if x == 0:
            pub_velPropeller1.publish(msg)
        elif x == 1:
            pub_velPropeller2.publish(msg)
        elif x == 2:
            pub_velPropeller3.publish(msg)
        elif x == 3:
            pub_velPropeller4.publish(msg)

    def get_visionSensor(self, frontVision_sensor):
        # Convert your ROS Image message to OpenCV2
        try:
            # Convert your ROS Image message to OpenCV2
            cv_image = self.bridge.imgmsg_to_cv2(frontVision_sensor, "bgr8")
        except CvBridgeError, e:
            print e

        (rows, cols, channels) = cv_image.shape
        # OpenCV
        self.frontResolution = [rows, cols]
        self.frontImage = cv_image
        self.frongimg = np.array(self.frontImage, dtype=np.uint8)
        self.frongimg.resize([self.frontResolution[0], self.frontResolution[1], 3])
        self.frongimg = np.rot90(self.frongimg, 2)
        self.frongimg = np.fliplr(self.frongimg)
        self.frongimg = cv2.cvtColor(self.frongimg, cv2.COLOR_RGB2BGR)

        # Show frame and exit with ESC
        cv2.imshow('Front Image', self.frongimg)
        tecla = cv2.waitKey(5) & 0xFF
        if tecla == 27:
            pass

    def stop_robot(self):
        rospy.loginfo("Stopping robot")
        for x in range(0, 4):
            self.set_speed([0, 0, 0, 0], x)

    def initialize(self):
        """Initialize the sample node."""
        global pub_velPropeller1
        global pub_velPropeller2
        global pub_velPropeller3
        global pub_velPropeller4

        # Provide a name for the node
        rospy.init_node("swarm_robotics", anonymous=True)

        # Give some feedback in the terminal
        rospy.loginfo("Swarm Robotics node initialization")

        # Subscribe to and synchronise parameters the robot
        odom_base_pub = message_filters.Subscriber("odom_base" + self.robot, Odometry)
        odom_targetObj_pub = message_filters.Subscriber("odom_targetObj" + self.robot, Odometry)
        odom_heli_pub = message_filters.Subscriber("odom_heli" + self.robot, Odometry)
        odom_matrix_pub = message_filters.Subscriber("odom_matrix_pub" + self.robot, Odometry)

        # Vision Sensor
        frontVision_sensor = message_filters.Subscriber("frontVision_sensor" + self.robot, Image)

        # Wait for all topics to arrive before calling the callback
        ts_control = message_filters.TimeSynchronizer(
            [odom_base_pub, odom_targetObj_pub, odom_heli_pub, odom_matrix_pub], 1)
        ts_camera = message_filters.TimeSynchronizer([frontVision_sensor], 1)

        # Register the callback to be called when all sensor readings are ready
        ts_control.registerCallback(self.control_quadricopter)
        ts_camera.registerCallback(self.get_visionSensor)

        # Publish the linear and angular velocities so the robot can move
        pub_velPropeller1 = rospy.Publisher("cmd_velPropeller1" + self.robot, Float32, queue_size=1)
        pub_velPropeller2 = rospy.Publisher("cmd_velPropeller2" + self.robot, Float32, queue_size=1)
        pub_velPropeller3 = rospy.Publisher("cmd_velPropeller3" + self.robot, Float32, queue_size=1)
        pub_velPropeller4 = rospy.Publisher("cmd_velPropeller4" + self.robot, Float32, queue_size=1)

        # Register the callback for when the node is stopped
        rospy.on_shutdown(self.stop_robot)

        # spin() keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == "__main__":
    # you can use multiple quadricopterrs with these statements
    x = Quadricopter('')
    x.initialize()
