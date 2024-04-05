#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.clock import ROSClock
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
import sensor_msgs.msg
from synapse_msgs.msg import RoadCurveAngle
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
import cv2

if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))


def to_black_and_white(bgr_image):
    gray = cv2.bitwise_not(cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY))
    (_, black_and_white) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    return black_and_white


def extract_lines_x_axis(warped_black_and_white):
    histogram = np.sum(warped_black_and_white[int(warped_black_and_white.shape[0] / 2):, :], axis=0)

    midpoint = int(histogram.shape[0] / 2)

    return np.argmax(histogram[:midpoint]), np.argmax(histogram[midpoint:]) + midpoint


def search_point(white_pixels, up, bottom, left, right):
    white_x, white_y = white_pixels

    indices = ((white_y >= up) & (white_y < bottom) & (white_x >= left) & (white_x < right)).nonzero()[0]

    if len(indices) > 25:
        current_x = int(np.mean(white_x[indices]))
        current_y = (bottom + up) / 2

        return int(current_x), int(current_y)

    return None


def calculate_road_curve_angle(left_points, right_points):
    total_theta = 0
    thetas = []
    total_phi = 0
    phis = []

    left_total_length = 0
    left_lengths = []

    right_total_length = 0
    right_lengths = []

    for k in range(0, len(left_points) - 1):
        theta = np.arctan((left_points[k][0] - left_points[k + 1][0]) / (left_points[k][1] - left_points[k + 1][1]))

        thetas.append(theta)

        length = (((left_points[k][1] - left_points[k + 1][1]) ** 2) + (
                (left_points[k][0] - left_points[k + 1][0]) ** 2)) ** 0.5

        left_lengths.append(length)

        left_total_length += length
        total_theta += length * theta

    total_theta = total_theta / left_total_length

    for k in range(0, len(right_points) - 1):
        phi = np.arctan(
            (right_points[k + 1][0] - right_points[k][0]) / (right_points[k + 1][1] - right_points[k][1]))

        phis.append(phi)

        length = (((right_points[k][1] - right_points[k + 1][1]) ** 2) + (
                (right_points[k][0] - right_points[k + 1][0]) ** 2)) ** 0.5

        right_lengths.append(length)

        right_total_length += length
        total_phi += length * phi

    total_phi = total_phi / right_total_length

    alpha = (len(thetas) * total_theta + len(phis) * total_phi) / (len(thetas) + len(phis))

    return alpha


class TrackVision(Node):

    def __init__(self):
        super().__init__("track_vision")

        self.bridge = CvBridge()

        self.imageHeight = 240
        self.imageWidth = 320

        # Subscribers
        _ = self.create_subscription(sensor_msgs.msg.CompressedImage,
                                     'camera/image_raw/compressed',
                                     self.camera_image_callback,
                                     qos_profile_sensor_data)

        # Publishers
        self.vision = self.create_publisher(sensor_msgs.msg.CompressedImage,
                                            "vision/image_raw/compressed", 0)

        self.road_curve_angle = self.create_publisher(RoadCurveAngle, "cerebri/in/road_curve_angle", 0)

        self.roi = np.float32([
            (70, 108),  # Top-left corner
            (0, 183),  # Bottom-left corner
            (320, 183),  # Bottom-right corner
            (238, 108)  # Top-right corner
        ])

        self.search_height = 15
        self.search_width = 25 * 2

    def warped_image(self, image):
        desired_roi_points = np.float32([
            [0, 0],
            [0, self.imageHeight],
            [self.imageWidth, self.imageHeight],
            [self.imageWidth, 0]
        ])

        return cv2.warpPerspective(image,
                                   cv2.getPerspectiveTransform(self.roi, desired_roi_points),
                                   (self.imageWidth, self.imageHeight),
                                   flags=cv2.INTER_LINEAR)

    def extract_points(self, warped_black_and_white):
        white_pixels = warped_black_and_white.nonzero()
        white_pixels = np.array(white_pixels[1]), np.array(white_pixels[0])

        left, right = extract_lines_x_axis(warped_black_and_white)

        left_points = []
        right_points = []

        for search in range(0, int(self.imageHeight / self.search_height)):
            up = self.imageHeight - (search + 1) * self.search_height
            bottom = self.imageHeight - search * self.search_height

            left_left = left - (self.search_width / 2)
            left_right = left + (self.search_width / 2)

            right_left = right - (self.search_width / 2)
            right_right = right + (self.search_width / 2)

            left_point, right_point = (search_point(white_pixels, up, bottom,
                                                    left_left, left_right),
                                       search_point(white_pixels, up, bottom,
                                                    right_left, right_right))

            if left_point is not None:
                left = left_point[0]
                left_points.append(left_point)

            if right_point is not None:
                right = right_point[0]
                right_points.append(right_point)

        return left_points, right_points

    def camera_image_callback(self, data):
        scene = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')

        warped_black_and_white = self.warped_image(to_black_and_white(scene))

        left_points, right_points = self.extract_points(warped_black_and_white)

        warped = cv2.cvtColor(warped_black_and_white, cv2.COLOR_BGR2RGB)

        for left_point in left_points:
            cv2.circle(warped, (left_point[0], left_point[1]), 3, (255, 0, 0), -1)

        for right_point in right_points:
            cv2.circle(warped, (right_point[0], right_point[1]), 3, (255, 0, 0), -1)

        angle = calculate_road_curve_angle(left_points, right_points)

        road_curve_angle_msg = RoadCurveAngle()
        now = ROSClock().now()
        road_curve_angle_msg.header.stamp = now.to_msg()
        road_curve_angle_msg.angle = angle

        self.road_curve_angle.publish(road_curve_angle_msg)
        self.vision.publish(self.bridge.cv2_to_compressed_imgmsg(warped))


def main(args=None):
    rclpy.init(args=args)
    node = TrackVision()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
