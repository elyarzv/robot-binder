#!/usr/bin/env python3
import rospy
import actionlib
import tf.transformations

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from phoenix_msgs.msg import (
    SendRailGoalAction, SendRailGoalFeedback, SendRailGoalResult
)
from phoenix_msgs.srv import (
    SendRailGoal, StartLocalization, StartLocalizationResponse
)

import yaml
import numpy as np
import cv2
import math


class RailPositionPublisher:
    def __init__(self):
        rospy.init_node('rail_navigation')
        map_file_name = rospy.get_param('~map_file_name')
        rail_positions = rospy.get_param('~rail_positions')
        self.rail_positions = RailPositionPublisher.pixels_to_coords(
            rail_positions, map_file_name
        )

        self.initial_pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True
        )

        rospy.Service('~start_localization', StartLocalization, self.start_localization)

        self.send_rail_goal_service = rospy.Service(
            '~send_rail_goal_simple', SendRailGoal, self.send_rail_goal_simple
        )

        self.movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_markers)

        self.send_rail_goal_feedback = SendRailGoalFeedback()
        self.send_rail_goal_result = SendRailGoalResult()
        self.send_rail_goal_as = actionlib.SimpleActionServer(
            "~send_rail_goal", SendRailGoalAction,
            execute_cb=self.send_rail_goal, auto_start=False
        )
        self.send_rail_goal_as.start()

    def start_localization(self, req):
        pose = PoseWithCovarianceStamped()
        pose.pose.pose = self.get_rail_pose(req.rail_number)
        self.initial_pose_pub.publish(pose)
        print('start pose', pose)

    def send_rail_goal(self, goal):
        if 1 > goal.rail_number > len(self.rail_positions):
            rospy.logerr('invalid rail number %d' % goal.rail_number)
            self.send_goal_result.success = False
            self.send_rail_goal_as.set_aborted(self.send_goal_result.success)
            return

        if self.go_to_rail_number(goal.rail_number):
            if self._as.is_preempt_requested():
                rospy.loginfo('Rail goal preempted')
                self.send_rail_goal_as.set_preempted()
                return
            # TODO: compute feedback percentage based on start/current pose and goal
            self.send_rail_goal_feedback.percentage = 50
            self.send_rail_goal_result_as.publish_feedback(self.self.send_rail_goal_feedback)
            self.send_goal_result.success = True
            self.send_rail_goal_as.set_succeeded(self.send_goal_result.success)

    def send_rail_goal_simple(self, req):
        res = SendRailGoalResponse()

        if 1 > req.rail_number > len(self.rail_positions):
            res.success = False
            res.message = 'invalid rail number'
            return res

        if self.go_to_rail_number(req.rail_number):
            res.success = False
            res.message = 'Action server not available!'
        else:
            res.success = True
            res.message = ''

        return res

    def get_rail_pose(self, rail_number):
        backward = rail_number > (len(self.rail_positions) // 2)
        rail_idx = rail_number - 1
        rail_position = self.rail_positions[rail_idx]
        x_offset = 0.75 * (+1 if backward else -1)
        yaw = math.pi if backward else 0
        orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)

        pose = Pose()
        pose.position.x = rail_position[0] + x_offset
        pose.position.y = rail_position[1]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose

    def go_to_rail_number(self, rail_number):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.get_rail_pose(rail_number)

        rospy.loginfo('Waiting for move base...')
        self.movebase_client.wait_for_server()
        rospy.loginfo('move base found')

        self.movebase_client.send_goal(goal)
        wait = self.movebase_client.wait_for_result()
        if not wait:
            rospy.logerr('Action server not available!')
        else:
            rospy.loginfo(self.movebase_client.get_result())

        return wait

    @staticmethod
    def positions_to_markers(positions):
        marker_array = MarkerArray()

        for idx, position in enumerate(positions):
            marker = Marker()
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.MODIFY
            marker.id = idx
            marker.text = str(idx+1)
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.orientation.w = 1
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1

            marker_array.markers.append(marker)

        return marker_array

    def publish_markers(self, event=None):
        marker_array = RailPositionPublisher.positions_to_markers(
            self.rail_positions
        )
        self.marker_pub.publish(marker_array)

    @staticmethod
    def pixels_to_coords(pixel_positions, map_file_name):
        yaml_file = map_file_name + '.yaml'
        with open(yaml_file, 'r') as f:
            map_cfg = yaml.safe_load(f)

        map_pgm = map_file_name + '.pgm'
        map_pgm = cv2.imread(map_pgm)

        # origin is bottom left but pixels start from top left
        def flip_pixels(pixel_position):
            nonlocal map_pgm
            return np.asarray([
                pixel_position[0], map_pgm.shape[0] - pixel_position[1]
            ])

        origin = np.asarray(map_cfg['origin'][:2])
        coords = [
            origin + (flip_pixels(pixel_position) * map_cfg['resolution'])
            for pixel_position in pixel_positions
        ]

        return coords


if __name__ == '__main__':
    node = RailPositionPublisher()
    rospy.spin()
