#!/usr/bin/env python3
from typing import Optional, Tuple
import math
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from ptz_tracker_interfaces.action import TrackTarget
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray


class PTZTracker(Node):
    """Action server that tracks a target while a goal is active."""

    def __init__(self) -> None:
        super().__init__('ptz_tracker')

        # Parameters
        self.declare_parameter('detection_topic', '/person_detector_node/detection_array')
        self.declare_parameter('image_topic', '/robot/top_ptz_rgbd_camera/color/image_raw')
        self.declare_parameter('pan_joint', 'robot_top_ptz_camera_pan_joint')
        self.declare_parameter('tilt_joint', 'robot_top_ptz_camera_tilt_joint')
        self.declare_parameter('zoom_joint', 'robot_top_ptz_camera_zoom_color_joint')
        self.declare_parameter('follow_action', '/robot/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('horizontal_fov_deg', 63.7)
        self.declare_parameter('vertical_fov_deg', 32.0)
        self.declare_parameter('kp_pan', 0.2)
        self.declare_parameter('kp_tilt', 0.1)
        self.declare_parameter('kp_zoom', 0.1)
        self.declare_parameter('max_pan', math.radians(180.0))
        self.declare_parameter('min_pan', math.radians(-179.0))
        self.declare_parameter('max_tilt', math.radians(80.0))
        self.declare_parameter('min_tilt', math.radians(-30.0))
        self.declare_parameter('detection_timeout', 3.0)
        self.declare_parameter('command_period', 0.2)
        self.declare_parameter('default_positions', [0.0, 0.0, 1.0])
        self.declare_parameter('verbose', True)

        self.detection_topic = self.get_parameter('detection_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.pan_joint = self.get_parameter('pan_joint').value
        self.tilt_joint = self.get_parameter('tilt_joint').value
        self.zoom_joint = self.get_parameter('zoom_joint').value
        self.follow_action = self.get_parameter('follow_action').value
        self.kp_pan = float(self.get_parameter('kp_pan').value)
        self.kp_tilt = float(self.get_parameter('kp_tilt').value)
        self.kp_zoom = float(self.get_parameter('kp_zoom').value)
        self.max_pan = float(self.get_parameter('max_pan').value)
        self.min_pan = float(self.get_parameter('min_pan').value)
        self.max_tilt = float(self.get_parameter('max_tilt').value)
        self.min_tilt = float(self.get_parameter('min_tilt').value)
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.command_period = float(self.get_parameter('command_period').value)
        self.default_positions = [float(v) for v in self.get_parameter('default_positions').value]
        self.verbose = bool(self.get_parameter('verbose').value)

        self.h_fov = math.radians(float(self.get_parameter('horizontal_fov_deg').value))
        self.v_fov = math.radians(float(self.get_parameter('vertical_fov_deg').value))

        self.last_detection: Optional[Tuple[float, float]] = None
        self.last_detection_stamp: Optional[Time] = None
        self.image_width: Optional[int] = None
        self.image_height: Optional[int] = None
        self.current_cmd = list(self.default_positions)

        self._tracking_active = False
        self._current_goal_handle = None
        self._result_success = False
        self._result_message = ''

        self.action_client = ActionClient(self, FollowJointTrajectory, self.follow_action)

        self._detection_subscription = self.create_subscription(Detection2DArray, self.detection_topic, self.detections_cb, 10)
        self._image_subscription = self.create_subscription(Image, self.image_topic, self.image_info_cb, 10)
        #self.timer = self.create_timer(self.command_period, self.control_step)

        self._action_server = ActionServer(
            self,
            TrackTarget,
            '~/start_tracking',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        self.get_logger().info('Waiting for joint trajectory controller...')
        self.action_client.wait_for_server()
        self.get_logger().info('PTZ tracker ready (action server)')

    def goal_callback(self, goal_request: TrackTarget.Goal) -> GoalResponse:
        if not goal_request.start or self._tracking_active:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        if self.verbose:
            self.get_logger().info('Cancel requested')
        self._stop_tracking(success=False, message='Cancelled by client')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        if self.verbose:
            self.get_logger().info('Tracking goal accepted')
        self._start_tracking(goal_handle)

        while self._tracking_active and rclpy.ok():            
            ret = self.control_step()
            
            if ret is False:
                self.get_logger().error('Control step failed')
                self._stop_tracking(success=False, message='Control step failed')
                return self._build_result(True)

            if goal_handle.is_cancel_requested:
                self._stop_tracking(success=False, message='Cancelled by client')
                goal_handle.canceled()
                return self._build_result(True)
            
            feedback = TrackTarget.Feedback()
            feedback.current_pan = float(self.current_cmd[0])
            feedback.current_tilt = float(self.current_cmd[1])
            feedback.current_zoom = float(self.current_cmd[2])
            goal_handle.publish_feedback(feedback)

            time.sleep(self.command_period)

        result = self._build_result(False)
        if self._result_success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        self._current_goal_handle = None

        return result

    def detections_cb(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            self.last_detection = None
            return
        det = msg.detections[0]
        cx = det.bbox.center.position.x
        cy = det.bbox.center.position.y
        self.last_detection = (cx, cy)
        self.last_detection_stamp = Time.from_msg(msg.header.stamp)
        if self.verbose:
            self.get_logger().debug(f'Updated detection: cx={cx:.2f}, cy={cy:.2f}, stamp={self.last_detection_stamp.nanoseconds}')

    def image_info_cb(self, msg: Image) -> None:
        first_time = self.image_width is None or self.image_height is None
        self.image_width = msg.width
        self.image_height = msg.height
        if self.verbose and first_time:
            self.get_logger().info(f'Image size received: width={self.image_width}, height={self.image_height}')

    def control_step(self) -> bool:
        #self.get_logger().info(f'Executing control step... active? {self._tracking_active}')
        
        if self.image_width is None or self.image_height is None:
            if self.verbose:
                self.get_logger().error('Skipping control_step: image size not known yet')
            return False
        target_zoom = 1.0
        now = self.get_clock().now()
        if self.last_detection_stamp is not None:
            age = (now - self.last_detection_stamp).nanoseconds / 1e9
            if self.verbose:
                self.get_logger().debug(f'Detection age: {age:.3f}s')
            if age > self.detection_timeout:
                if self.verbose:
                    self.get_logger().warn(f'Target lost (age={age:.2f}s)')
                self._stop_tracking(success=False, message='Target lost')
                self.last_detection = None
                self.last_detection_stamp = None
                return False
        else:
            self.get_logger().error('No detection stamp available')
            return False

        if self.last_detection is None:
            if self.verbose:
                self.get_logger().debug('No detection available, holding position')
            return True

        cx, cy = self.last_detection
        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0

        dx = (cx - image_center_x) / image_center_x
        dy = (cy - image_center_y) / image_center_y

        pan_error = -dx * (self.h_fov / 2.0)
        tilt_error = -dy * (self.v_fov / 2.0)
        target_zoom = self.current_cmd[2]

        target_pan = self._clamp(self.current_cmd[0] + self.kp_pan * pan_error, self.min_pan, self.max_pan)
        target_tilt = self._clamp(self.current_cmd[1] + self.kp_tilt * tilt_error, self.min_tilt, self.max_tilt)
        target = [target_pan, target_tilt, target_zoom]

        if self.verbose:
            self.get_logger().debug(
                f'Control calc: dx={dx:.3f}, dy={dy:.3f}, pan_err={pan_error:.4f}, tilt_err={tilt_error:.4f}, '
                f'cmd_pan={target_pan:.4f}, cmd_tilt={target_tilt:.4f}, cmd_zoom={target_zoom:.4f}'
            )

        if any(abs(target[i] - self.current_cmd[i]) > 1e-4 for i in range(3)):
            if self.verbose:
                self.get_logger().debug(
                    f'Sending goal (delta pan={target[0]-self.current_cmd[0]:.5f}, '
                    f'delta tilt={target[1]-self.current_cmd[1]:.5f}, '
                    f'delta zoom={target[2]-self.current_cmd[2]:.5f})'
                )
            self.current_cmd = target
            self.send_goal(target)
        else:
            if self.verbose:
                self.get_logger().debug('No significant change, not sending new goal')
        return True

    def send_goal(self, positions) -> None:
        traj = JointTrajectory()
        traj.joint_names = [self.pan_joint, self.tilt_joint, self.zoom_joint]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(seconds=max(self.command_period, 0.2)).to_msg()
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        if self.verbose:
            self.get_logger().debug(f'Goal sent: pan={positions[0]:.4f}, tilt={positions[1]:.4f}')
        self.action_client.send_goal_async(goal_msg)

    def _start_tracking(self, goal_handle) -> None:
        self._tracking_active = True
        self._current_goal_handle = goal_handle
        self._result_success = False
        self._result_message = 'Tracking started'
        
    def _stop_tracking(self, success: bool, message: str) -> None:
        if not self._tracking_active:
            return
        self._tracking_active = False
        self._result_success = success
        self._result_message = message
        self.current_cmd = list(self.default_positions)
        self.send_goal(self.current_cmd)

    def _build_result(self, cancelled: bool):
        result = TrackTarget.Result()
        result.success = self._result_success and not cancelled
        result.message = self._result_message if self._result_message else ('Cancelled' if cancelled else '')
        result.final_pan = float(self.current_cmd[0])
        result.final_tilt = float(self.current_cmd[1])
        result.final_zoom = float(self.current_cmd[2])
        return result

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def destroy_node(self):
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PTZTracker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()