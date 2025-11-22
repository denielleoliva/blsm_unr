#!/usr/bin/env python3
"""
Gesture Server Node
Provides action server interface for playing gestures with feedback
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time


class GestureServer(Node):
    """Action server for executing gestures on Blossom."""
    
    def __init__(self):
        super().__init__('gesture_server')
        
        # Callback groups for concurrent execution
        self.cb_group = ReentrantCallbackGroup()
        
        # Current execution state
        self.current_goal_handle = None
        self.executing = False
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            'sequence_status',
            self.status_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Publisher for sequence commands
        self.sequence_pub = self.create_publisher(
            String,
            'play_sequence',
            10
        )
        
        # Action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'execute_gesture',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        
        self.get_logger().info('Gesture action server ready')
    
    def goal_callback(self, goal_request):
        """Handle new goal requests."""
        if self.executing:
            self.get_logger().warn('Rejecting new goal - already executing')
            return GoalResponse.REJECT
        
        self.get_logger().info('Accepting new gesture goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('Gesture cancellation requested')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """
        Execute a gesture trajectory.
        
        Args:
            goal_handle: The goal to execute
        """
        self.get_logger().info('Executing gesture...')
        
        self.executing = True
        self.current_goal_handle = goal_handle
        
        request = goal_handle.request
        trajectory = request.trajectory
        
        # Feedback message
        feedback_msg = FollowJointTrajectory.Feedback()
        
        try:
            # Process each point in the trajectory
            for i, point in enumerate(trajectory.points):
                # Check if goal has been cancelled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Gesture cancelled')
                    self.executing = False
                    return FollowJointTrajectory.Result()
                
                # Create and publish joint command
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = trajectory.joint_names
                joint_msg.position = list(point.positions)
                
                # Calculate time to next point
                if i < len(trajectory.points) - 1:
                    next_time = trajectory.points[i + 1].time_from_start
                    current_time = point.time_from_start
                    wait_time = (next_time.sec - current_time.sec) + \
                               (next_time.nanosec - current_time.nanosec) / 1e9
                else:
                    wait_time = 0.1
                
                # Publish feedback
                feedback_msg.joint_names = trajectory.joint_names
                feedback_msg.desired.positions = list(point.positions)
                feedback_msg.actual.positions = list(point.positions)  # Simplified
                goal_handle.publish_feedback(feedback_msg)
                
                # Wait for next point
                await self.get_clock().sleep_for(
                    rclpy.duration.Duration(seconds=max(0.01, wait_time))
                )
            
            # Goal succeeded
            goal_handle.succeed()
            
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            
            self.get_logger().info('Gesture completed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error executing gesture: {e}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        
        finally:
            self.executing = False
            self.current_goal_handle = None
        
        return result
    
    def status_callback(self, msg: String):
        """Handle status updates from sequence player."""
        status = msg.data
        self.get_logger().debug(f'Sequence status: {status}')
        
        # Parse status
        if ':' in status:
            status_type, sequence_name = status.split(':', 1)
            
            if status_type == 'completed' and self.current_goal_handle:
                self.get_logger().info(f'Sequence {sequence_name} completed')


def main(args=None):
    rclpy.init(args=args)
    node = GestureServer()
    
    # Use MultiThreadedExecutor for concurrent callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
