from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy import logging
from rclpy.duration import Duration


def main():
    rclpy.init()
    logger = logging.get_logger(name='nav_to_pose.py')
    logger.info('Waiting for an action server to become available... ')

    navigator = BasicNavigator()
    # Wait for navigation to be active
    navigator.waitUntilNav2Active()

    # Set the goal point coordinates
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.5
    goal_pose.pose.position.y = 2.0
    goal_pose.pose.orientation.w = 1.0

    # Send the goal and receive feedback
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(
            f"Estimated: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s to arrival"
        )
        # Cancel task if timeout
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=3.0):
            navigator.cancelTask()

    # Final result judgment
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info("Navigation result: Success! Reached x:2.5 y:2.0 w:1.0")
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn("Navigation result: Cancelled")
    elif result == TaskResult.FAILED:
        navigator.get_logger().error("Navigation result: Failed")
    else:
        navigator.get_logger().error("Navigation result: Invalid returned status")

if __name__ == '__main__':
    main()