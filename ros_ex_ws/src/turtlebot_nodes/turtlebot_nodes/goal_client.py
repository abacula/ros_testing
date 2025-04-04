import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from custom_interfaces.action import RobotGoal

class GoalClientNode(Node):

    def __init__(self):

        super().__init__('goal_client')

        self.goal_client = ActionClient(self, RobotGoal, "go_to_goal")

        while not self.goal_client.wait_for_server(1.0):
            self.get_logger().info("Waiting for server")


    # Send Goal
    def send_goal(self, x, y, theta):
        goal = RobotGoal.Goal()
        goal.goal_x = x
        goal.goal_y = y
        goal.goal_theta = theta
        self.goal_client.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)


    # Process Goal Accept/Reject
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle.accepted:
            self.get_logger().info("Goal got accepted")
            goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected: ripperoni")


    # Process Goal Feedback
    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.distance
        self.get_logger().info("Got feedback: " )


    # Process Action Result
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        # if status == GoalStatus.STATUS_SUCCEEDED:
        #     self.get_logger().info("Success!")
        #     acrostic = result.success
        #     self.get_logger().info(acrostic)
        # else:
        #     self.get_logger().info("No poem for you")

        rclpy.shutdown()

def main(args=None):
    # Start up rclpy
    rclpy.init(args=args)

    # Create instance of node
    action_client = GoalClientNode()

    # Send the goal
    action_client.send_goal(-1.0,-1.0,0.0)

    # Normal spin, yay! (callback handles shutdown)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()