import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty, String


class UiNode(Node):
    def __init__(self):
        super().__init__('ui_node')  # initializes the node with the given name and default parameters

        self.target_pub = self.create_publisher(Pose2D, '/ui/target_pose', 10)    # creates a publisher that publishes Pose2D messages to the /ui/target_pose topic allowing up to 10 messages to be queued
        self.cancel_pub = self.create_publisher(Empty, '/ui/cancel_goal', 10)    # creates a publisher that publishes Empty messages to the /ui/cancel_goal topic allowing up to 10 messages to be queued

        self.status_sub = self.create_subscription(  # creates a subscriber that subscribes to String messages from the /ui/nav_status topic and calls the status_callback method when a message is received
            String,
            '/ui/nav_status',
            self.status_callback,
            10  # allows up to 10 messages to be queued
        )

        self.result_sub = self.create_subscription(  # creates a subscriber that subscribes to String messages from the /ui/nav_result topic and calls the result_callback method when a message is received
            String,
            '/ui/nav_result',
            self.result_callback,
            10  # allows up to 10 messages to be queued
        )

        # logs information about the available commands
        self.get_logger().info('Nav UI node started.')
        self.get_logger().info('Commands:')
        self.get_logger().info('  goal x y theta')
        self.get_logger().info('  cancel')
        self.get_logger().info('  quit')

    def status_callback(self, msg: String):
        # prints the navigation status received from the /ui/nav_status topic
        print(f'[STATUS] {msg.data}')

    def result_callback(self, msg: String):
        # prints the navigation result received from the /ui/nav_result topic
        print(f'[RESULT] {msg.data}')

    def publish_goal(self, x: float, y: float, theta: float):
        # creates a Pose2D message and publishes the message to the /ui/target_pose topic
        msg = Pose2D()
        msg.x = x
        msg.y = y
        msg.theta = theta
        self.target_pub.publish(msg)
        self.get_logger().info(
            f'Published goal: x={x:.3f}, y={y:.3f}, theta={theta:.3f}'
        )

    def publish_cancel(self):
        # Upon cancel request, creates an Empty message and publishes the message to the /ui/cancel_goal topic
        msg = Empty()
        self.cancel_pub.publish(msg)
        self.get_logger().warn('Published cancel request.')

def main(args=None):
    # initializes the rclpy client and creates a UiNode instance
    rclpy.init(args=args)
    node = UiNode() # creates an instance of UiNode and starts the ROS 2 node

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)     # Creates a background thread for rclpy.spin to handle ROS 2 communication asynchronously
    spin_thread.start()  

    try:
        while rclpy.ok():
            # starts the main thread to handle user input
            # takes user input and processes it
            user_input = input('Enter command: ').strip()

            if not user_input:
                continue

            if user_input.lower() == 'quit':
                break

            if user_input.lower() == 'cancel':
                node.publish_cancel()
                continue

            parts = user_input.split()    

            if parts[0].lower() == 'goal' and len(parts) == 4:
                try:
                    # attempts to convert the arguments to floats and publish them if successful
                    x = float(parts[1])
                    y = float(parts[2])
                    theta = float(parts[3])
                    node.publish_goal(x, y, theta)
                except ValueError:
                    print('Invalid numbers. Use: goal x y theta')
            else: # handles unknown commands and prints the available commands
                print('Unknown command.')
                print('Use:')
                print('  goal x y theta')
                print('  cancel')
                print('  quit')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # destroys the node and cleans up ROS 2 resources
        rclpy.shutdown()  # shuts down the ROS 2 client and terminates the program


if __name__ == '__main__':
    main()