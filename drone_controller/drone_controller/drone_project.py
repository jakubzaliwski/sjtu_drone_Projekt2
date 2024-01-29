import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Subscribe to the current position
        self.gt_pose_sub = self.create_subscription(
            Pose,
            '/drone/gt_pose',
            self.pose_callback,
            1)

        self.gt_pose = None

        # Publisher for control commands
        self.command_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Timer callback for control command execution
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Starting point (3 meters above ground)
        self.starting_point = Point(x=0.0, y=0.0, z=3.0)

        # Start time of the movement
        self.start_time = time.time()

        # Current flight stage
        self.flight_stage = 0

    def pose_callback(self, data):
        self.gt_pose = data

    def timer_callback(self):
        if self.gt_pose is not None:
            # Check if at least 2 seconds have passed since the start of the movement
            elapsed_time = time.time() - self.start_time

            if elapsed_time >= 8.0:
                # Move to the next flight stage
                self.flight_stage += 1
                # Reset the start time for the new stage
                self.start_time = time.time()

            # Execute movement according to the current flight stage
            self.execute_flight_stage()

    def execute_flight_stage(self):
        cmd = Twist()

        if self.flight_stage == 0:
            # Ascend by 1 meter (upward)
            cmd.linear.z = 1.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 1:
            # Move 2 meters to the left
            cmd.linear.z = 3.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 2:
            # Move 2 meters downward
            cmd.linear.z = 3.0
            cmd.linear.x = 2.0
        elif self.flight_stage == 3:
            # Move 2 meters to the right
            cmd.linear.z = 1.0
            cmd.linear.x = 2.0
        elif self.flight_stage == 4:
            # Return to the starting position
            cmd.linear.z = 1.0
            cmd.linear.x = 0.0
        elif self.flight_stage == 5:
            # Return to the ground
            cmd.linear.z = 0.0
            cmd.linear.x = 0.0

        # Send the control command
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = DroneController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by the user. Closing.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



