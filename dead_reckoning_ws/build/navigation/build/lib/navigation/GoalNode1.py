import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from custom_msgs.msg import Goal

class GoalNode(Node):
    def __init__(self):
        super().__init__('GoalNode')
        self.pub = self.create_publisher(Goal, '/goal_topic', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.index = 0
        self.x_coordinates = []
        self.y_coordinates = []

    def timer_callback(self):
        if self.index < len(self.x_coordinates):
            goal_msg = Goal()
            goal_msg.x_coordinates = self.x_coordinates[self.index:self.index+1]
            goal_msg.y_coordinates = self.y_coordinates[self.index:self.index+1]
            self.pub.publish(goal_msg)
            self.index += 1

def main(args=None):
    rclpy.init(args=args)
    sides = 0
    while sides <= 1:
        sides = int(input("Select number of sides (>1):"))
        if sides <= 1:
            print('Invalid input')

    side_length = 2.5

    # Calculate the angles for the vertices
    angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)

    # Calculate the x and y coordinates
    x_coordinates = side_length * np.cos(angles)
    y_coordinates = side_length * np.sin(angles)

    # Append the origin and close the loop by adding the first point again at the end
    first_x = x_coordinates[0]
    first_y = y_coordinates[0]
    x_coordinates = np.append(x_coordinates, first_x)
    y_coordinates = np.append(y_coordinates, first_y)

    for i in range(len(x_coordinates)):
        x_coordinates[i] = x_coordinates[i] - first_x
        y_coordinates[i] = -1 * (y_coordinates[i] - first_y)

    print("x_coordinates:", x_coordinates)
    print("y_coordinates:", y_coordinates)

    # Plot the shape
    plt.figure(figsize=(10, 10))
    plt.plot(x_coordinates, y_coordinates, marker='o', linestyle='-', color='b')
    plt.fill(x_coordinates, y_coordinates, 'lightblue', alpha=0.5)
    plt.title('Regular Polygon with Specified Number of Sides')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    plt.axis('equal')  # Ensure the aspect ratio is equal to make the shape look regular
    plt.show()

    goal_node = GoalNode()
    goal_node.x_coordinates = x_coordinates
    goal_node.y_coordinates = y_coordinates

    rclpy.spin(goal_node)
    goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
