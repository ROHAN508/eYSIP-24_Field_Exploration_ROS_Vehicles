import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from custom_msgs.msg import Goal
x_coordinate = []
y_coordinate = []


class GoalNode(Node):
    def __init__(self):
        super().__init__('GoalNode')
        global x_coordinate, y_coordinate
        # self.Goal_ = Goal()
        # self.Goal_.x_coordinates = x_coordinate
        # self.Goal_.y_coordinates = y_coordinate
        self.pub = self.create_publisher(Goal, '/goal_topic', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        global Goal1
        self.pub.publish(Goal1)
        



def main(args=None):
    global Goal1
    rclpy.init(args=args)
    while True:
        sides=int(input("select no of sides (>1):"))
        if sides > 1:
            break
        else:
            print('invalid input')
        # Calculate the coordinates of a regular hexagon with side length 1 meter
    side_length = 2.5

# Calculate the angles for the vertices
    angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)

# Calculate the x and y coordinates
    y_coordinates = side_length * np.cos(angles)
    x_coordinates = side_length * np.sin(angles)

# Append the origin and close the loop by adding the first point again at the end
    first_x = x_coordinates[0]
    first_y = y_coordinates[0]
    x_coordinates = np.append(x_coordinates, first_x )
    y_coordinates = np.append(y_coordinates, first_y )

    for i in range(len(x_coordinates)):
        x_coordinates[i] = x_coordinates[i] - first_x
        y_coordinates[i] = -1*(y_coordinates[i] - first_y)

    print("x_coordinates:", x_coordinates)
    print("y_coordinates:", y_coordinates)


    # print("X coordinates:", x_coordinates)
    # print("Y coordinates:", y_coordinates)

    # Plot the hexagon
    plt.figure(figsize=(10, 10))
    plt.plot(x_coordinates,y_coordinates, marker='o', linestyle='-', color='b')
    plt.fill(x_coordinates,y_coordinates, 'lightblue', alpha=0.5)
    plt.title('Regular Hexagon with Side Length 1 Meter')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    plt.axis('equal')  # Ensure the aspect ratio is equal to make the hexagon look regular
    plt.show()
    Goal1 = Goal()
    Goal1.x_coordinates = x_coordinates
    Goal1.y_coordinates = y_coordinates
    # Goal1.x_coordinates = np.array([0,3])
    # Goal1.y_coordinates = np.array([0,0])
    goal = GoalNode()
    rclpy.spin(goal)
    goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()