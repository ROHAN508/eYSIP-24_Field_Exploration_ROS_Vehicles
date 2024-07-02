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
    y_coordinates = side_length*1.3 * np.cos(angles)
    x_coordinates = side_length*0.6*np.sin(2*angles)

# Append the origin and close the loop by adding the first point again at the end
    first_x = x_coordinates[0]
    first_y = y_coordinates[0]
    x_coordinates = np.append(x_coordinates, first_x )
    y_coordinates = np.append(y_coordinates, first_y )

    # x_coordinates = [0.49999999999999983, 0.7653675258913084, 1.0988039786893213, 1.495427990440091, 1.9503581931896679, 2.458713218984105, 3.015611699869455, 3.6161339411764812, 4.245876069570692, 4.868563931864995, 5.444850951833028, 5.935390553248438, 6.300836159884867, 6.501841195515956, 6.499059098836485, 6.282389409681897, 5.955923268556837, 5.651618104893777, 5.501431348125201, 5.604046158474596, 5.886054087704794, 6.218576994736202, 6.47270976241276, 6.534420536917244, 6.392362180210615, 6.078523996681939, 5.6250483024069595, 5.0640774134614075, 4.427753645921028, 3.748219315861557, 3.0576267316742634, 2.3882558128548306, 1.7724745935313206, 1.2426528596352249, 0.8311603970980326, 0.5703669918512355, 0.4926424298263203, 0.6178535124788597, 0.8817158609713076, 1.185232066905643, 1.4292959533181828, 1.5148013432452405, 1.3593905736549616, 1.0253932028414483, 0.6491791045431303, 0.36360634859967544, 0.22303896406375584, 0.21134217875565037, 0.3098758912196385, 0.49999999999999983]
    # y_coordinates = [0.4999999999999999, 0.26438804768784635, 0.14951487749032916, 0.13065508671719947, 0.1830832726782083, 0.282074032683107, 0.4029019640416468, 0.5208832252692939, 0.6216185478639471, 0.7144246325422584, 0.8119498927706547, 0.926842742015562, 1.0717515937434081, 1.259324861420619, 1.5022109559568704, 1.8080469384741054, 2.164903020789407, 2.5560744971870317, 2.964856661951234, 3.374641806606549, 3.7693238839158063, 4.132958545373403, 4.449601521111216, 4.705846272712547, 4.905806709484495, 5.060990965554703, 5.182933282473292, 5.283167901790387, 5.373229065056109, 5.464651013820588, 5.567556242720596, 5.674037933533794, 5.763740145512883, 5.816059437411111, 5.810392367981732, 5.726135495977993, 5.542685380153144, 5.245376979903695, 4.8595135857191165, 4.426885741789665, 3.9893356528768313, 3.588705523742121, 3.2594477293177895, 2.9721753123770385, 2.664833024412789, 2.278239299077783, 1.8131347890533516, 1.3240788135889403, 0.867543256099505, 0.5000000000000001]

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