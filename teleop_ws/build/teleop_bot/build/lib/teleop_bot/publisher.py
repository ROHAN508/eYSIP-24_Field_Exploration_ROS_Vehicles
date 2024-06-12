import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Publisher(Node):
    def __init__(self):
        super().__init__('Publisher_node')
       # Initialze Publisher with the "/Integer" topic
        self.pub = self.create_publisher(Int32, "/Integer",10)
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = Int32()
        msg.data = 1800
        self.pub.publish(msg)
       # Assign the msg variable to i
       # Publish the msg 
       # Increment the i

def main(args = None):

    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

