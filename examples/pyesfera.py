import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, '/visualization_marker', 10)
        self.timer_ = self.create_timer(1.0, self.publish_marker)
        self.get_logger().info('Marker Publisher Node has been started.')

    def publish_marker(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = 'my_frame'
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = 0.0
        marker_msg.pose.position.y = 0.0
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        # Tamaño de la esfera de 2 metros de diámetro
        diameter = 2.0
        marker_msg.scale.x = diameter
        marker_msg.scale.y = diameter
        marker_msg.scale.z = diameter

        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0
        marker_msg.lifetime.sec = 0

        self.publisher_.publish(marker_msg)

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisherNode()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
