import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

import struct

class PointCloud2Publisher(Node):
    def __init__(self):
        super().__init__('pointcloud2_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/os1/points', 10)
        self.timer_ = self.create_timer(1.0, self.publish_pointcloud)
        self.get_logger().info('PointCloud2 Publisher Node has been started.')

    def publish_pointcloud(self):
        # Create the PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = Header()
        pointcloud_msg.header.frame_id = 'laser_data_frame'

        # Populate the fields of the PointCloud2 message
        pointcloud_msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        pointcloud_msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        pointcloud_msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))

        # Populate the point data of the PointCloud2 message
        pointcloud_msg.point_step = 12
        pointcloud_msg.is_dense = True
        pointcloud_msg.is_bigendian = True

        pointcloud_msg.height = 15
        pointcloud_msg.width = 20

        pointcloud_msg.row_step = pointcloud_msg.width * pointcloud_msg.point_step
        n = pointcloud_msg.height * pointcloud_msg.width

        points = []  # Empty array to store the points

        #Para tener puntos fijos y comparar con los mensajes
        points.append(1.1)
        points.append(1.1)
        points.append(1.1)
        points.append(2.2)
        points.append(2.2)
        points.append(2.2)
        points.append(3.3)
        points.append(3.3)
        points.append(3.3)
        n = n - 3

        for _ in range(n):                        
            x= random.uniform(0.0, 10.0)  # Create a point as a list [x, y, z]
            points.append(x)
            y = random.uniform(0.0, 10.0)
            points.append(y)
            z = random.uniform(0.0, 10.0)
            # point = (x, y, z)
            points.append(z)  # Add the point to the points array

        # # Create a buffer to store the point data
        # buffer = []
        # for point in points:
        #     buffer.extend(struct.pack('fff', *point))

        # Convert the buffer to bytes
        pointcloud_msg.data = list(b''.join(struct.pack('f', f) for f in points))

        # Publish the PointCloud2 message
        self.publisher_.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_publisher = PointCloud2Publisher()
    rclpy.spin(pointcloud_publisher)
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''import struct

# Lista de floats
floats = [1.23, 4.56, 7.89, 8.52]

# Conversión a lista de bytes
msg.data = list(b''.join(struct.pack('f', f) for f in floats))

# Imprimir el resultado
print(byte_list)
'''




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2, PointField
# import numpy as np

# class PointCloudPublisher(Node):
#     def __init__(self):
#         super().__init__('point_cloud_publisher')
#         self.publisher_ = self.create_publisher(PointCloud2, '/camera/depth_registered/points', 10)

#     def publish_point_cloud(self):
#         # Create the PointCloud2 message
#         cloud_msg = PointCloud2()

#         # Set the header information
#         cloud_msg.header.stamp = self.get_clock().now().to_msg()
#         cloud_msg.header.frame_id = 'camera_link'  # Set the appropriate frame ID

#         # Define the fields of the PointCloud2 message
#         fields = [
#             PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#             PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#             PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
#         ]
#         cloud_msg.fields = fields

#         cloud_msg.point_step = 12
#         cloud_msg.row_step = pointcloud_msg.width * pointcloud_msg.point_step
#         cloud_msg.is_dense = True

#         cloud_msg.height = 30
#         cloud_msg.width = 30

#         # Generate a sample point cloud data
#         points = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]  # Replace with your actual point cloud data

#         # Convert the points to a numpy array
#         point_array = np.array(points, dtype=np.float32)

#         # Set the point cloud data
#         cloud_msg.data = point_array.tobytes()
#         cloud_msg.width = point_array.shape[0]
#         cloud_msg.height = 1
#         cloud_msg.point_step = 12  # 3 fields * 4 bytes (float32)
#         cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

#         # Publish the PointCloud2 message
#         self.publisher_.publish(cloud_msg)
#         self.get_logger().info('Published PointCloud2 message')

# def main(args=None):
#     rclpy.init(args=args)
#     point_cloud_publisher = PointCloudPublisher()
#     point_cloud_publisher.publish_point_cloud()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
