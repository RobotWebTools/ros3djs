import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer_ = self.create_timer(1.0, self.publish_map)  # Publicar cada 1 segundo
        self.map_ = OccupancyGrid()
        self.map_.header.frame_id = 'map'
        self.map_.info.width = 100  # Ancho del mapa
        self.map_.info.height = 100  # Alto del mapa
        self.map_.info.resolution = 0.1  # Resolución del mapa (metros/píxel)
        self.map_.data = [0] * (self.map_.info.width * self.map_.info.height)  # Datos de ocupación del mapa

    def publish_map(self):
        # Generar datos realistas del mapa (aquí puedes inventar tus propios datos)
        for i in range(len(self.map_.data)):
            self.map_.data[i] = 100  # Valor de ocupación máximo (totalmente ocupado)

        self.publisher_.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    publisher = MapPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

