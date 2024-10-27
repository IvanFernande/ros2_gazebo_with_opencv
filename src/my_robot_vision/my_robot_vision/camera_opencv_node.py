import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera_sensor_opencv/image_raw',
            self.image_callback,
            10)
        self.get_logger().info("Camera node has been started.")

    def image_callback(self, msg):
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Procesar la imagen (mostrarla en pantalla en este ejemplo)
        cv2.imshow("Robot Camera Feedback", cv_image)
        cv2.waitKey(1)  # Necesario para actualizar la imagen en OpenCV

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
