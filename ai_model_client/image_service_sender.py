import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import requests
import json

class ImageServiceSender(Node):
    def __init__(self):
        super().__init__('image_service_sender')
        
        # 1. Declare Parameters
        self.declare_parameter('server_url', 'http://localhost:5000/predict')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('service_name', 'trigger_ai_detection')

        # 2. Get Parameter Values
        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        
        # 3. Tools
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # 4. Subscriber
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
            
        # 5. Service Server
        self.srv = self.create_service(
            Trigger, 
            self.service_name, 
            self.handle_service_request)
        
        # 6. ENHANCED LOGGING: Configuration and Usage Instructions
        self.print_startup_instructions()

    def print_startup_instructions(self):
        """Prints a helpful guide to the console on how to use this node."""
        self.get_logger().info("==============================================")
        self.get_logger().info("IMAGE HTTP SERVICE SENDER STARTUP")
        self.get_logger().info(f"Endpoint: {self.server_url}")
        self.get_logger().info(f"Topic:    {self.image_topic}")
        self.get_logger().info(f"Service:  /{self.service_name}")
        self.get_logger().info("----------------------------------------------")
        self.get_logger().info("HOW TO USE:")
        self.get_logger().info(f" 1. Trigger detection via terminal:")
        self.get_logger().info(f"    ros2 service call /{self.service_name} std_srvs/srv/Trigger {{}}")
        self.get_logger().info(f" 2. Change configuration at runtime:")
        self.get_logger().info(f"    ros2 run ai_model_client image_service --ros-args -p service_name:='my_service'")
        self.get_logger().info("==============================================")

    def image_callback(self, msg):
        self.latest_frame = msg

    def handle_service_request(self, request, response):
        if self.latest_frame is None:
            self.get_logger().warn("Service called but no image available on topic!")
            response.success = False
            response.message = f"Error: No frames received on {self.image_topic}"
            return response

        self.get_logger().info("HTTP Request initiated...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            
            files = {'image': ('image.jpg', buffer.tobytes(), 'image/jpeg')}
            res = requests.post(self.server_url, files=files, timeout=10.0)
            
            if res.status_code == 200:
                data = res.json()
                response.success = True
                response.message = json.dumps(data)
                self.get_logger().info(f"Success! Response: {response.message[:50]}...")
            else:
                response.success = False
                response.message = f"Server Error Code: {res.status_code}"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Exception: {str(e)}"
            self.get_logger().error(f"HTTP Error: {e}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageServiceSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()