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
        
        # 1. Configuration
        self.declare_parameter('server_url', 'http://your-cloud-server.com/predict')
        self.server_url = self.get_parameter('server_url').value
        
        # 2. Tools
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # 3. Subscriber (Just to keep the latest frame ready)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # 4. Service Server
        self.srv = self.create_service(Trigger, 'trigger_ai_detection', self.handle_service_request)
        
        self.get_logger().info("AI Detection Service is ready. Call '/trigger_ai_detection'")

    def image_callback(self, msg):
        # Simply store the most recent frame
        self.latest_frame = msg

    def handle_service_request(self, request, response):
        if self.latest_frame is None:
            response.success = False
            response.message = "No image received from camera yet."
            return response

        self.get_logger().info("Service triggered! Uploading latest frame...")

        try:
            # Convert and Encode
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image)
            
            # Synchronous HTTP Post (Fine for service calls as they are intended to be blocking)
            files = {'image': ('image.jpg', buffer.tobytes(), 'image/jpeg')}
            res = requests.post(self.server_url, files=files, timeout=10.0)
            
            if res.status_code == 200:
                # Assuming the server returns JSON detection data
                data = res.json()
                response.success = True
                response.message = json.dumps(data) # Return detection results as a string
                self.get_logger().info("Detection successful.")
            else:
                response.success = False
                response.message = f"Server Error: {res.status_code}"
                
        except Exception as e:
            response.success = False
            response.message = f"Exception: {str(e)}"
            self.get_logger().error(f"Service call failed: {e}")

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