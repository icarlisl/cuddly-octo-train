import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBallTracker(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')
        
        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Image Processing
        self.bridge = CvBridge()
        self.frame_center = None
        
        # Control Parameters
        self.target_width = 100  # pixels (adjust based on desired following distance)
        self.target_center_tolerance = 20  # pixels
        
        # PID Constants
        self.kp_linear = 0.004
        self.kp_angular = 0.01
        self.kd_linear = 0.001
        self.kd_angular = 0.005
        
        # State variables
        self.prev_error_linear = 0
        self.prev_error_angular = 0
        
        # Debug window
        cv2.namedWindow('Ball Tracking', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.frame_center is None:
                self.frame_center = (frame.shape[1]//2, frame.shape[0]//2)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
            return

        # Ball detection logic
        ball_found, x, y, w, h = self.detect_red_ball(frame)
        cmd_vel = Twist()
        
        if ball_found:
            # Calculate control signals
            error_linear = w - self.target_width
            error_angular = (x + w/2) - self.frame_center[0]
            
            # PID control with derivative term
            cmd_vel.linear.x = self.kp_linear * error_linear + self.kd_linear * (error_linear - self.prev_error_linear)
            cmd_vel.angular.z = -self.kp_angular * error_angular - self.kd_angular * (error_angular - self.prev_error_angular)
            
            # Save previous errors
            self.prev_error_linear = error_linear
            self.prev_error_angular = error_angular
            
            # Draw debug info
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, self.frame_center, 5, (255, 0, 0), -1)
        else:
            # Stop if no ball detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.prev_error_linear = 0
            self.prev_error_angular = 0

        self.cmd_vel_pub.publish(cmd_vel)
        cv2.imshow('Ball Tracking', frame)
        cv2.waitKey(1)

    def detect_red_ball(self, frame):
        # Resize for faster processing
        frame = cv2.resize(frame, (640, 360))
        
        # HSV color thresholding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Noise reduction
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            return True, x, y, w, h
        return False, 0, 0, 0, 0

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RedBallTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot before exiting
        node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
