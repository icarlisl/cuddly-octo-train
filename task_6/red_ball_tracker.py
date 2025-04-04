import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBallTracker(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
        self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_image, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.cv_bridge = CvBridge()
        
        self.color_mins = np.array([0, 150, 150]) # hue, saturation, value
        self.color_maxs = np.array([10, 255, 255])
        
        # robot control parameters
        self.screen_center_location = None
        self.target_ball_size = 100
        self.speed = 0.1
        self.buffer_zone = 60  # prevent oscillations
        
        # pid parameters
        self.kp = 0.0008  
        self.ki = 0.0001  
        self.kd = 0.0002  
  
        self.previous_error = 0.0
        self.integral = 0.0
        self.max_turn_speed = 0.18

        cv2.namedWindow('Robot View')
        
    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # converts camera view into hsv view to determine colors
        red = cv2.inRange(hsv, self.color_mins, self.color_maxs)
      
        contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            ball_area = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(ball_area)
            return (True, x, y, w, h)
        
        return (False, 0, 0, 0, 0)


    def process_image(self, msg):
        try:
            camera_frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.screen_center_location is None: # first round to set location of center of screen
                screen_width = camera_frame.shape[1]
                self.screen_center_location = screen_width // 2
        except:
            return

        velocity = Twist()
        ball_found, x, y, w, h = self.detect_ball(camera_frame)
        
        if ball_found:
            ball_center = x + (w // 2)
            center_error = self.screen_center_location - ball_center
            
            # distance from ball
            if w > self.target_ball_size + 10: # 10 for oscillations
                velocity.linear.x = -self.speed * 0.4
            elif w < self.target_ball_size - 10:
                velocity.linear.x = self.speed
            else:
                velocity.linear.x = 0.0

            if abs(center_error) > self.buffer_zone:
                self.integral += center_error
                self.derivative = center_error - self.previous_error
                
                # controller
                turning_control = (self.kp * center_error) + (self.ki * self.integral) + (self.kd * self.derivative)
                
                self.previous_error = center_error
                
                turning_control = max(min(turning_control, self.max_turn_speed), -self.max_turn_speed) # troubleshooting
                velocity.angular.z = turning_control
                
            else: # clears PID components when in buffer zone and stops robot
                self.integral = 0.0
                self.previous_error = 0.0
                velocity.angular.z = 0.0

            cv2.rectangle(camera_frame, (x, y), (x+w, y+h), (0,255,0), 2)
            
        else: # clears PID components when ball is lost and looks for ball
            self.integral = 0.0
            self.previous_error = 0.0
            velocity.angular.z = 0.4  

        self.velocity_publisher.publish(velocity)
        cv2.imshow('Robot View', camera_frame)
        cv2.waitKey(1) # cv thing


def main(args=None):
    rclpy.init(args=args)
    red_ball_tracker = RedBallTracker()
    rclpy.spin(red_ball_tracker)
    red_ball_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
