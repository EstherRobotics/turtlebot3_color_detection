#!/usr/bin/env python3
"""
Color Detection and Tracking Node

Main ROS node for detecting colored objects and controlling robot movement
to track and follow the detected objects.

Dependencies:
- Run color_detection launch commands before execution
- Requires color_image.py and velocity.py modules

Execute with: python3 color_detection.py
"""

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from color_image import ColorImageProcessor
from velocity import VelocityController


class ColorDetectionNode:
    """Main ROS node for color detection and robot tracking control"""
    
    def __init__(self):
        """
        Initialize the color detection node
        Sets up ROS parameters, publishers, subscribers and processing modules
        """
        # Initialize ROS node
        rospy.init_node('color_detection_node', anonymous=True)
        
        # Configurable parameters with default values
        self.target_color = rospy.get_param('~target_color', 'red')
        self.min_detection_area = rospy.get_param('~min_detection_area', 30000)
        self.image_scale_factor = rospy.get_param('~image_scale_factor', 0.3)
        
        # Initialize processing modules
        self.image_processor = ColorImageProcessor(self.image_scale_factor)
        self.velocity_controller = VelocityController()
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Setup ROS publisher for velocity commands
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Setup ROS subscriber for camera images
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        rospy.loginfo("Color Detection Node initialized successfully")
        rospy.loginfo(f"Target color: {self.target_color}")
        rospy.loginfo(f"Minimum detection area: {self.min_detection_area}")
        rospy.loginfo("Waiting for camera images...")
    
    def image_callback(self, msg):
        """
        Callback function for processing incoming image messages
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.logdebug("Successfully converted ROS image to OpenCV format")
            
            # Process the image and control robot movement
            self.process_image(cv_image)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge conversion error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in image callback: {e}")
    
    def process_image(self, image):
        """
        Process the image to detect target color and control robot movement        
        """
        try:
            # Display original camera image
            self.image_processor.show_image(image, "Robot Camera")
            
            # Get image dimensions for navigation calculations
            image_height, image_width = image.shape[:2]
            image_center_x = image_width / 2
            
            # Get color range parameters for target color
            lower_range, upper_range = self.image_processor.get_color_range(self.target_color)
            
            # Detect target color in the image
            color_mask = self.image_processor.detect_color(image, lower_range, upper_range)
            self.image_processor.show_image(color_mask, f"{self.target_color.capitalize()} Color Detection Mask")
            
            # Find the largest contour in the detection mask
            contour, area, center = self.image_processor.get_max_contour(color_mask)
            rospy.logdebug(f"Detected area: {area} pixels")
            
            # Initialize velocity command
            velocity_command = Twist()
            
            if area > self.min_detection_area:
                rospy.loginfo(f"Target color detected with area: {area} pixels")
                
                # Visualize the detection results
                self.visualize_detection(image, contour, center)
                
                # Calculate appropriate velocity based on detection
                velocity_command = self.velocity_controller.calculate_velocity(
                    velocity_command, area, center[0], image_center_x
                )
            else:
                rospy.logwarn("No significant color detection - initiating search")
                velocity_command.angular.z = 0.6  # Rotate to search for target
            
            # Publish the velocity command
            self.cmd_vel_publisher.publish(velocity_command)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def visualize_detection(self, image, contour, center):
        """
        Draw detection visualization elements on the image
        
        Args:
            image (numpy.ndarray): Original image to draw on
            contour: Detected contour points
            center (tuple): (x, y) coordinates of detection center
        """
        try:
            # Draw the detected contour
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 3, cv2.LINE_AA)
            
            # Draw center point of detection
            cv2.circle(image, center, 8, (0, 200, 200), -1)
            cv2.circle(image, center, 12, (0, 200, 200), 2)
            
            # Add informational text overlay
            cv2.putText(image, f"Target: {self.target_color}", (15, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(image, f"Area: {int(center[0])},{int(center[1])}", (15, 75),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Display the annotated image
            self.image_processor.show_image(image, "Detection Results")
            
        except Exception as e:
            rospy.logwarn(f"Visualization error: {e}")
    
    def run(self):
        """
        Main execution loop for the node        
        """
        rospy.loginfo("Color Detection Node started")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Node shutdown requested")
        finally:
            cv2.destroyAllWindows()
            rospy.loginfo("Color Detection Node shutdown complete")


if __name__ == '__main__':
    try:
        node = ColorDetectionNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"Failed to initialize Color Detection Node: {e}")