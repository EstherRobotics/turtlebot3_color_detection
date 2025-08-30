#!/usr/bin/env python3
"""
Color Image Processing Module

Provides image processing functionality for color detection including:
- Image display with resizing
- Color range definitions for different colors
- Color detection using HSV color space
- Contour detection and analysis

This module contains the ColorImageProcessor class with various image
processing methods for color-based object detection.
"""

import cv2
import numpy as np
import rospy


class ColorImageProcessor:
    """
    Image processor for color detection operations
    
    Handles image display, color range definitions, color detection,
    and contour analysis for object tracking.
    """
    
    def __init__(self, scale_factor=0.3):
        """
        Initialize the image processor
        """
        self.scale_factor = scale_factor
        rospy.logdebug(f"Image processor initialized with scale factor: {scale_factor}")
    
    def show_image(self, image, window_name):
        """
        Display image with resizing and automatic window management
        """
        try:
            # Resize image for display if scaling is needed
            if self.scale_factor != 1.0:
                display_image = cv2.resize(
                    image, 
                    None, 
                    fx=self.scale_factor, 
                    fy=self.scale_factor,
                    interpolation=cv2.INTER_AREA
                )
            else:
                display_image = image
            
            # Display the image
            cv2.imshow(window_name, display_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logwarn(f"Failed to display image: {e}")
    
    def get_color_range(self, color_name):
        """
        Get HSV color range for specified color name
        """
        color_name = color_name.lower()
        
        if color_name == 'red':
            # Red color range (handles wrap-around in HSV)
            lower_range = (0, 200, 102)
            upper_range = (15, 255, 255) 
            
        elif color_name == 'green':
            # Green color range
            lower_range = np.array([40, 50, 50])
            upper_range = np.array([80, 255, 255])
            
        elif color_name == 'blue':
            # Blue color range
            lower_range = np.array([100, 150, 50])
            upper_range = np.array([140, 255, 255])
            
        elif color_name == 'yellow':
            # Yellow color range
            lower_range = np.array([20, 100, 100])
            upper_range = np.array([30, 255, 255])
            
        else:
            rospy.logerr(f"Unsupported color: {color_name}")
            raise ValueError(f"Color '{color_name}' is not supported")
        return lower_range, upper_range

    def detect_color(self, image, *color_ranges):
        """
        Detect specified color in the image using HSV color space        
        """
        try:
            # Apply Gaussian blur to reduce noise
            blurred_image = cv2.GaussianBlur(image, (15, 15), 0)
            
            # Convert to HSV color space for better color segmentation
            hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
            
            # Initialize empty mask
            combined_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            
            # Process each color range
            for i in range(0, len(color_ranges), 2):
                if i + 1 < len(color_ranges):
                    lower = color_ranges[i]
                    upper = color_ranges[i + 1]
                    mask = cv2.inRange(hsv_image, lower, upper)
                    combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            # Apply morphological operations to clean up the mask
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
            cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)
            
            return cleaned_mask
            
        except Exception as e:
            rospy.logerr(f"Color detection failed: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8)
    
    def get_max_contour(self, binary_mask, min_area=1000):
        """
        Find the largest contour in a binary mask
        """
        try:
            # Find all contours in the binary mask
            contours, _ = cv2.findContours(
                binary_mask, 
                cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE
            )
            
            max_contour = None
            max_area = min_area
            center = (-1, -1)
            
            # Find contour with maximum area
            for contour in contours:
                current_area = cv2.contourArea(contour)
                
                if current_area > max_area:
                    max_area = current_area
                    max_contour = contour
                    
                    # Calculate contour center using moments
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                        center = (center_x, center_y)
            
            return max_contour, max_area, center
            
        except Exception as e:
            rospy.logwarn(f"Contour detection error: {e}")
            return None, 0, (-1, -1)