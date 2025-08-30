#!/usr/bin/env python3
"""
Velocity Control Module

Provides velocity calculation functionality for robot movement control
based on color detection results. Includes distance estimation and
angular velocity calculation for precise tracking.

This module contains the VelocityController class with methods for
calculating appropriate linear and angular velocities.
"""

import rospy
from geometry_msgs.msg import Twist


class VelocityController:
    """
    Velocity controller for robot movement based on detection parameters
    
    Calculates appropriate linear and angular velocities for tracking
    and following detected objects based on their size and position.
    """
    
    def __init__(self):
        """
        Initialize the velocity controller with default parameters
        """
        # Configuration parameters with default values
        self.max_area_threshold = rospy.get_param('~max_area_threshold', 450000)
        self.min_area_threshold = rospy.get_param('~min_area_threshold', 300000)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.15)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.15)
        self.center_offset_threshold = rospy.get_param('~center_offset_threshold', 100)
        
        rospy.logdebug("Velocity controller initialized")
        rospy.logdebug(f"Area thresholds: min={self.min_area_threshold}, max={self.max_area_threshold}")
    
    def calculate_velocity(self, velocity_msg, detection_area, object_x, image_center_x):
        """
        Calculate appropriate velocity based on detection parameters        
        """
        try:
            # Reset previous velocities
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            
            # Calculate distance-based linear velocity
            velocity_msg = self._calculate_linear_velocity(velocity_msg, detection_area)
            
            # Calculate position-based angular velocity (only if moving forward)
            if velocity_msg.linear.x > 0:
                velocity_msg.angular.z = self._calculate_angular_velocity(object_x, image_center_x)
            
            return velocity_msg
            
        except Exception as e:
            rospy.logwarn(f"Velocity calculation error: {e}")
            return velocity_msg
    
    def _calculate_linear_velocity(self, velocity_msg, detection_area):
        """
        Calculate linear velocity based on detected object area        
        """
        # Object too close - move backward
        if detection_area > self.max_area_threshold:
            rospy.logwarn("Object too close - moving backward")
            velocity_msg.linear.x = -0.1
        
        # Object too far - move forward
        elif detection_area < self.min_area_threshold:
            rospy.loginfo("Object detected - approaching")
            velocity_msg.linear.x = self.max_linear_speed
        
        # Object at optimal distance - maintain position
        else:
            rospy.loginfo("Object at optimal distance - maintaining position")
            velocity_msg.linear.x = 0.0
        
        return velocity_msg
    
    def _calculate_angular_velocity(self, object_x, image_center_x):
        """
        Calculate angular velocity to center the object in view
        """
        # Calculate offset from center
        offset = object_x - image_center_x
        absolute_offset = abs(offset)
        
        # No turning needed if object is centered
        if absolute_offset < self.center_offset_threshold:
            rospy.logdebug("Object centered - no turning required")
            return 0.0
        
        # Calculate proportional angular velocity
        proportional_gain = 0.002
        angular_velocity = -proportional_gain * offset
        
        # Limit angular velocity to maximum allowed
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
        
        # Log turning direction
        if angular_velocity > 0:
            rospy.loginfo(f"Turning left to center object (offset: {offset:.1f}px)")
        else:
            rospy.loginfo(f"Turning right to center object (offset: {offset:.1f}px)")
        
        return angular_velocity
    
    def update_parameters(self, **kwargs):
        """
        Update controller parameters dynamically
        """
        for param_name, param_value in kwargs.items():
            if hasattr(self, param_name):
                setattr(self, param_name, param_value)
                rospy.loginfo(f"Updated parameter {param_name} = {param_value}")
            else:
                rospy.logwarn(f"Attempted to update unknown parameter: {param_name}")