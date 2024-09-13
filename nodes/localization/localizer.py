#!/usr/bin/env python3

import math
import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer, Proj

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('undulation')
        utm_origin_lat = rospy.get_param('utm_origin_lat')
        utm_origin_lon = rospy.get_param('utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()
        
        # Create coordinate transformer
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        # Transform the origin point
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)
                
        
    def transform_coordinates(self, msg):
        
        # Transform incoming msgs from WGS84 to UTM (our map coordinate system)
        x_utm, y_utm = self.transformer.transform(msg.latitude, msg.longitude)
        
        # Substract our custom location point
        pos_utm_x, pos_utm_y = x_utm-self.origin_x, y_utm-self.origin_y 
        print(pos_utm_x, pos_utm_y)
        
        # Calculate azimuth correction for UTM zone 35N projection
        azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
        
        # The correction is subtracted from azimuth angle coming from the message
        corrected_azymuth = msg.azimuth - azimuth_correction
        
        # Get yaw
        yaw = self.convert_azimuth_to_yaw(corrected_azymuth)
        
        # Convert yaw (radians) to quaternion. Ignore roll and pitch for our use case.
        quat_x, quat_y, quat_z, quat_w = quaternion_from_euler(0, 0, yaw)
        
        # To publish current pose:
        
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = msg.header.stamp
        current_pose_msg.header.frame_id = "map" # for now hardcoded, later reference frame
        # Position (Point)
        current_pose_msg.pose.position.x = pos_utm_x
        current_pose_msg.pose.position.y = pos_utm_y
        current_pose_msg.pose.position.z = msg.height - self.undulation
        # Orientation (Quaternion)
        current_pose_msg.pose.orientation = Quaternion(quat_x, quat_y, quat_z, quat_w)
        # Publish
        self.current_pose_pub.publish(current_pose_msg)
        

    # Converts azimuth to yaw. 
    # - Azimuth is CW angle from the North (y-axis) in radians. 
    # - Yaw is CCW angle from the East (x-axis) in radians.
    def convert_azimuth_to_yaw(self, azimuth):        
        yaw = -azimuth + math.pi/2
        # Clamp within 0 to 2 pi
        if yaw > 2 * math.pi:
            yaw = yaw - 2 * math.pi
        elif yaw < 0:
            yaw += 2 * math.pi

        return yaw
    

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()
