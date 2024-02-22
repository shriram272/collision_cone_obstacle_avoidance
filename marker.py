#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray

def read_points_from_file(filename):
    points = []
    with open(filename, 'r') as file:
        for line in file:
            try:
                x_str, y_str = line.strip().split(',')
                x, y = float(x_str), float(y_str)
                points.append((x, y))
            except ValueError:
                rospy.logwarn("Skipping invalid line in file: {}".format(line.strip()))
    return points

def create_markers(points):
    marker_array = MarkerArray()
    for i, (x, y) in enumerate(points):
        marker = Marker()
        marker.header.frame_id = "map"  # Make sure this frame exists in your TF tree
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0  # Assuming points are on the ground
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Adjust size of the marker as needed
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)
    return marker_array

def main():
    rospy.init_node('points_visualizer', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    
    filename = "/home/shriram/test_turtlebot/src/turtlebot3_simulations/global_planner/obstacle_coordinates_20240125-162411.txt"  # Replace with the path to your text file
    points = read_points_from_file(filename)
    marker_array = create_markers(points)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    main()
