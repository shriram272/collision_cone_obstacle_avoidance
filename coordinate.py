# #!/usr/bin/env python
# import rospy
# from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import Marker, MarkerArray
# import datetime

# def grid_to_world(index, width, resolution, origin):
#     # Convert index to 2D grid coordinates
#     x_grid = index % width
#     y_grid = index // width

#     # Convert grid coordinates to world coordinates
#     x_world = (x_grid * resolution) + origin.x
#     y_world = (y_grid * resolution) + origin.y

#     return x_world, y_world

# def callback(data, args):
#     file, marker_pub = args  # Unpack the arguments

#     # Extracting width, resolution, and origin from the OccupancyGrid data
#     width = data.info.width
#     resolution = data.info.resolution
#     origin = data.info.origin.position

#     # Prepare a MarkerArray
#     marker_array = MarkerArray()

#     # Loop through the data array and check for occupied cells
#     for idx, cell in enumerate(data.data):
#         if cell == 100:  # Adjust this threshold based on your requirements
#             x_world, y_world = grid_to_world(idx, width, resolution, origin)
#             file.write(f"{x_world}, {y_world}\n")
#             rospy.loginfo("Obstacle at x: {}, y: {}".format(x_world, y_world))

#             # Create a marker for each obstacle
#             marker = Marker()
#             marker.header.frame_id = "map"  # Or your specific frame
#             marker.header.stamp = rospy.Time.now()
#             marker.ns = "obstacles"
#             marker.id = idx
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.pose.position.x = x_world
#             marker.pose.position.y = y_world
#             marker.pose.position.z = 0  # Assuming obstacles are on the ground
#             marker.pose.orientation.w = 1.0
#             marker.scale.x = 0.1  # Size of the marker
#             marker.scale.y = 0.1
#             marker.scale.z = 0.1
#             marker.color.a = 1.5  # Opacity
#             marker.color.r = 5.0  # Red color
#             marker.color.g = 0.0
#             marker.color.b = 0.0
#             marker.lifetime = rospy.Duration()  # Permanent marker


#             marker_array.markers.append(marker)

#     # Publish the marker array
#     marker_pub.publish(marker_array)

# def listener():
#     # Initialize the ROS node
#     rospy.init_node('costmap_obstacle_listener', anonymous=True)

#     # Open a file to write coordinates
#     filename = "obstacle_coordinates_" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".txt"
#     with open(filename, "w") as file:
#         # Create a marker publisher
#         marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

#         # Subscribe to the OccupancyGrid topic
#         rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback, callback_args=(file, marker_pub))

#         # Keep the program alive
#         rospy.spin()

# if __name__ == '__main__':
#     listener()

# #!/usr/bin/env python
# import rospy
# from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import Marker, MarkerArray
# import numpy as np
# from sklearn.cluster import DBSCAN
# from scipy.spatial import ConvexHull
# import math
# from geometry_msgs.msg import Point
# from std_msgs.msg import Float64

# def grid_to_world(index, width, resolution, origin):
#     # Convert index to 2D grid coordinates
#     x_grid = index % width
#     y_grid = index // width
#     # Convert grid coordinates to world coordinates
#     x_world = (x_grid * resolution) + origin.x
#     y_world = (y_grid * resolution) + origin.y
#     return x_world, y_world

# def calculate_convex_hull_area(points):
#     if len(points) < 3:
#         return 0  # Not enough points to form a convex hull
#     hull = ConvexHull(points)
#     return hull.volume  # For 2D, volume is the area

# def calculate_circle_properties(points):
#     area = calculate_convex_hull_area(points)
#     radius = math.sqrt(area / math.pi)
#     centroid = np.mean(points, axis=0)
#     return centroid, radius, area

# def callback(data, marker_pub):
#     # Extracting width, resolution, and origin from the OccupancyGrid data
#     width = data.info.width
#     resolution = data.info.resolution
#     origin = data.info.origin.position
#     # Collecting obstacle coordinates
#     obstacle_coords = []
#     for idx, cell in enumerate(data.data):
#         if cell > 70:  # Threshold for obstacles
#             obstacle_coords.append(grid_to_world(idx, width, resolution, origin))
#     # Convert to NumPy array for processing
#     obstacle_coords_np = np.array(obstacle_coords)
#     # DBSCAN clustering
#     db = DBSCAN(eps=0.2, min_samples=3)  # Adjust these parameters as necessary
#     db.fit(obstacle_coords_np)
#     labels = db.labels_
#     # Create and publish markers
#     marker_array = MarkerArray()
#     # Process each cluster
#     unique_labels = set(labels)
#     for label in unique_labels:
#         if label == -1:
#             continue  # Skip noise
#         # Extract the coordinates of the current cluster
#         points = obstacle_coords_np[labels == label]
#         centroid, radius, area = calculate_circle_properties(points)
#         rospy.loginfo('Cluster {} has an area of {:.2f}, approximates a circle with radius {:.2f} centered at ({:.2f}, {:.2f})'.format(label, area, radius, centroid[0], centroid[1]))
#         # Create a marker for the cluster's centroid
#         marker = Marker()
#         marker.header.frame_id = "map"
#         marker.header.stamp = rospy.Time.now()
#         marker.ns = "cluster_centroids"
#         marker.id = label
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = centroid[0]
#         marker.pose.position.y = centroid[1]
#         marker.pose.position.z = 0
#         marker.pose.orientation.w = 1.0
#         marker.scale.x = radius * 2  # Diameter
#         marker.scale.y = radius * 2
#         marker.scale.z = 0.1  
#         marker.color.a = 0.5  # Semi-transparent
#         marker.color.r = 0.0
#         marker.color.g = 1.0  # Green color for centroids
#         marker.color.b = 0.0
#         marker.lifetime = rospy.Duration()
#         marker_array.markers.append(marker)
#     # Publish the marker array
#     marker_pub.publish(marker_array)

# def listener():
#     rospy.init_node('costmap_obstacle_listener', anonymous=True)
#     marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
#     rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback, marker_pub)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()








#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull
import math
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

def grid_to_world(index, width, resolution, origin):
    x_grid = index % width
    y_grid = index // width
    x_world = (x_grid * resolution) + origin.x
    y_world = (y_grid * resolution) + origin.y
    return x_world, y_world

def calculate_convex_hull_area(points):
    if len(points) < 3:
        return 0
    hull = ConvexHull(points)
    return hull.volume

def calculate_circle_properties(points):
    area = calculate_convex_hull_area(points)
    radius = math.sqrt(area / math.pi)
    centroid = np.mean(points, axis=0)
    return centroid, radius, area

def callback(data, args):
    marker_pub, centroid_pub, radius_pub = args
    width = data.info.width
    resolution = data.info.resolution
    origin = data.info.origin.position

    obstacle_coords = []
    for idx, cell in enumerate(data.data):
        if cell > 70:
            obstacle_coords.append(grid_to_world(idx, width, resolution, origin))
    obstacle_coords_np = np.array(obstacle_coords)

    db = DBSCAN(eps=0.2, min_samples=3)
    db.fit(obstacle_coords_np)
    labels = db.labels_

    marker_array = MarkerArray()
    for label in set(labels):
        if label == -1:
            continue
        points = obstacle_coords_np[labels == label]
        centroid, radius, area = calculate_circle_properties(points)
        rospy.loginfo('Cluster {} has an area of {:.2f}, approximates a circle with radius {:.2f} centered at ({:.2f}, {:.2f})'.format(label, area, radius, centroid[0], centroid[1]))

        centroid_msg = Point(x=centroid[0], y=centroid[1], z=0)
        radius_msg = Float64(data=radius)

        centroid_pub.publish(centroid_msg)
        radius_pub.publish(radius_msg)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "cluster_centroids"
        marker.id = label
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2  # Diameter
        marker.scale.y = radius * 2
        marker.scale.z = 0.1  
        marker.color.a = 0.5  # Semi-transparent
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green color for centroids
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration()
        marker_array.markers.append(marker)
    # Publish the marker array
    marker_pub.publish(marker_array)

        # Assuming marker creation code remains the same and uses centroid and radius for visualization

    # marker_pub.publish(marker_array) # Uncomment if marker publishing code is added

def listener():
    rospy.init_node('costmap_obstacle_listener', anonymous=True)

    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    centroid_pub = rospy.Publisher('/cluster_centroid', Point, queue_size=10)
    radius_pub = rospy.Publisher('/cluster_radius', Float64, queue_size=10)

    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback, (marker_pub, centroid_pub, radius_pub))

    rospy.spin()

if __name__ == '__main__':
    listener()
