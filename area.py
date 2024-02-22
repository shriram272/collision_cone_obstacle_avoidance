#!/usr/bin/env python

import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

# Function to calculate the area of the convex hull
def convex_hull_area(points):
    hull = ConvexHull(points)
    return hull.volume

# Load the coordinates from the file written by the ROS node
def load_coordinates(file_path):
    with open(file_path, 'r') as file:
        coordinates = np.array([list(map(float, line.strip().split(','))) for line in file])
    return coordinates

# Run DBSCAN on the coordinates and calculate the area of each cluster
def cluster_and_calculate_areas(coordinates, eps=0.5, min_samples=5):
    # DBSCAN clustering
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(coordinates)
    labels = db.labels_
    
    unique_labels = set(labels)
    areas = []

    for k in unique_labels:
        if k != -1:  # -1 is for noise points
            class_member_mask = (labels == k)
            cluster_points = coordinates[class_member_mask]
            area = convex_hull_area(cluster_points)
            areas.append((k, area))
    
    return areas

# Replace 'path_to_file.txt' with the actual path to the file containing the coordinates
file_path = '/home/shriram/test_turtlebot/src/turtlebot3_simulations/global_planner/obstacle_coordinates_20240125-162411.txt'
coordinates = load_coordinates(file_path)
areas = cluster_and_calculate_areas(coordinates)

# Print the area of each cluster
for cluster_id, area in areas:
    print(f"Cluster {cluster_id} has an area of {area:.2f} square units")
