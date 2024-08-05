Collision Cone Logic

    Distance and Angle Calculation:
        Position Difference: (x_1 - x_0, y_1 - y_0) gives the vector from the robot to the centroid.
        Distance (d): Calculated using Euclidean distance.
        Angle (x): Computed using atan2 for the vector direction.

    Collision Cone Boundaries:
        Argument for asin: 2 * r / d is used to calculate the angle boundaries theta_1 and theta_2.
        alpha_1 and alpha_2: These are the angles defining the boundaries of the collision cone, adjusted based on the robot's velocities and angle beta (which is zero in this case).

    Visualization:
        Markers: Two markers are published to visualize the collision cone. These markers are arrows with orientations set by alpha_1 and alpha_2.


 
 RESULTS -       


        

https://github.com/user-attachments/assets/2d155738-2c50-4b97-a90c-df7931956251

