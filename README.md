# SLAM_planner
A SLAM technique for Robot planning using MATLAB and Navigation ToolBox

In this project, the SLAM technique and its integration with other methods are used to make a mobile robot interpret the environment, create a map and navigate through the map from an initial point and return to that initial point.

To do this I used Navigation Toolbox in MATLAB to simulate the program.

For the simulation in MATLAB, a certain time is defined with a test frequency of 20Hz for the sensors, in which all the calculations corresponding to the system are carried out, including the use of the lidar SLAM tool that MATLAB has in their system, which allowed the map to be rebuilded based on scans, as well as estimating a pose based on these.

The loop begins by capturing the LiDAR to perform the SLAM, then the control is updated to obtain the desired wheel speeds, and finally the IMU readings are obtained to calculate the odometry, which we obtain by integrating the accelerometer readings. and the gyroscope.

![Screenshot 2023-05-02 at 21 03 13](https://user-images.githubusercontent.com/22161124/235762302-4545f1ca-9e4d-4143-a025-a45e4d5037c2.png)

Once the information from the sensors is obtained and processed, the Cartesian data obtained from the LiDAR is processed, the points are displaced and rotated in relation to the current pose of the robot, obtained through the lidarSLAM object.

![Voronoi](https://user-images.githubusercontent.com/22161124/235762527-4a23a474-5fc3-413b-84c6-ece326e87c10.png)

RESULTS:

During the design process, multiple problems were discovered with the foundation of the algorithm, where the selection of the points itself did not correspond to the geometry of the maps. The processing of the data from a LiDAR creates in the corresponding Voronoi diagram an agglomeration in a local minimum of the map, creating an agglomeration of greater probability where it tends towards the direction of the local minimum.

![voronoi2](https://user-images.githubusercontent.com/22161124/235766285-9c556588-c5e6-4c0a-b6f6-5a1f469af0f1.png)

Caution should be exercised with the use of Voronoi diagrams:
This is because filtering is required, which takes processing time and random sampling, which generates a solution that makes it easier to move the robot in a random direction. Regardless, it must be taken into account that clusters of points within the diagram make the probability of choosing one direction or the other are not directly those of a uniform probability distribution, but in accordance with the geometry of the environment. This reduces the risk of collisions within a static environment.
