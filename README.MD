Project Goals:

1. Load Camera and Lidar data and synchronize the data.

2. process Lidar Data to do the following:
    1. Read the Lidar data
    2. Process Lidar Points to segment various parts of the point clouds into road and objects
    3. Process the point clouds into different objects using clustering algorithms -  use KD-Tree 
    4. Create bounding boxes around the clustered objects and store them as needed. 

3. Process camera data to do the following
    1. Read camera image
    2. Convert to grey scale image
    3. Detect key points and key point descriptors
    4. Track objects using key points/descriptors

4. Combine Key points detection and Lidar BBs to create a sensor fusion model to detect and track objects
    1. Plot the ROI created for the PCL onto the image. 
    2. Identifty which key points fall into the PCL ROI and create a Camera and Lidar based sensor fusion model
    3. Create accuracy metrics to show the detection accuracy of various objects in the scene. 

    Make sure to only use Lidar or Camera data as needed. -- Optional. 

    Write up documentation for code distribution.

    Identify tools to visualize the output. 