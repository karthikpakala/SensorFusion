Project Goals:

1. Load Camera and Lidar data and synchronize the data.

2. process Lidar Data to do the following:
    a. Read the Lidar data
    b. Process Lidar Points to segment various parts of the point clouds into road and objects
    c. Process the point clouds into different objects using clustering algorithms -  use KD-Tree 
    d. Create bounding boxes around the clustered objects and store them as needed. 

3. Process camera data to do the following
    a. Read camera image
    b. Convert to grey scale image
    c. Detect key points and key point descriptors
    d. Track objects using key points/descriptors

4. Combine Key points detection and Lidar BBs to create a sensor fusion model to detect and track objects
    a. Plot the ROI created for the PCL onto the image. 
    b. Identifty which key points fall into the PCL ROI and create a Camera and Lidar based sensor fusion model
    c. Create accuracy metrics to show the detection accuracy of various objects in the scene. 

    Make sure to only use Lidar or Camera data as needed. -- Optional. 

    Write up documentation for code distribution.

    Identify tools to visualize the output. 