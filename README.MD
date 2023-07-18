Project Goals:

1. Load Camera and Lidar data and synchronize the data. - Done.

2. process Lidar Data to do the following:
    1. Read the Lidar data. 
    2. Filtering Point clouds.
    3. Process Lidar Points to segment various parts of the point clouds into road and objects
    4. Process the point clouds into different objects using clustering algorithms -  use KD-Tree 
    5. Create bounding boxes around the clustered objects and store them as needed. 

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


5. Performance enhancements:
    1. Use smart pointers/move semantics to enable better usage of the memory as well as enable better performance
    2. Update segmentation function to be wayyyy more faster than it is now. look for improvements. 
    3. Update Lidar class to the best possible template class to be able to make it very versatile. 
    4. Use concurrency to enable better usage of resources and improve performance. 
    
    