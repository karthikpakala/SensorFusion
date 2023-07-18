#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

namespace CameraProcessing
{
    class Camera
    {
        public:
        // Copy constructor
        // Copy Assignment Conostructor
        // Move Constructor
        // Move Assignment Constructor
        // Destructor

        // Default Constructor
        Camera(cv::Mat & inputImage); // Default constructor
        Camera(const Camera& cameraObject); // Copy constructor
        Camera &operator=(const Camera cameraObject); // Copy assignment constructor
        Camera (Camera &&cameraObject); // Move constructor
        Camera &operator=(Camera &&cameraObject); // Move assignment constructor
        ~Camera(); // Destructor


        // Step 1: Input image into memory
        void inputCameraImage();


        private:
        
            cv::Mat inputImage;


    };
}



// Step 2: Convert image into a grey scale image
// Step 3: Detect key points and descriptors
// Step 4: Track Objects 
// Step 5: Estimate vehicle velocity

#endif