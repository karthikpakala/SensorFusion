#ifndef CUDA_TEST_H
#define CUDA_TEST_H

#include <cuda.h>
#include <cudart_platform.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_stdint.h>
#include <thrust/async/sort.h>
#include <thrust/async/transform.h>
#include <vpi/Image.h>
#include <vpi/Stream.h>
#include <iostream>

#include <vpi/OpenCVInterop.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>


//#include <thrust/memory/detail/device_system_resource.h>
using namespace std;
using namespace cv;
using namespace cv::cuda;
//using namespace cv::cuda::device;


class CudaTest
{
    public:
        CudaTest();
        ~CudaTest();
        void test_cuda();
        void test_cuda_vpi();
    private:
        VPIImage vpiImage;
        VPIStream vpiStream;
        cv::cuda::GpuMat gpuMat;
        cv::Mat cpuMat;
        cv::Mat cpuMat2;
        
};

void test_cuda_vpi();


#endif 