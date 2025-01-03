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

//#include <thrust/memory/detail/device_system_resource.h>


void test_cuda_vpi();

#endif 