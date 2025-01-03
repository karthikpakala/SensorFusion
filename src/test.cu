#include "cuda-test.h"

//using namespace cuda;
//using namespace thrust;
//using namespace std;
//using namespace vpi;


void test_cuda_vpi()
{
    std::cout << "in test_cuda_vpi" << std::endl;
    // vpi::init();

    // // Create a stream
    // cudaStream_t stream;

    // // Create a device buffer
    // device_vector<int> d_vec(10);

    // // Create a host buffer
    // host_vector<int> h_vec(10);

    // // Initialize the host buffer
    // for (int i = 0; i < 10; i++)
    // {
    //     h_vec[i] = 10 - i;
    // }

    // // Copy the host buffer to the device buffer
    // copy(h_vec.begin(), h_vec.end(), d_vec.begin());

    // // Sort the device buffer
    // async::sort(d_vec.begin(), d_vec.end());

    // // Copy the device buffer to the host buffer
    // copy(d_vec.begin(), d_vec.end(), h_vec.begin());

    // // Print the host buffer
    // for (int i = 0; i < 10; i++)
    // {
    //     printf("%d\n", h_vec[i]);
    // }

    // // Destroy the stream
    // cudaStreamDestroy(stream);

    // vpi::shutdown();
}

/*
int main(int argv, char **argc)
{
    // std::cout << "in main.cu" << std::endl;
    // vpi::init();

    // // Create a stream
    // cudaStream_t stream;

    // // Create a device buffer
    // device_vector<int> d_vec(10);

    // // Create a host buffer
    // host_vector<int> h_vec(10);

    // // Initialize the host buffer
    // for (int i = 0; i < 10; i++)
    // {
    //     h_vec[i] = 10 - i;
    // }

    // // Copy the host buffer to the device buffer
    // copy(h_vec.begin(), h_vec.end(), d_vec.begin());

    // // Sort the device buffer
    // async::sort(d_vec.begin(), d_vec.end());

    // // Copy the device buffer to the host buffer
    // copy(d_vec.begin(), d_vec.end(), h_vec.begin());

    // // Print the host buffer
    // for (int i = 0; i < 10; i++)
    // {
    //     printf("%d\n", h_vec[i]);
    // }

    // // Destroy the stream
    // cudaStreamDestroy(stream);

    // vpi::shutdown();

    return 0;
}
*/
