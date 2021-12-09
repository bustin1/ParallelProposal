#include <thrust/scan.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>

__global__ void init(int* data) {

    int threadId = threadIdx.x + blockIdx.x * blockDim.x;
    data[threadId] = threadId % 10;
    
}

__global__ void print(int* data) {
    int threadId = threadIdx.x + blockDim.x * blockIdx.x;
    printf("data[%d]=%d\n", threadId, data[threadId]);
}

int main () {
    int* data;
    cudaMalloc(&data, sizeof(int) * 100);
    init<<<1,100>>>(data);
    cudaDeviceSynchronize();

    thrust::device_ptr<int> d_ptr(data);
    thrust::inclusive_scan(thrust::device, d_ptr, d_ptr + 100, d_ptr); // in-place scan

    print<<<1,100>>>(data);
    cudaDeviceSynchronize();
    // data is now {4, 5, 5, 7, 9, 10}
    int total;
    cudaMemcpy(&total, data+99, sizeof(int), cudaMemcpyDeviceToHost);
    printf("data[99]=%d\n", total);
}
