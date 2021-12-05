
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <vector>
#include <curand.h>
#include "include/cycleTimer.h"

#include "include/refRenderer.h"
#define BLOCKSIZE 256

struct Constants {
    int* cudaGrid;
    int* cudaParticleLocations;
    float* cudaImage;

    int imgWidth;
    int imgHeight;
    int gridScale;
};

__constant__ Constants params;

RefRenderer::RefRenderer(Pfilter* f) {

    filter = f;

    Grid* grid = filter->get_grid();
    int w = grid->width;
    int h = grid->height;
    int scale = filter->get_grid_scale();
    image = new Image(w * scale, h * scale);

    cudaMalloc(&cudaGrid, sizeof(int) * w * h);
    cudaMalloc(&cudaImage, sizeof(float) * image->width * image->height * 4);
    cudaMalloc(&cudaParticleLocations, sizeof(int) * filter->get_num_particles());

    cudaMemcpy(cudaGrid, grid->walls, sizeof(int) * w * h, cudaMemcpyHostToDevice);

    Constants globs;
    globs.cudaGrid = cudaGrid;
    globs.cudaImage = cudaImage;
    globs.cudaParticleLocations = cudaParticleLocations;
    globs.imgWidth = image->width;
    globs.imgHeight = image->height;
    globs.gridScale = scale;

    cudaMemcpyToSymbol(params, &globs, sizeof(Constants));
}

RefRenderer::~RefRenderer() {
    if (image) {
        delete image;
    }
}

const Image* RefRenderer::get_image() {
    cudaMemcpy(image->data,
               cudaImage,
               sizeof(float) * 4 * image->width * image->height,
               cudaMemcpyDeviceToHost);
    return image;
}

Pfilter* RefRenderer::get_filter() {
    return filter;
}

__device__ int device_to_grid(int imgWidth, int i, int gridScale) {

    int gridWidth = imgWidth / gridScale;

    int x = i % imgWidth;
    int y = i / imgWidth;

    int gx = x / gridScale; 
    int gy = y / gridScale;

    return gx + gy*gridWidth;

}

__global__ void kernelClearImage() {

    int imageX = blockIdx.x * blockDim.x + threadIdx.x;
    int imageY = blockIdx.y * blockDim.y + threadIdx.y;

    int imgWidth = params.imgWidth;
    int imgHeight = params.imgHeight;

    if (imageX >= imgWidth || imageY >= imgHeight) return;

    int offset = 4 * (imageY * imgWidth + imageX);

    *(float4*)(&params.cudaImage[offset]) = make_float4(1,1,1,1);

    int gridScale = params.gridScale;
    int* cudaGrid = params.cudaGrid;
    int g = device_to_grid(imgWidth, offset/4, gridScale);
    
    if (cudaGrid[g] == 1) {
        *(float4*)(&params.cudaImage[offset]) = make_float4(0,0,0,0);
    }

}

void RefRenderer::clearImage() {

    dim3 blockDim(16, 16, 1);
    dim3 gridDim((image->width + blockDim.x - 1) / blockDim.x
                , (image->height + blockDim.y - 1) / blockDim.y);
    kernelClearImage<<<gridDim, blockDim>>>();
    cudaDeviceSynchronize();

}

void RefRenderer::advanceAnimation() {
    filter->update();
}

__global__ void kernelRender(int particleScale, int numParticles) {

    int threadId = threadIdx.x + blockDim.x * blockIdx.x;
    int imgWidth = params.imgWidth;

    float* data = params.cudaImage;

    // 1) draws the particles
    if (threadId < numParticles) {
        int pLoc = params.cudaParticleLocations[threadId];
        printf("ploc is %d\n", pLoc);
        int loc = 4 * pLoc;
        for (int y=0; y<particleScale; y++) {
            for (int x=0; x<particleScale; x++) {
                int pos = loc + 4 * (y * imgWidth) + 4 * x;
                data[pos] = 1;
                data[pos+1] = 0;
                data[pos+2] = 0;
                data[pos+3] = 1;
            }
        }
    }


}

void RefRenderer::render() {

    int particleScale = filter->get_particle_scale();
    int numParticles = filter->get_num_particles();

    dim3 blockDim(BLOCKSIZE);
    dim3 gridDim((numParticles + BLOCKSIZE - 1) / BLOCKSIZE);
    kernelRender<<<gridDim, blockDim>>>(particleScale, numParticles);
    cudaDeviceSynchronize();

    /*
    int goal = filter->get_goal() * 4;
    int goalScale = filter->get_goal_scale();
    int imgWidth = image->width;
    float* data = image->data;
    printf("goal is %d\n", goal/4);
    printf("goalScale is %d\n", goalScale);
    printf("imgWidth is %d\n", imgWidth);

    // 2) draw location of goal
    int goalSize = goalScale * particleScale;
    for (int y=-goalSize/2; y<goalSize/2; y++) {
        for (int x=-goalSize/2; x<goalSize/2; x++) {
            int pos = goal + 4 * (y * imgWidth) + 4 * x;
            data[pos] = 1;
            data[pos+1] = 0;
            data[pos+2] = 0;
            data[pos+3] = 1;
        }
    }

    // 3) draw robot
    Robot* robot = filter->get_robot();
    int robotScale = robot->get_scale() * particleScale;
    int robopos = robot->get_pos() * 4;
    for (int y=-robotScale/2; y<robotScale/2; y++) {
        for (int x=-robotScale/2; x<robotScale/2; x++) {
            int pos = robopos + 4 * (y * imgWidth) + 4 * x;
            data[pos] = 0;
            data[pos+1] = 0;
            data[pos+2] = 1;
            data[pos+3] = 1;
        }
    }
    */

}

void RefRenderer::dumpWalls(const char* filename) {
    FILE* output = fopen(filename, "w");
    
    Grid* grid = filter->get_grid();
    int w = grid->width;
    int h = grid->height;
    fprintf(output, "%d %d\n", w, h);

    for (int r=0; r<h; r++) {
        for (int c=0; c<w; c++) {
            fprintf(output, "%d", grid->walls[r*w+c]);
        }
        fprintf(output, "\n");
    }

}


