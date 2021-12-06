
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include <curand.h>
#include <curand_kernel.h>
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>
#include <thrust/functional.h>
#include "include/cycleTimer.h"


#include "include/refRenderer.h"
#include "include/particleFilter.h"

#define BLOCKSIZE 256
#define SCAN_BLOCK_DIM BLOCKSIZE
#include "exclusiveScan.cu_inl"

// TODO: robot explore unvisited states
// TODO: once particles converge, guess best location
// TODO: store best paths in a table?
// TODO: particleGuess hit wall => resample
// TODO: for the robot, once the robot remembers too many states, refresh unvisted

struct PConstants {
    int* particleLocations;
    float* particleOrientations;
    float* weights;
    int* rays;
    Robot* robot;
    int *closed;
    int *open;
    curandState* states;

    int imgWidth;
    int gridScale;
    int num_open;
    int num_closed;
    int numParticles;
    int maxRayLen;
};

__constant__ PConstants pparams;

struct Constants {
    int* cudaGrid;
    float* cudaImage;

    int imgWidth;
    int imgHeight;
    int gridScale;
};

__constant__ Constants params;

__global__ void setup_kernel(unsigned int seed, curandState_t* states) {

  /* we have to initialize the state */
  curand_init(seed, /* the seed can be the same for each core, here we pass the time in from the CPU */
              blockIdx.x, /* the sequence number should be different for each core (unless you want all
                             cores to get the same sequence of numbers for some reason - use thread id! */
              0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
              &states[blockIdx.x]);
}

__device__ int device_to_grid(int imgWidth, int i, int gridScale) {

    int gridWidth = imgWidth / gridScale;

    int x = i % imgWidth;
    int y = i / imgWidth;

    int gx = x / gridScale; 
    int gy = y / gridScale;

    return gx + gy*gridWidth;

}


Pfilter::Pfilter(Robot* r, Grid* g, 
                int numParticles, 
                const int gridScale, 
                const int particleScale,
                const int numRays,
                const int DEBUG)  
                : numParticles (numParticles),
                gridScale(gridScale),
                particleScale(particleScale),
                numRays (numRays),
                DEBUG (DEBUG),
                maxNumParticles (numParticles),
                imgWidth (g->width * gridScale),
                sample_freq(5),
                maxRayLen (10 * gridScale) {


    cudaMalloc(&particleLocations, sizeof(int) * numParticles);
    cudaMalloc(&particleOrientations, sizeof(float) * numParticles);
    cudaMalloc(&rays, sizeof(float) * numParticles * numRays);
    cudaMalloc(&weights, sizeof(float) * numParticles);
    cudaMalloc(&states, sizeof(curandState) * numParticles);

    robot = r;
    grid = g;
    i_am_speed = false;

    // sample_counter = 0;
    // sampling_pos_variance = gridScale / 5;

    // 1) init some stuff
    grid->init();
    if (DEBUG) {
        robot->init(numRays, grid, gridScale, -1);
    } else {
        robot->init(numRays, grid, gridScale, rand_location());
    }
    goalScale = robot->get_scale();

    cudaMalloc(&open, sizeof(int) * grid->num_open);
    cudaMalloc(&closed, sizeof(int) * grid->num_closed);

    cudaMemcpy(open, grid->open, sizeof(int) * grid->num_open, cudaMemcpyHostToDevice);
    cudaMemcpy(closed, grid->closed, sizeof(int) * grid->num_closed, cudaMemcpyHostToDevice);

    PConstants globs;
    globs.particleLocations = particleLocations;
    globs.particleOrientations = particleOrientations;
    globs.rays = rays;
    globs.imgWidth = imgWidth;
    globs.gridScale = gridScale;
    globs.open = open;
    globs.closed = closed;
    globs.num_closed = grid->num_closed;
    globs.num_open = grid->num_open;
    globs.weights = weights;
    globs.states = states;
    globs.numParticles = numParticles;
    globs.maxRayLen = maxRayLen;

    cudaMemcpyToSymbol(pparams, &globs, sizeof(PConstants));

    // printf("=====================>\n");
    setup_kernel<<<numParticles, 1>>>(time(NULL), globs.states);


    // 2) randomly assign locations to particles in the grid
    uniform_sample();

    // 3) randomize the goal location
    init();

    // 4) reweight to prevent seg fault on first transition
    // reweight(); 
    

}

__global__ void kernel_reset_weights() {

    int threadId = blockDim.x * blockIdx.x + threadIdx.x;
    pparams.weights[threadId] = 1;

}

void Pfilter::reset_weights() {

    dim3 blockDim(BLOCKSIZE);
    dim3 gridDim((numParticles + BLOCKSIZE - 1) / BLOCKSIZE);

    kernel_reset_weights<<<gridDim, blockDim>>>();

}

int Pfilter::rand_location() {

    int random = rand_num() * grid->num_open;
    int loc = grid->open[random];
    int j = to_img(grid->width, loc, gridScale);

    int offsetX = rand_num() * (gridScale-2) + 1;
    int offsetY = rand_num() * (gridScale-2) + 1;

    int x = get_x(j, imgWidth) + offsetY;
    int y = get_y(j, imgWidth) + offsetX;

    return y*imgWidth + x;

}

__device__ int device_get_x(int i, int imgWidth) {
    return i % imgWidth;
}

__device__ int device_get_y(int i, int imgWidth) {
    return i / imgWidth;
}

__device__ double device_rand_num(int threadId) {
    double rand = curand_uniform_double(&pparams.states[threadId]);
    return rand;
}

__device__ float device_rand_angle(int threadId) {
    return 2.0f * PI * device_rand_num(threadId);
}


__device__ int device_to_img(int gridWidth, int i, int gridScale) {

    int imgWidth = gridWidth * gridScale;

    int x = i % gridWidth;
    int y = i / gridWidth;

    int ix = x * gridScale;
    int iy = y * gridScale;

    return iy*imgWidth + ix;

}


void Pfilter::init() {

    if (DEBUG) {
        // bottom right
        goal = (imgWidth - 1.5f*gridScale) + (gridScale * 2.f) * imgWidth;
    } else {
        goal = rand_location();
    }

}

__device__ int device_rand_location(int threadId) {

    int num_open = pparams.num_open;
    int gridScale = pparams.gridScale;
    int* open = pparams.open;
    int imgWidth = pparams.imgWidth;
    int gridWidth = pparams.imgWidth / pparams.gridScale;

    int random = device_rand_num(threadId) * num_open;
    int loc = open[random];
    int j = device_to_img(gridWidth, loc, gridScale);

    int offsetX = device_rand_num(threadId) * (gridScale-2) + 1;
    int offsetY = device_rand_num(threadId) * (gridScale-2) + 1;

    int x = device_get_x(j, imgWidth) + offsetY;
    int y = device_get_y(j, imgWidth) + offsetX;

    return y*imgWidth + x;

}


__global__ void kernel_uniform_sample() {
    int threadId = blockDim.x * blockIdx.x + threadIdx.x;
    int* particleLocations = pparams.particleLocations;
    float* particleOrientations = pparams.particleOrientations;
    int numParticles = pparams.numParticles;

    if (threadId < numParticles) {
        particleLocations[threadId] = device_rand_location(threadId);
        // printf("new loc just dropped %d\n", particleLocations[threadId]);
        particleOrientations[threadId] = device_rand_angle(threadId);
    }
    // printf("particlocations[%d] = %d and ", threadId, particleLocations[threadId]);
    // printf("particoreitation[%d] = %f\n", threadId, particleOrientations[threadId]);

}

void Pfilter::uniform_sample() {

    dim3 blockDim(BLOCKSIZE);
    dim3 gridDim((numParticles + BLOCKSIZE - 1) / BLOCKSIZE);

    kernel_uniform_sample<<<gridDim, blockDim>>>();
    cudaDeviceSynchronize();

    // 2) Choose random best particle
    particleGuess = rand_num() * numParticles;
    // printf("particle guesss is %d\n", particleGuess);

    // 3) Reset all particle weights
    reset_weights();

}

Pfilter::~Pfilter() {
    if (particleLocations) {
        delete[] particleLocations;
    }
    if (particleOrientations) {
        delete[] particleOrientations;
    }
}


// getters and setters
Grid* Pfilter::get_grid() {
    return grid;
}

int Pfilter::get_goal_scale() {
    return goalScale;
}

int Pfilter::get_num_particles() {
    return numParticles;
}

int* Pfilter::get_particle_locations() {
    return particleLocations;
}

int Pfilter::get_grid_scale() {
    return gridScale;
}

int Pfilter::get_particle_scale() {
    return particleScale;
}

int* Pfilter::get_rays() {
    return rays;
}

int Pfilter::get_num_rays() {
    return numRays;
}

int Pfilter::get_goal() {
    return goal;
}

int Pfilter::get_best_particle() {
    return particleGuess;
}

Robot* Pfilter::get_robot() {
    return robot;
}



// https://gamedev.stackexchange.com/questions/82710/smooth-path-direct-movement
void Pfilter::find_next_step(float& dtheta, float& speed, float theta, int start) {

    // 1) bfs to find shortest path
    std::vector<int> best_path = 
        bfs(to_grid(this->imgWidth, start, this->gridScale), 
        to_grid(this->imgWidth, this->goal, this->gridScale), this->grid);

    // 2) haven't hit the goal state, find direction and speed to go
    if (best_path.size() >= 2) { 

        int nextGrid = to_img(this->grid->width, best_path[1], this->gridScale);
        float yawRate = this->robot->get_yaw_rate();
        
        int nextX = get_x(nextGrid, this->imgWidth) + this->gridScale/2;
        int nextY = get_y(nextGrid, this->imgWidth) + this->gridScale/2;
        int startX = get_x(start, this->imgWidth);
        int startY = get_y(start, this->imgWidth);

        double angle_dir = atan2((double) (nextY-startY), (double) (nextX-startX));
        double turn_angle = angle_dir - theta;

        // clamp the turning speed
        dtheta = clamp(turn_angle, yawRate);

        // TODO: Fine tune the variance
//        dtheta = gaussian1d(dtheta, .1); 

        // TODO: Fine the tune the speed
        speed = 1.f; 

    } else {
        speed = 1.f;
        dtheta = rand_angle();
    }

}

__device__ double device_double_mod(double value, double threshold) {
    if (value > threshold) value -= threshold;
    if (value < -threshold) value += threshold;
    return value;
}

__device__ double device_gaussian1d(float mean, float variance, int threadId) {

    double v1 = device_rand_num(threadId);
    double v2 = device_rand_num(threadId);

    double newx = cos(2*PI*v2) * sqrt(-2*log(v1));

    newx = newx * variance + mean;

    return newx;
}

__device__ double device_gaussian2d(int mean, int imgWidth, double variance, int threadId) {

    int x = mean % imgWidth;
    int y = mean / imgWidth;
    
    double newx = device_gaussian1d(x, variance, threadId);
    double newy = device_gaussian1d(y, variance, threadId);

    return newx + round(newy) * imgWidth;
}

__global__ void kernelTransition(float dtheta, float speed, int numParticles) {

    int threadId = threadIdx.x + blockDim.x * blockIdx.x;

    float* particleOrientations = pparams.particleOrientations;
    int* particleLocations = pparams.particleLocations;
    int imgWidth = pparams.imgWidth;
    int gridScale = pparams.gridScale;

    // 2) move each particle
    if (threadId < numParticles) {

        float candidate_angle = 
            device_double_mod(particleOrientations[threadId] + dtheta, 2 * PI);

        float dirX = speed * cos(candidate_angle);
        float dirY = speed * sin(candidate_angle);
        
        int px = device_get_x(particleLocations[threadId], imgWidth);
        int py = device_get_y(particleLocations[threadId], imgWidth);

        int candidate_x = round(px + dirX);
        int candidate_y = round(py + dirY);

        int candidate_pos = candidate_x + imgWidth*candidate_y;

        particleOrientations[threadId] = candidate_angle;

        int g = device_to_grid(imgWidth, candidate_pos, gridScale);

        if (params.cudaGrid[g] == 0) {
            particleLocations[threadId] = candidate_pos;
        } else { 
            // randomize the angle for the particle to move in
            particleOrientations[threadId] = device_gaussian1d(candidate_angle, .25, threadId);
        }
    }

}

void Pfilter::transition() {

    float dtheta;
    float speed;

    // 1) robot move
    if (i_am_speed) {
        float orientation;
        int start;
        cudaMemcpy(&orientation, &particleOrientations[particleGuess], sizeof(int), cudaMemcpyDeviceToHost);
        cudaMemcpy(&start, &particleLocations[particleGuess], sizeof(int), cudaMemcpyDeviceToHost);
        find_next_step(dtheta, speed, orientation, start); // hmm particleOrientations
        robot->move(dtheta, speed); 
    } else {
        // Robot will do it's own stuff
        robot->move_greedy(dtheta, speed); 
    }


    dim3 blockDim(BLOCKSIZE);
    dim3 gridDim((numParticles + BLOCKSIZE - 1) / BLOCKSIZE);
    kernelTransition<<<gridDim, blockDim>>>(dtheta, speed, numParticles);
    cudaDeviceSynchronize();

}


int Pfilter::getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4) {

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // 1) colinear :(
    if (denom == 0) {
        return -1;
    }

    double numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    double t = numerator / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    if (t > 0 && t < 1 && u > 0) {
        // 2) Found points!
        int x = round(x1 + t * (double)(x2 - x1));
        int y = round(y1 + t * (double)(y2 - y1));
        return x + this->imgWidth*y;
    }
    
    // 3) segments too far apart :(
    return -1;
}

int Pfilter::getClosestIntersection(int pLoc, double angle) {

    int num_closed = this->grid->num_closed;
    int* closed = this->grid->closed;

    double dir_x = cos(angle);
    double dir_y = sin(angle);

    // 1) particle locations
    int x3 = get_x(pLoc, this->imgWidth);
    int y3 = get_y(pLoc, this->imgWidth);
    int x4 = x3 + maxRayLen * dir_x;
    int y4 = y3 + maxRayLen * dir_y;

    // 2) save best intersection
    int closestX = x4;
    int closestY = y4; 
    int closestDist = pow(maxRayLen, 2);

    for (int i=0; i<num_closed; i++) {

        int wLoc = closed[i];

        // WEIRDEST BUG: line intersection will wrap since it's a 1d contiguous array
        int x1 = get_x(wLoc, this->grid->width) * this->gridScale; // left_x
        int y1 = get_y(wLoc, this->grid->width) * this->gridScale; // bot_y
        int x2 = x1 + gridScale; // right_x
        int y2 = y1 + gridScale; // top_y

        int a[4][4] = {{x1+1,y1-1,x1+1,y2+1}, 
                      {x1-1,y2-1,x2+1,y2-1}, 
                      {x2-1,y2+1,x2-1,y1-1},
                      {x2+1,y1+1,x1-1,y1+1}};

        for (int j=0; j<4; j++) {

            int intersection = this->getSingleIntersection(a[j][0], a[j][1], 
                                                            a[j][2], a[j][3], 
                                                            x3, y3, x4, y4);

            if (intersection >= 0) {

                int x = get_x(intersection, this->imgWidth);
                int y = get_y(intersection, this->imgWidth);

                int sx = x3-x;
                int sy = y3-y;
                int dist = sx*sx + sy*sy;

                if (dist < closestDist) {
                    closestDist = dist;
                    closestX = x;
                    closestY = y;
                }
            }
        }

    }

    return closestY * this->imgWidth + closestX;
}

void Pfilter::firerays(int loc, int* ptr, double angle_bias) {

    // 1) assume 360 degree view
    const double dtheta = 2 * PI / this->numRays;
    for (int i=0; i<this->numRays; i++) {
        double angle = i * dtheta + angle_bias;
        int coord = this->getClosestIntersection(loc, angle);
        *ptr = coord;
        ptr++;
    }
     
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

__device__ int device_getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4) {

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // 1) colinear :(
    if (denom == 0) {
        return -1;
    }

    double numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    double t = numerator / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    if (t > 0 && t < 1 && u > 0) {
        // 2) Found points!
        int x = round(x1 + t * (double)(x2 - x1));
        int y = round(y1 + t * (double)(y2 - y1));
        int imgWidth = pparams.imgWidth;
        return x + imgWidth*y;
    }
    
    // 3) segments too far apart :(
    return -1;
}

__device__ int device_getClosestIntersection(int pLoc, double angle) {

    int num_closed = pparams.num_closed;
    int* closed = pparams.closed;
    int imgWidth = pparams.imgWidth;
    int maxRayLen = pparams.maxRayLen;
    int gridWidth = pparams.imgWidth / pparams.gridScale;
    int gridScale = pparams.gridScale;

    double dir_x = cos(angle);
    double dir_y = sin(angle);

    // 1) particle locations
    int x3 = device_get_x(pLoc, imgWidth);
    int y3 = device_get_y(pLoc, imgWidth);
    int x4 = x3 + maxRayLen * dir_x;
    int y4 = y3 + maxRayLen * dir_y;

    // 2) save best intersection
    int closestX = x4;
    int closestY = y4; 
    int closestDist = maxRayLen * maxRayLen;

    for (int i=0; i<num_closed; i++) {

        int wLoc = closed[i];

        // WEIRDEST BUG: line intersection will wrap since it's a 1d contiguous array
        int x1 = device_get_x(wLoc, gridWidth) * gridScale; // left_x
        int y1 = device_get_y(wLoc, gridWidth) * gridScale; // bot_y
        int x2 = x1 + gridScale; // right_x
        int y2 = y1 + gridScale; // top_y

        int a[4][4] = {{x1+1,y1-1,x1+1,y2+1}, 
                      {x1-1,y2-1,x2+1,y2-1}, 
                      {x2-1,y2+1,x2-1,y1-1},
                      {x2+1,y1+1,x1-1,y1+1}};

        for (int j=0; j<4; j++) {

            int intersection = device_getSingleIntersection(a[j][0], a[j][1], 
                                                            a[j][2], a[j][3], 
                                                            x3, y3, x4, y4);

            if (intersection >= 0) {

                int x = device_get_x(intersection, imgWidth);
                int y = device_get_y(intersection, imgWidth);

                int sx = x3-x;
                int sy = y3-y;
                int dist = sx*sx + sy*sy;

                if (dist < closestDist) {
                    closestDist = dist;
                    closestX = x;
                    closestY = y;
                }
            }
        }

    }

    return closestY * imgWidth + closestX;
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

__device__ float device_dist2d(int p1, int p2, int imgWidth) {

    int x1 = p1 % imgWidth;
    int y1 = p1 / imgWidth;
    
    int x2 = p2 % imgWidth;
    int y2 = p2 / imgWidth;

    return sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

}

__global__ void kernelReweight(float* roboRayLen, int numRays, int numParticles, float variance, int sample_freq) {

    int threadId = threadIdx.x + blockDim.x * blockIdx.x;
    int rayId = threadId % numRays;
    int particleId = threadId / numRays;
    int imgWidth = pparams.imgWidth;
    float* particleOrientations = pparams.particleOrientations;
    int* particleLocations = pparams.particleLocations;
    int* rays = pparams.rays;
    float* weights = pparams.weights;

    if (particleId < numParticles) {

        const double dtheta = 2 * PI / numRays;
        int pLoc = particleLocations[particleId];
        float angle_bias = particleOrientations[particleId];

        double angle = rayId * dtheta + angle_bias;
        int coord = device_getClosestIntersection(pLoc, angle);
        rays[threadId] = coord;

        // reweight with gaussian distribution
        // 1) we assume that given the particle's location
        // what is the probability that we got an observation
        // 2) this follows ~ e^{-r * p}, where r := robo observation
        // and p := particle's observation

        __shared__ uint sInput[BLOCKSIZE];
        __shared__ uint sOutput[BLOCKSIZE];
        __shared__ uint sScratch[2 * BLOCKSIZE];

        int pRayLen = device_dist2d(coord, pLoc, imgWidth);
        int rayDiff = roboRayLen[rayId] - pRayLen;

        sInput[threadIdx.x] = (uint) powf(rayDiff,2);

        __syncthreads();
        sharedMemExclusiveScan(threadIdx.x, sInput, sOutput, sScratch, SCAN_BLOCK_DIM);
        __syncthreads();

        if (rayId == 0) { // important that numRays == 16
            double exp = sOutput[min(threadIdx.x+numRays, BLOCKSIZE-1)] - sOutput[threadIdx.x];
            weights[particleId] *= pow(E, -exp / variance / sample_freq); // TODO: tune
        }

    }

}


void Pfilter::reweight() {

    // get robo observation
    int* roboRay = robot->get_rays();
    int roboPos = robot->get_pos();
    firerays(roboPos, roboRay, this->robot->get_angle());

    // 1) fire ray for robot
    float* roboRayLen = new float[numRays];
    for (int i=0; i<numRays; i++) {
        float rRayLen = dist2d(roboPos, roboRay[i], this->imgWidth);
        roboRayLen[i] = rRayLen;
    }

    // 2) TODO: is this the best variance for particle weight?
    const float variance = maxRayLen * numRays * 5;

    float* cudaRoboRayLen;
    cudaMalloc(&cudaRoboRayLen, sizeof(float) * numRays);
    dim3 blockDim(BLOCKSIZE);
    dim3 gridDim((numParticles * numRays + BLOCKSIZE - 1) / BLOCKSIZE);
    kernelReweight<<<gridDim, blockDim>>>(cudaRoboRayLen, numRays, numParticles, variance, sample_freq);
    cudaDeviceSynchronize();

    cudaFree(cudaRoboRayLen);

}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
__device__ bool device_confidence(float param, int particleGuess) {

    // 1) test if it is a majority
    int count = 0;
    int imgWidth = pparams.imgWidth;
    int* particleLocations = pparams.particleLocations;
    int gridScale = pparams.gridScale;
    int res = device_to_grid(imgWidth, particleLocations[particleGuess], gridScale);
    int numParticles = pparams.numParticles;
    for (int i=0; i<numParticles; i++) {
        int testLoc = device_to_grid(imgWidth, particleLocations[i], gridScale);
        if (testLoc == res) {
            count++;
        }
    }

    float c = (float) count / (float) numParticles;
//    printf("your confidence is %d/%d = %f\n", count, this->numParticles, c);
    return c > param;

}

__global__ void kernelSample(int* particleGuess, int roboPos, int roboAngle, int* numParticles) {

    // 0) check if we are confident in the robot location
    int imgWidth = pparams.imgWidth;
    int* particleLocations = pparams.particleLocations;
    float* particleOrientations = pparams.particleOrientations;
    float* weights = pparams.weights;
    int gridScale = pparams.gridScale;
    const int n = *numParticles;
    int pguess = *particleGuess;
    int res = device_to_grid(imgWidth, particleLocations[pguess], gridScale);

    if (device_confidence(.5, pguess)) {

        // in real life we would base on sensors but this isn't a robo class
        if (device_to_grid(imgWidth, particleLocations[pguess], gridScale)
                == device_to_grid(imgWidth, roboPos, gridScale)) {
            *particleGuess = 0;
            particleLocations[0] = roboPos;
            particleOrientations[0] = roboAngle; 
            *numParticles = 1;
            printf("wow! ur a cool robot :p You win!\n");
            return;
        }
    }

    // 1) find the best particle (GREEDY)
    float best_weight = 0;
    for (int i=0; i<n; i++) {
        if (best_weight < weights[i]) {
            best_weight = weights[i];
            *particleGuess = i;             
        }

    }

//    printf("best weight is %f\n", best_weight);

    // 2) calculate running weights
    float* running_weights = weights;
    for (int i=1; i<n; i++) {
        running_weights[i] += running_weights[i-1];
    }
    
    // 3) resample every particle
    int newParticleLocations[n];
    float newParticleOrientations[n];
    for (int j=0; j<n; j++) {
        float rand = device_rand_num(0) * running_weights[this->numParticles-1];
        for (int i=0; i<n; i++) {
            if (rand < running_weights[i]) {
                float candidate_random_pos;
                do {
                    // TODO: variance is based on gridScale. Is this a good hueristic?
                    candidate_random_pos = device_gaussian2d(particleLocations[i], imgWidth, 2, 0);
                } while (device_to_grid(imgWidth, candidate_random_pos, gridScale) == 1);
                
                newParticleLocations[j] = round(candidate_random_pos);
                newParticleOrientations[j] = device_gaussian1d(particleOrientations[i], 0.1); // ~6 degrees
                break;
            }
        }
    }

    // 4) update particle locations
    for (int i=0; i<n; i++) {
        particleLocations[i] = newParticleLocations[i];
        particleOrientations[i] = newParticleOrientations[i];
    }

}
*/

bool Pfilter::confidence(float param) {

    // 1) test if it is a majority
    int count = 0;
    int res = to_grid(this->imgWidth, this->particleLocations[particleGuess], this->gridScale);
    for (int i=0; i<this->numParticles; i++) {
        int testLoc = to_grid(this->imgWidth, this->particleLocations[i], this->gridScale);
        if (testLoc == res) {
            count++;
        }
    }

    float c = (float) count / (float) numParticles;
//    printf("your confidence is %d/%d = %f\n", count, this->numParticles, c);
    return c > param;

}

void Pfilter::sample() {
    
    // 0) check if we are confident in the robot location

    // move to device 

    if (confidence(.5)) {

        // in real life we would base on sensors but this isn't a robo class
        if (to_grid(imgWidth, particleLocations[this->particleGuess], gridScale)
                == to_grid(this->imgWidth, this->robot->get_pos(), gridScale)) {
            particleGuess = 0;
            particleLocations[0] = robot->get_pos();
            particleOrientations[0] = robot->get_angle();
            sampling_pos_variance = 2;
            numParticles = 1;
            i_am_speed = true;
            printf("wow! ur a cool robot :p You win!\n");
        } else if (confidence(.75)) {
            uniform_sample();
        }
        return;
    }

    // 1) find the best particle (GREEDY)
    double best_weight = 0;
    for (int i=0; i<this->numParticles; i++) {
        if (best_weight < this->weights[i]) {
            best_weight = this->weights[i];
            this->particleGuess = i;             
        }

    }

//    printf("best weight is %f\n", best_weight);

    // 2) calculate running weights
    double* running_weights = weights;
    for (int i=1; i<this->numParticles; i++) {
        running_weights[i] += running_weights[i-1];
    }
    
    // 3) resample every particle
    int* newParticleLocations = new int[numParticles];
    double* newParticleOrientations = new double[this->numParticles];
    for (int j=0; j<this->numParticles; j++) {
        double rand = rand_num() * running_weights[this->numParticles-1];
        for (int i=0; i<numParticles; i++) {
            if (rand < running_weights[i]) {
                int candidate_random_pos;
                do {
                    // TODO: variance is based on gridScale. Is this a good hueristic?
                    candidate_random_pos = gaussian2d(this->particleLocations[i], this->imgWidth, this->sampling_pos_variance);
                } while (grid->is_wall_at(to_grid(this->imgWidth, candidate_random_pos, this->gridScale)));
                
                newParticleLocations[j] = round(candidate_random_pos);
                newParticleOrientations[j] = gaussian1d(this->particleOrientations[i], 0.1); // ~6 degrees
                break;
            }
        }
    }

    // 4) update particle locations
    for (int i=0; i<this->numParticles; i++) {
        this->particleLocations[i] = newParticleLocations[i];
        this->particleOrientations[i] = newParticleOrientations[i];
    }

}

void Pfilter::sample() {
   
    int* d_particleGuess;
    int* d_numParticles;
    cudaMalloc(&d_particleGuess, sizeof(int));
    cudaMalloc(&d_numParticles, sizeof(int));
    cudaMemcpy(d_particleGuess, &particleGuess, sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_numParticles, &numParticles, sizeof(int), cudaMemcpyHostToDevice);
    int roboPos = robot->get_pos();
    int roboAngle = robot->get_angle();
    kernelSample<<<1,1>>>(d_particleGuess, roboPos, roboAngle); 
    cudaMemcpy(&particleGuess, d_particleGuess, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&numParticles, d_numParticles, sizeof(int), cudaMemcpyDeviceToHost);

}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


void Pfilter::update() {


    // 1) transition particles uniformly random
    transition();


    // 2) Reweight (fire rays and check intersection)
    reweight(); 

    // 3) resample
    sample_counter++; // must hit wall to activate a sample
    if (numParticles > 1 && DEBUG != 1 && sample_counter >= sample_freq) {
        sample();
        // this->sample_counter = 0;
        // this->reset_weights();
        // this->robot->print_stuff();
    }

}


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

__global__ void kernelRender(int particleScale, int numParticles, int goal, int goalScale, int robotScale, int robopos) {

    int threadId = threadIdx.x + blockDim.x * blockIdx.x;
    int imgWidth = params.imgWidth;

    float* data = params.cudaImage;

    // 1) draws the particles
    if (threadId < numParticles) {
        int pLoc = pparams.particleLocations[threadId];
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

    if (threadId == 0) {

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
        for (int y=-robotScale/2; y<robotScale/2; y++) {
            for (int x=-robotScale/2; x<robotScale/2; x++) {
                int pos = robopos + 4 * (y * imgWidth) + 4 * x;
                data[pos] = 0;
                data[pos+1] = 0;
                data[pos+2] = 1;
                data[pos+3] = 1;
            }
        }
    }


}

void RefRenderer::render() {

    int particleScale = filter->get_particle_scale();
    int numParticles = filter->get_num_particles();
    int goal = filter->get_goal() * 4;
    int goalScale = filter->get_goal_scale();
    Robot* robot = filter->get_robot();
    int robotScale = robot->get_scale() * particleScale;
    int robopos = robot->get_pos() * 4;

    dim3 blockDim(BLOCKSIZE);
    dim3 gridDim((numParticles + BLOCKSIZE - 1) / BLOCKSIZE);
    kernelRender<<<gridDim, blockDim>>>(particleScale, numParticles, goal, goalScale, robotScale, robopos);
    cudaDeviceSynchronize();

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


