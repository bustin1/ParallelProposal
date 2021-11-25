
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <algorithm>

#include "particleFilter.h"
#include "grid.h"
#include "util.h"
#include "debug.h"
#include "robot.h"

#define SAMPLE_FREQ 10


Pfilter::Pfilter(Robot* r, Grid* g, int n, int s1, int s2) : numRays (20), maxRayLen (100){

    numParticles = n;
    particleLocations = new int[n]; // x,y
    particleOrientations = new double[n]; // angles
    rays = new int[numRays * n]; // each particle gets numRays;
    weights = new double[n]; 
    robot = r;
    grid = g;
    gridScale = s1;
    particleScale = s2;
    sample_counter = 0;
    sampling_pos_variance = gridScale / 5;

    // 1) init some stuff
    robot->init_rays(20);
    grid->calc_num_open();
    grid->calc_num_closed();
    int* open = grid->open;
    int num_open = grid->num_open;
    int imgW = g->width * gridScale;

    // 2) randomly assign locations to particles in the grid

    srand(time(NULL));

    for (int i=-1; i<n; i++) {

        
        int random = rand_num() * num_open;
        int loc = open[random]; // uniformly random

        int j = to_img(g->width, loc, gridScale);

        int offsetX = rand_num() * (gridScale-2) + 1;
        int offsetY = rand_num() * (gridScale-2) + 1;

        int x = j % imgW + offsetX;
        int y = j / imgW + offsetY;
        if (i == -1) {
//            robot->set_pose(y*imgW + x, 2.0f * PI * rand_num());
            robot->set_pose(37 + imgW * 20 * gridScale, 2.0f * PI * rand_num());
        } else {
            particleLocations[i] = y*imgW + x;
            particleOrientations[i] = 2.0f * PI * rand_num();
        }

    }

    // choose best particle uniformly random
    particleGuess = rand_num() * n;

    // randomize the goal location
    int random = rand_num() * num_open;
    int loc = open[random];
    int j = to_img(g->width, loc, gridScale);

    int offsetX = rand_num() * (gridScale-2) + 1;
    int offsetY = rand_num() * (gridScale-2) + 1;

    int y = j / imgW + offsetX;
    int x = j % imgW + offsetY;
//    goal = y*imgW + x;
    goal = (imgW - 37) + 52 * imgW;

    for (int i=0; i<numParticles; i++) {
        weights[i] = 1;
    }

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
void Pfilter::find_next_step(double& dtheta, double& speed, double theta) {

    int start = particleLocations[particleGuess];
    // bfs to find shortest path
    int imgWidth = grid->width * gridScale;
    std::vector<int> best_path = bfs(to_grid(imgWidth, start, gridScale), 
                                to_grid(imgWidth, goal, gridScale), grid);

    // TODO: follow the link to find the best path with smoothing
    
    if (best_path.size() >= 2) { // haven't hit the goal state
        int nextGrid = to_img(grid->width, best_path[1], gridScale);
//        printf("next node is %d\n", best_path[1]);
        float yawRate = robot->get_yaw_rate();
        
        int nextX = (nextGrid % imgWidth) + gridScale/2;
        int nextY = (nextGrid / imgWidth) + gridScale/2;
        int startX = start % imgWidth;
        int startY = start / imgWidth;

//        printf("from %d,%d -> %d,%d\n",to_grid(imgWidth, start, gridScale)%grid->width, to_grid(imgWidth, start, gridScale)/grid->width, best_path[1]%grid->width, best_path[1]/grid->width);

        double angle_dir = atan2((double) (nextY-startY), (double) (nextX-startX));
        double turn_angle = angle_dir - theta;

//        printf("angle dir is %f\n", angle_dir * 180 / PI);
//        printf("turn angle is %f\n", turn_angle * 180 / PI);
        // clamp the turning speed
        dtheta = clamp(turn_angle, yawRate);
//        printf("dtheta is %f\n", dtheta);

        // TODO: Fine tune the variance
        dtheta = gaussian1d(dtheta, .1); 

        // TODO: Fine the tune the speed
        speed = 1; 
    } else {
        speed = 0;
        dtheta = 0;
        printf("Found flag at cell\n");
    }

}


void Pfilter::transition() {

    int imgWidth = grid->width * gridScale;

    double dtheta = 0;
    double speed = 0;
    find_next_step(dtheta, speed, particleOrientations[particleGuess]);

    // robot move
    robot->move(dtheta, speed); 

    // particle move
    for (int i=0; i<numParticles; i++) {
        int loc = particleLocations[i];
        double candidate_angle = double_mod(particleOrientations[i] + dtheta, 2 * PI);

        float dirX = speed * cos(candidate_angle);
        float dirY = speed * sin(candidate_angle);
        
        int px = loc % imgWidth;
        int py = loc / imgWidth;

        int candidate_x = round(px + dirX);
        int candidate_y = round(py + dirY);

        int candidate_pos = candidate_x+imgWidth*candidate_y;

        // if hit wall, don't move 
        if (!grid->is_wall_at(to_grid(imgWidth, candidate_pos, gridScale))) {
            loc = candidate_pos;  
        }

        // Update particle pose
        particleLocations[i] = loc;
        particleOrientations[i] = candidate_angle;
    }


}


int Pfilter::getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4) {

    int imgWidth = grid->width * gridScale;
    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // colinear :(
    if (denom == 0) {
        return -1;
    }

    double numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    double t = numerator / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    if (t > 0 && t < 1 && u > 0) {
        int x = round(x1 + t * (double)(x2 - x1));
        int y = round(y1 + t * (double)(y2 - y1));
        return x + imgWidth*y;
    }
    
    // segments too far apart :(
    return -1;
}

int Pfilter::getClosestIntersection(int ploc, double angle) {

    int gw = grid->width;
    int gh = grid->height;
    int iw = gw * gridScale;

    int num_closed = gw * gh - grid->num_open;
    int* closed = grid->closed;

    double dir_x = cos(angle);
    double dir_y = sin(angle);

    // particle locations
    int x3 = ploc % (iw);
    int y3 = ploc / (iw);
    int x4 = x3 + maxRayLen * dir_x;
    int y4 = y3 + maxRayLen * dir_y;


    // save best intersection
    int closestX = x4;
    int closestY = y4; 
    int closestDist = maxRayLen*maxRayLen;

    for (int i=0; i<num_closed; i++) {

        int wloc = closed[i];

        // WEIRDEST BUG: line intersection will wrap since it's a 1d contiguous array
        int x1 = wloc % gw * gridScale; // left_x
        int y1 = wloc / gw * gridScale; // bot_y
        int x2 = x1 + gridScale; // right_x
        int y2 = y1 + gridScale; // top_y

        int a[4][4] = {{x1+1,y1-1,x1+1,y2+1}, 
                      {x1-1,y2-1,x2+1,y2-1}, 
                      {x2-1,y2+1,x2-1,y1-1},
                      {x2+1,y1+1,x1-1,y1+1}};

        for (int j=0; j<4; j++) {

            int intersection = getSingleIntersection(a[j][0], a[j][1], 
                                                        a[j][2], a[j][3], 
                                                            x3, y3, x4, y4);

            if (intersection >= 0) {

                int x = intersection % iw;
                int y = intersection / iw;

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

    return closestY * iw + closestX;
}



void Pfilter::firerays(int loc, int* ptr, double angle_bias) {

    const double dtheta = 2 * PI / (double)numRays;
    int imgW = grid->width * gridScale;
    for (int i=0; i<numRays; i++) {
        double angle = i * dtheta + angle_bias;
        int coord = getClosestIntersection(loc, angle);
        *ptr = coord;
        ptr++;
    }
     
}


void Pfilter::reweight() {

    // get robo observation
    int* roboRay = robot->get_rays();
    
    firerays(robot->get_pos(), roboRay, robot->get_angle());
    const double var = maxRayLen * numRays;
    int iw = grid->width * gridScale;

    double* roboRayLen = new double[numRays];
    for (int i=0; i<numRays; i++) {
        int rRay = roboRay[i];
        int rRayX = rRay % iw;
        int rRayY = rRay / iw;
        double rRayLen = sqrt(pow((rRayX-robot->get_x(iw)), 2) + pow(rRayY-robot->get_y(iw), 2));
        roboRayLen[i] = rRayLen;
    }

    for (int i=0; i<numParticles; i++) {

        int loc = particleLocations[i];
        firerays(loc, rays + i*numRays, particleOrientations[i]);

        // reweight with gaussian distribution
        // 1) we assume that given the particle's location
        // what is the probability that we got an observation
        // 2) this follows ~ e^{-r * p}, where r := robo observation
        // and p := particle's observation

        int px = loc % iw;
        int py = loc / iw;
        double exp = 0;
        for (int j=0; j<numRays; j++) {

            int pRay = rays[i*numRays+j];
            int pRayX = pRay % iw;
            int pRayY = pRay / iw;
            double pRayLen = sqrt(pow((pRayX-px), 2) + pow(pRayY-py, 2));
            
            double rayDiff = roboRayLen[j] - pRayLen;
            exp += pow(rayDiff, 2);

        }
//        printf("exp: %f\n", exp);
//        printf("-exp/var: %f\n", -exp/var);
        weights[i] *= pow(E, -exp / var / SAMPLE_FREQ); // TODO: tune
//        printf("weights[%d]: %f\n", i, weights[i]);

    }
//    printf("particleFilter.cpp: particle locations is %d\n", particleLocations[0]);

}

void Pfilter::sample() {

    // find the best particle
    double best_weight = 0;
    for (int i=0; i<numParticles; i++) {
        if (best_weight < weights[i]) {
            best_weight = weights[i];
            particleGuess = i;             
        }

    }

    printf("best weight is %f\n", best_weight);
    if (best_weight > .9) {
        sampling_pos_variance *= .9;
        printf("wow!\n");
    }

    int imgWidth = grid->width * gridScale;

    double* running_weights = weights;
    for (int i=1; i<numParticles; i++) {
        running_weights[i] += running_weights[i-1];
    }
    
//    float rand = rand_num() * running_weights[numParticles-1];

    // stochastically pull a random particle based on it's weight
    // that random particle is the guess for where the robot lies
//    for (int i=0; i<numParticles; i++) {
//        if (rand < running_weights[i]) {
//            particleGuess = i;             
//            break;
//        }
//    }


    // resample every particle
    int* newParticleLocations = new int[numParticles];
    double* newParticleOrientations = new double[numParticles];
    for (int j=0; j<numParticles; j++) {
        double rand = rand_num() * running_weights[numParticles-1];
        for (int i=0; i<numParticles; i++) {
            if (rand < running_weights[i]) {
                int candidate_random_pos;
                do {
                    // TODO: variance is based on gridScale. Is this a good hueristic?
                    candidate_random_pos = gaussian2d(particleLocations[i], imgWidth, gridScale / 5); // TODO: change this
                } while (grid->is_wall_at(to_grid(imgWidth, candidate_random_pos, gridScale)));
                
                newParticleLocations[j] = round(candidate_random_pos);
                newParticleOrientations[j] = gaussian1d(particleOrientations[i], 0.05); // ~3 degrees
                break;
            }
        }
    }

    for (int i=0; i<numParticles; i++) {
        particleLocations[i] = newParticleLocations[i];
        particleOrientations[i] = newParticleOrientations[i];
    }

}


void Pfilter::update() {

    // TODO
    // add some robot random walk


    // 1) transition particles uniformly random
    transition();

    // 2) Reweight (fire rays and check intersection)
    reweight(); 

    // 3) resample
    sample_counter++;
    if (DEBUG != 1 && sample_counter == SAMPLE_FREQ) {
        sample();
        sample_counter = 0;
        for (int i=0; i<numParticles; i++) {
            weights[i] = 1;
        }
    }
    

}























