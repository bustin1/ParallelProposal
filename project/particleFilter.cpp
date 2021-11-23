
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <algorithm>    // std::min

#include "particleFilter.h"
#include "grid.h"
#include "util.h"
#include "debug.h"
#include "robot.h"



Pfilter::Pfilter(Robot* r, Grid* g, int n, int s1, int s2) : numRays (10), maxRayLen (1000) {

    numParticles = n;
    particleLocations = new int[n]; // x,y
    particleOrientations = new double[n]; // angles
    rays = new int[numRays * n]; // each particle gets numRays;
    weights = new double[n]; 
    robot = r;
    grid = g;
    gridScale = s1;
    particleScale = s2;

    // 1) randomly assign locations to particles in the grid
    
    grid->calc_num_open();
    grid->calc_num_closed();
    int* open = grid->open;
    int num_open = grid->num_open;
    int imgW = g->width * gridScale;

    srand(time(NULL));

    for (int i=0; i<n; i++) {
        int random = rand_num() * num_open;
        int loc = open[random]; // uniformly random

        int j = to_img(g->width, loc, gridScale);

        int offsetX = rand_num() * (gridScale-2) + 1;
        int offsetY = rand_num() * (gridScale-2) + 1;

        int y = j / imgW + offsetX;
        int x = j % imgW + offsetY;
        particleLocations[i] = y*imgW + x;
        particleOrientations[i] = 2.0f * PI * rand_num();
        printf("particleOrientation %f\n", particleOrientations[i]);
    }

    // choose best particle uniformly random
    particleGuess = rand_num() * n;

    // TODO: randomly set goal
    goal = 190 * imgW + 190; 

}

Pfilter::~Pfilter() {
    if (particleLocations) {
        delete particleLocations;
        delete particleOrientations;
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


//int Pfilter::find_next_step(int& dtheta, int& speed) {
//    return
//}

void Pfilter::transition() {

//    int imgWidth = grid->width * gridScale;
//
//    // TODO move based on best particle location
//
//    int dtheta;
//    int speed;
//    find_next_step(dtheta, speed);
//    robot->move(dtheta, speed); 
//
//    for (int i=0; i<numParticles; i++) {
//        int loc = particleLocations[i];
//        int angle = particleOrientations[i] + dtheta; // particle angle moves wrt to bestParticle
//
//        float dirX = speed * cos(angle);
//        float dirY = speed * sin(angle);
//        
//        int imgWidth = grid->width * gridScale;
//        int rx = get_x(imgWidth);
//        int ry = get_y(imgWidth);
//
//        int candidate_x = round(rx + dirX);
//        int candidate_y = round(ry + dirY);
//
//        int candidate_pos = candidate_x+imgWidth*candidate_y
//
//        // if hit wall, don't move 
//        if (!grid->is_wall_at(to_grid(imgWidth, candidate_pos, gridScale))) {
//            pos = candidate_pos;  
//        }
//
//    }
}


int Pfilter::getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4) {

    int imgWidth = grid->width * gridScale;
    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // colinear :(
    if (denom == 0) {
        return -1;
    }

    float numerator = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    float t = numerator / denom;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    if (t > 0 && t < 1 && u > 0) {
        int x = round(x1 + t * (float)(x2 - x1));
        int y = round(y1 + t * (float)(y2 - y1));
        return x + imgWidth*y;
    }
    
    // segments too far apart :(
    return -1;
}

// TODO: fix line intersection 
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

        int x1 = wloc % gw * gridScale; // left_x
        int y1 = wloc / gw * gridScale; // bot_y
        int x2 = x1 + gridScale; // right_x
        int y2 = y1 + gridScale; // top_y

        int a[4][4] = {{x1,y1,x1,y2}, 
                      {x1,y2,x2,y2}, 
                      {x2,y2,x2,y1},
                      {x2,y1,x1,y1}};

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
    int iw = grid->width * gridScale;
    for (int i=0; i<numRays; i++) {
        double angle = ((double)i) * dtheta + angle_bias;
        int coord = getClosestIntersection(loc, angle);
        if (DEBUG) { // for printing the rays in display.cpp
            *ptr = coord;
            ptr++;
        } else {
            int dx = loc % iw;
            int dy = loc / iw;
            *ptr = pow((dx*dx) + (dy*dy), .5); // NOTE: distance with sqrt may be inefficent
            ptr++;
        }
    }
     
}


void Pfilter::reweight() {

    // get robo observation
    int* r = new int[numRays];
    firerays(robot->get_pos(), r, robot->get_angle());
    const double var = grid->width * gridScale;

    
    for (int i=0; i<numParticles; i++) {

        int loc = particleLocations[i];
        firerays(loc, rays + i*numRays, particleOrientations[i]);

        // reweight with gaussian distribution
        // 1) we assume that given the particle's location
        // what is the probability that we got an observation
        // 2) this follows ~ e^{-r * p}, where r := robo observation
        // and p := particle's observation

        double exp = 0;
        for (int j=0; j<numRays; j++) {
            double p = rays[i*numRays+j];
            double rj = r[j];
            double diff = p - rj;
            exp += diff * diff;
//            printf("exp[%d]: %f\n", j, exp);
        }
        weights[i] = pow(E, -exp / var);
//        printf("weights[%d]: %f\n", i, weights[i]);

    }

}

void Pfilter::sample() {

//    int imgWidth = grid->width * gridScale;
//    double* running_weights = weights;
//    for (int i=1; i<numParticles; i++) {
//        running_weights[i] += running_weights[i-1];
//    }
//    
//    float rand = rand_num() * running_weights[numParticles-1];
//
//    // stochastically pull a random particle based on it's weight
//    // that random particle is the guess for where the robot lies
//    for (int i=numParticles-1; i>=0; i--) {
//        if (rand < running_weights[i]) {
//            particleGuess = i;             
//            break;
//        }
//    }
//
//    // resample every particle
//    for (int j=0; j<numParticles; j++) {
//        float rand = rand_num() * running_weights[numParticles-1];
//        for (int i=numParticles-1; i>=0; i--) {
//            if (rand < running_weights[i]) {
//                int candidate_random_pos;
//                do {
//                    // TODO: variance is based on gridScale. Is this a good hueristic?
//                    candidate_random_pos = gaussian(particleLocations[i], imgWidth, round(gridScale / 20)); 
//                } while (grid->is_wall_at(to_grid(imgWidth, candidate_random_pos, gridScale)));
//                particleLocations[i] = candidate_random_pos;
//                break;
//            }
//        }
//    }

}


void Pfilter::update() {

    // TODO
    // 1) transition particles uniformly random
//    transition();
    // 2) Reweight (fire rays and check intersection)
    reweight(); 
    // 3) resample
//    sample();

}























