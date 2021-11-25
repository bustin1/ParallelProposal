
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

#define SAMPLE_FREQ 5

// TODO: robot explore unvisited states
// TODO: once particles converge, guess best location
// TODO: store best paths in a table


Pfilter::Pfilter(Robot* r, Grid* g, int n, int s1, int s2) : numRays (20), maxRayLen (100){

    numParticles = n;
    maxNumParticles = n;
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
    robotHitWall = 0;
    i_am_speed = false;

    // 1) init some stuff
    robot->init_rays(numRays);
    grid->calc_num_open();
    grid->calc_num_closed();
    int imgW = g->width * gridScale;

    srand(time(NULL));
    // 2) randomly assign locations to particles in the grid
    uniform_sample();

    // 3) randomize the goal location
//    int random = rand_num() * num_open;
//    int loc = open[random];
//    int j = to_img(g->width, loc, gridScale);
//
//    int offsetX = rand_num() * (gridScale-2) + 1;
//    int offsetY = rand_num() * (gridScale-2) + 1;
//
//    int y = j / imgW + offsetX;
//    int x = j % imgW + offsetY;
//    goal = y*imgW + x;
    goal = (imgW - 1.5f*gridScale) + (gridScale * 2.f) * imgW;

    for (int i=0; i<numParticles; i++) {
        weights[i] = 1;
    }

}


void Pfilter::uniform_sample() {

    int num_open = grid->num_open;
    int* open = grid->open;
    int imgW = grid->width * gridScale;

    for (int i=0; i<numParticles; i++) {

        int random = rand_num() * num_open;
        int loc = open[random]; // uniformly random

        int j = to_img(grid->width, loc, gridScale);

        int offsetX = rand_num() * (gridScale-2) + 1;
        int offsetY = rand_num() * (gridScale-2) + 1;

        int x = j % imgW + offsetX;
        int y = j / imgW + offsetY;

        particleLocations[i] = y*imgW + x;
        particleOrientations[i] = 2.0f * PI * rand_num();

    }
    particleGuess = rand_num() * numParticles;

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
        float yawRate = robot->get_yaw_rate();
        
        int nextX = (nextGrid % imgWidth) + gridScale/2;
        int nextY = (nextGrid / imgWidth) + gridScale/2;
        int startX = start % imgWidth;
        int startY = start / imgWidth;

        double angle_dir = atan2((double) (nextY-startY), (double) (nextX-startX));
        double turn_angle = angle_dir - theta;

        // clamp the turning speed
        dtheta = clamp(turn_angle, yawRate);

        // TODO: Fine tune the variance
        dtheta = gaussian1d(dtheta, .1); 

        // TODO: Fine the tune the speed
        speed = 2; 

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

    double dirX = 0; 
    double dirY = 0; 

    // robot move
    if (robotHitWall == 0) {
        find_next_step(dtheta, speed, particleOrientations[particleGuess]);
        robotHitWall = robot->move(dtheta, speed); 
    } else { // ouc
        speed = 2;
        robot->move_greedy(dtheta, speed); // U tell robot how fast
        printf("robot hit wall! (greedy)\n");
    }

    // particle move
    for (int i=0; i<numParticles; i++) {

        double candidate_angle = double_mod(particleOrientations[i] + dtheta, 2 * PI);

        dirX = speed * cos(candidate_angle);
        dirY = speed * sin(candidate_angle);
        
        int px = particleLocations[i] % imgWidth;
        int py = particleLocations[i] / imgWidth;

        int candidate_x = round(px + dirX);
        int candidate_y = round(py + dirY);

        int candidate_pos = candidate_x+imgWidth*candidate_y;

        particleOrientations[i] = candidate_angle;

        // if hit wall, don't move 
        if (!grid->is_wall_at(to_grid(imgWidth, candidate_pos, gridScale))) {
            // Update particle pose
            particleLocations[i] = candidate_pos;
        } else { // randomize the angle for the particle to move in
            particleOrientations[i] = gaussian1d(candidate_angle, .5);
        }

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
        double rRayLen = dist2d(robot->get_pos(), rRay, iw);
        roboRayLen[i] = rRayLen;
    }

    for (int i=0; i<numParticles; i++) {

        int pLoc = particleLocations[i];
        firerays(pLoc, rays + i*numRays, particleOrientations[i]);

        // reweight with gaussian distribution
        // 1) we assume that given the particle's location
        // what is the probability that we got an observation
        // 2) this follows ~ e^{-r * p}, where r := robo observation
        // and p := particle's observation

        double exp = 0;
        for (int j=0; j<numRays; j++) {

            int pRay = rays[i*numRays+j];
            double pRayLen = dist2d(pRay, pLoc, iw);
            
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

bool Pfilter::confidence(float param) {
    int count = 0;
    int imgWidth = grid->width * gridScale;
    int res = to_grid(imgWidth, particleLocations[0], gridScale);

    // moore's majority voting algorithm
    for (int i=0; i<numParticles; i++) {
        int testLoc = to_grid(imgWidth, particleLocations[i], gridScale);
        if (testLoc == res) {
            count++;
        } else {
            count--;
        }
        if (count == 0) {
            res = testLoc;
            count = 1;
        }
    }

    // test if it is a majority
    count = 0;
    for (int i=0; i<numParticles; i++) {
        int testLoc = to_grid(imgWidth, particleLocations[i], gridScale);
        if (testLoc == res) {
            count++;
        }
    }

    float c = (float) count / (float) numParticles;
    printf("your confidence is %d/%d = %f\n", count, numParticles, c);
    return c > param;

}

void Pfilter::sample() {
    
    int imgWidth = grid->width * gridScale;

    // 0) check if we are confident in the robot location
    if (confidence(.5)) {

        printf("wow! ur a cool robot :p\n");
        // in real life we would base on sensors but this isn't a robo class
        if (to_grid(imgWidth, particleLocations[particleGuess], gridScale)
                == to_grid(imgWidth, robot->get_pos(), gridScale)) {
            i_am_speed = true;
        } else {
            uniform_sample();
        }
        return;
    }

    // 1) find the best particle (GREEDY)
    double best_weight = 0;
    for (int i=0; i<numParticles; i++) {
        if (best_weight < weights[i]) {
            best_weight = weights[i];
            particleGuess = i;             
        }

    }

    printf("best weight is %f\n", best_weight);

    // 2) calculate running weights
    double* running_weights = weights;
    for (int i=1; i<numParticles; i++) {
        running_weights[i] += running_weights[i-1];
    }
    
    // 3) resample every particle
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


    // 1) transition particles uniformly random
    transition();

    // 2) Reweight (fire rays and check intersection)
    reweight(); 

    // 3) resample
    if (robotHitWall) {
        sample_counter++; // must hit wall to activate a sample
    } 

    if (DEBUG != 1 && sample_counter == SAMPLE_FREQ) {
        sample();
        sample_counter = 0;
        robotHitWall = 0;
        for (int i=0; i<numParticles; i++) {
            weights[i] = 1;
        }
        printf("done sampling :O !\n");
        robot->print_stuff();
    }


}























