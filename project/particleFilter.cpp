
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include <omp.h>

#include <immintrin.h>

#include "include/cycleTimer.h"
#include "include/particleFilter.h"

// TODO: robot explore unvisited states
// TODO: once particles converge, guess best location
// TODO: store best paths in a table?
// TODO: particleGuess hit wall => resample
// TODO: for the robot, once the robot remembers too many states, refresh unvisted


Pfilter::Pfilter(Robot* robot, Grid* grid, 
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
                imgWidth (grid->width * gridScale),
                sample_freq(5),
                maxRayLen (10 * gridScale) {


    this->particleLocations = new int[numParticles];
    this->particleOrientations = new double[numParticles];
    this->rays = new int[numRays * numParticles];
    this->weights = new double[numParticles]; 
    this->robot = robot;
    this->grid = grid;
    this->i_am_speed = false;

    this->sample_counter = 0;
    this->sampling_pos_variance = gridScale / 5;

    // 1) init some stuff
    this->grid->init();
    if (DEBUG) {
        this->robot->init(numRays, grid, gridScale, -1);
    } else {
        this->robot->init(numRays, grid, gridScale, rand_location());
    }
    this->goalScale = this->robot->get_scale();

    // 2) randomly assign locations to particles in the grid
    this->uniform_sample();

    // 3) randomize the goal location
    this->init();

    // 4) reweight to prevent seg fault on first transition
    this->reweight(); 
    


}

void Pfilter::reset_weights() {
    for (int i=0; i<this->numParticles; i++) {
        this->weights[i] = 1;
    }
}

int Pfilter::rand_location() {

    int random = rand_num() * this->grid->num_open;
    int loc = this->grid->open[random];
    int j = to_img(this->grid->width, loc, this->gridScale);

    int offsetX = rand_num() * (this->gridScale-2) + 1;
    int offsetY = rand_num() * (this->gridScale-2) + 1;

    int x = get_x(j, this->imgWidth) + offsetY;
    int y = get_y(j, this->imgWidth) + offsetX;

    return y*this->imgWidth + x;

}

void Pfilter::init() {

    if (DEBUG) {
        // bottom right
        this->goal = (this->imgWidth - 1.5f*this->gridScale) + (this->gridScale * 1.5f) * this->imgWidth;
    } else {
        this->goal = this->rand_location();
    }

}

void Pfilter::uniform_sample() {


    // 1) Particle gets rand location
    for (int i=0; i<this->numParticles; i++) {

        this->particleLocations[i] = this->rand_location();
        this->particleOrientations[i] = rand_angle();

    }

    // 2) Choose random best particle
    this->particleGuess = rand_num() * this->numParticles;

    // 3) Reset all particle weights
    this->reset_weights();

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
    return this->grid;
}

int Pfilter::get_goal_scale() {
    return this->goalScale;
}

int Pfilter::get_num_particles() {
    return this->numParticles;
}

int* Pfilter::get_particle_locations() {
    return this->particleLocations;
}

int Pfilter::get_grid_scale() {
    return this->gridScale;
}

int Pfilter::get_particle_scale() {
    return this->particleScale;
}

int* Pfilter::get_rays() {
    return this->rays;
}

int Pfilter::get_num_rays() {
    return this->numRays;
}

int Pfilter::get_goal() {
    return this->goal;
}

int Pfilter::get_best_particle() {
    return this->particleGuess;
}

Robot* Pfilter::get_robot() {
    return this->robot;
}



// https://gamedev.stackexchange.com/questions/82710/smooth-path-direct-movement
void Pfilter::find_next_step(double& dtheta, double& speed, double theta) {

    int start = this->particleLocations[this->particleGuess];
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


void Pfilter::transition() {

    double dtheta = 0;
    double speed = 0;

    double dirX = 0; 
    double dirY = 0; 

    // 1) robot move
    if (this->i_am_speed) {
        find_next_step(dtheta, speed, this->particleOrientations[this->particleGuess]);
        this->robot->move(dtheta, speed); 
    } else {
        // Robot will do it's own stuff
        this->robot->move_greedy(dtheta, speed); 
    }

    // 2) move each particle
    double currentTime = CycleTimer::currentSeconds();
    for (int i=0; i<this->numParticles; i++) {
//        printf("%d: thread # %d\n", i, omp_get_thread_num());

        double candidate_angle = 
            double_mod(this->particleOrientations[i] + dtheta, 2 * PI);

        dirX = speed * cos(candidate_angle);
        dirY = speed * sin(candidate_angle);
        
        int px = get_x(this->particleLocations[i], this->imgWidth);
        int py = get_y(this->particleLocations[i], this->imgWidth);

        int candidate_x = round(px + dirX);
        int candidate_y = round(py + dirY);

        int candidate_pos = candidate_x + this->imgWidth*candidate_y;

        this->particleOrientations[i] = candidate_angle;

        if (!grid->is_wall_at(to_grid(this->imgWidth, candidate_pos, this->gridScale))) {
            this->particleLocations[i] = candidate_pos;
        } else { 
            // randomize the angle for the particle to move in
            this->particleOrientations[i] = gaussian1d(candidate_angle, .25);
        }

    }
    double endTime = CycleTimer::currentSeconds();
    printf("Transition Time: %.3f\n", endTime-currentTime);


}


int Pfilter::getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4) {

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // 1) colinear :(
    if (denom == 0) { // Interesting this case doesn't occur that often
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

// TODO: HERE!!
//bool Pfilter::wall_in_box(int wLoc, int center) {
//
//    int x1 = get_x(wLoc, this->grid->width);
//    int y1 = get_y(wLoc, this->grid->width);
//
//    int x2 = get_x(to_grid(this->imgWidth, center, this->gridScale), this->grid->width);
//    int y2 = get_y(to_grid(this->imgWidth, center, this->gridScale), this->grid->width);
//
//    const int s = this->maxRayLen / this->gridScale;
//    return (x1 > x2 - s && x1 < x2 + s && y1 > y2 - s && y1 < y2 + s);
//}

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

//        if (!wall_in_box(wLoc, pLoc)) {
//            continue;
//        }

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



void Pfilter::reweight() {

    // get robo observation
    int* roboRay = this->robot->get_rays();
    int roboPos = this->robot->get_pos();
    this->firerays(roboPos, roboRay, this->robot->get_angle());

    // 1) fire ray for robot
    double* roboRayLen = new double[numRays];
    for (int i=0; i<numRays; i++) {
        double rRayLen = dist2d(roboPos, roboRay[i], this->imgWidth);
        roboRayLen[i] = rRayLen;
    }

    // 2) TODO: is this the best variance for particle weight?
    const double variance = maxRayLen * numRays * 5;

    double currentTime = CycleTimer::currentSeconds();
#pragma omp parallel
{
    #pragma omp for
    for (int i=0; i<this->numParticles; i++) {

        int pLoc = this->particleLocations[i];
        firerays(pLoc, rays + i*this->numRays, this->particleOrientations[i]);

        // reweight with gaussian distribution
        // 1) we assume that given the particle's location
        // what is the probability that we got an observation
        // 2) this follows ~ e^{-r * p}, where r := robo observation
        // and p := particle's observation

        double exp = 0;
        for (int j=0; j<this->numRays; j++) {

            int pRay = rays[i*this->numRays+j];
            double pRayLen = dist2d(pRay, pLoc, this->imgWidth);

            double rayDiff = roboRayLen[j] - pRayLen;
            exp += pow(rayDiff, 2);

        }
        this->weights[i] *= pow(E, -exp / variance / this->sample_freq);

    }
    double endTime = CycleTimer::currentSeconds();
    printf("Process %d =====> Reweight Time: %.3f\n", omp_get_thread_num(), endTime-currentTime);

}

}

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
    if (confidence(.5)) {

        // in real life we would base on sensors but this isn't a robo class
        if (to_grid(this->imgWidth, this->particleLocations[this->particleGuess], this->gridScale)
                == to_grid(this->imgWidth, this->robot->get_pos(), this->gridScale)) {
            this->particleGuess = 0;
            this->particleLocations[0] = robot->get_pos();
            this->particleOrientations[0] = robot->get_angle();
            this->sampling_pos_variance = 2;
            this->numParticles = 1;
            this->i_am_speed = true;
            printf("wow! ur a cool robot :p You win!\n");
        } else if (confidence(.75)) {
            this->uniform_sample();
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



void Pfilter::update() {


    // 1) transition particles uniformly random
    this->transition();

    // 2) Reweight (fire rays and check intersection)
    this->reweight(); 

    // 3) resample
    this->sample_counter++; // must hit wall to activate a sample
    if (numParticles > 1 && this->DEBUG != 1 && sample_counter >= this->sample_freq) {
        this->sample();
        this->sample_counter = 0;
        this->reset_weights();
//        this->robot->print_stuff();
    }

}























