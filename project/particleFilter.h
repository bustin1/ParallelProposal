
#ifndef _PARTICLE_FILTER_H
#define _PARTICLE_FILTER_H

#include "grid.h"
#include "image.h"
#include "robot.h"

class Pfilter {

private:

    Grid* grid;
    Robot* robot;

    const int numRays;
    const int maxRayLen;

    int numParticles;
    int gridScale;
    int particleScale;
    int goal;
    int* particleLocations;
    double* particleOrientations;
    int* rays;
    int particleGuess;
    double* weights;
    int sample_counter;
    int sampling_pos_variance;

    // unseen helper functions
    void find_next_step(double& dtheta, double& speed, double theta);
    void transition();
    void firerays(int loc, int *ptr, double angle_bias);
    void reweight();
    void sample();
    int getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4);
    int getClosestIntersection(int loc, double angle);



public:

    Pfilter(Robot* robot, Grid* grid, int numParticles, int scale, int particleSize);

    Grid* get_grid();

    virtual ~Pfilter();

    void setup();

    void update();

    int get_num_particles();
    
    int* get_particle_locations();

    int* get_rays();
    
    int get_num_rays();

    int get_goal();
    
    int get_grid_scale();

    int get_particle_scale();

    int get_best_particle();

    Robot* get_robot();

};



#endif
