
#ifndef _PARTICLE_FILTER_H
#define _PARTICLE_FILTER_H

#include "grid.h"
#include "image.h"
#include "robot.h"
#include "util.h"

class Pfilter {

private:

    Grid* grid;
    Robot* robot;

    int* particleLocations;
    int* rays;
    double* particleOrientations;
    double* weights;

    const int numRays;
    const int maxRayLen;
    const int sample_freq;
    const int imgWidth;
    const int gridScale;
    const int particleScale;
    const int maxNumParticles;
    const int DEBUG;

    int numParticles;
    int goalScale;
    int goal;
    int particleGuess;
    int sample_counter;
    int sampling_pos_variance;
    int numThreads;

    bool i_am_speed;

    // unseen helper functions
    void init();
    int rand_location();
    void find_next_step(double& dtheta, double& speed, double theta);
    void transition();
    void firerays(int loc, int *ptr, double angle_bias);
    void reweight();
    void sample();
    void uniform_sample();
    void reset_weights();
    int getSingleIntersection(int x1, int y1, int x2, int y2,
                                   int x3, int y3, int x4, int y4);
    int getClosestIntersection(int loc, double angle);
    bool confidence(float param);

public:

    Pfilter(Robot* robot, Grid* grid, 
                int numParticles, 
                const int gridScale, 
                const int particleScale,
                const int numRays,
                const int DEBUG,
                int numThreads);

    virtual ~Pfilter();

    void update();

    // getters and setters
    int get_num_particles();
    int get_goal_scale();
    int get_num_rays();
    int get_goal();
    int get_grid_scale();
    int get_particle_scale();
    int get_best_particle();

    Robot* get_robot();
    Grid* get_grid();

    int* get_particle_locations();
    int* get_rays();

};



#endif
