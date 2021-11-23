
#ifndef _PARTICLE_FILTER_H
#define _PARTICLE_FILTER_H

#include "grid.h"
#include "image.h"

class Pfilter {

private:

    Grid* grid;

    const int numRays;

    int numParticles;
    int gridScale;
    int particleScale;
    int* particleLocations;
    int* rays;

    // unseen helper functions
    void transition();
    void firerays(int loc, int *ray);
    void reweight();
    int getIntersection(int loc, double angle);


public:

    Pfilter(Grid* grid, int numParticles, int scale, int particleSize);

    Grid* get_grid();

    virtual ~Pfilter();

    void setup();

    void update();

    int get_num_particles();
    
    int* get_particle_locations();

    int* get_rays();
    
    int get_num_rays();
    
    int get_grid_scale();

    int get_particle_scale();

};



#endif
