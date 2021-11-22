
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

    // unseen helper functions
    void transition();
    int fireray(int loc, float angle);
    void reweight();


public:

    Pfilter(Grid* grid, int numParticles, int scale, int particleSize);

    Grid* getGrid();

    virtual ~Pfilter();

    void setup();

    void update();

    int get_numParticles();
    
    int* get_particleLocations();
    
    int get_grid_scale();

    int get_particle_scale();

};



#endif
