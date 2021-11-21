
#ifndef _PARTICLE_FILTER_H
#define _PARTICLE_FILTER_H

#include "grid.h"
#include "image.h"

class Pfilter {

private:

    Grid* grid;
    int numParticles;
    int scale;
    int* particleLocations;
    void transition();


public:

    Pfilter(Grid* grid, int numParticles, int scale);

    Grid* getGrid();

    virtual ~Pfilter();

    void setup();

    void update();

    int get_numParticles();
    
    int* get_particleLocations();

};



#endif
