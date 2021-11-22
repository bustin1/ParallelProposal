
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#include "particleFilter.h"
#include "grid.h"
#include "util.h"

#define PI 3.14159265


Pfilter::Pfilter(Grid* g, int n, int s1, int s2) : numRays (10) {

    numParticles = n;
    particleLocations = new int[n];
    grid = g;
    gridScale = s1;
    particleScale = s2;

    // 1) randomly assign locations to particles in the grid
    
    grid->calc_num_open();
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
    }


}

Pfilter::~Pfilter() {
    if (particleLocations) {
        delete particleLocations;
    }
}


// getters and setters
Grid* Pfilter::getGrid() {
    return grid;
}

int Pfilter::get_numParticles() {
    return numParticles;
}

int* Pfilter::get_particleLocations() {
    return particleLocations;
}

int Pfilter::get_grid_scale() {
    return gridScale;
}

int Pfilter::get_particle_scale() {
    return particleScale;
}


void Pfilter::transition() {
    int imgWidth = grid->width * gridScale;
    for (int i=0; i<numParticles; i++) {
        float dir = rand_num();
        int loc = particleLocations[i];

        if (dir < .25) { // up
            loc += imgWidth;
            int top = loc + imgWidth * (particleScale - 1);
            int j = to_grid(imgWidth, top, gridScale);
            if (grid->is_wall_at(j)) {
                loc -= imgWidth;
            }
        } else if (dir < .5) { // right
            loc += 1;
            int right = loc + (particleScale - 1);
            int j = to_grid(imgWidth, right, gridScale);
            if (grid->is_wall_at(j)) {
                loc -= 1;
            }
        } else if (dir < .75) { // down 
            loc -= imgWidth;
            int bot = loc - imgWidth * (particleScale - 1);
            int j = to_grid(imgWidth, bot, gridScale);
            if (grid->is_wall_at(j)) {
                loc += imgWidth;
            }
        } else { // left
            loc -= 1;
            int left = loc - (particleScale - 1);
            int j = to_grid(imgWidth, left, gridScale);
            if (grid->is_wall_at(j)) {
                loc += 1;
            }
        }

        particleLocations[i] = loc;
    }
}

int Pfilter::fireray(int loc, float angle) {

//    int slope = 
     
    return 0;
}


void Pfilter::reweight() {
    
//    const double dtheta = 2 * PI / numRays;
//    for (int i=0; i<numParticles; i++) {
//        for (int j=0; j<numRays; j++) {
//            int loc = particleLocations[i];
//            double angle = j * dtheta;
//            int intersectionPnt = fireray(loc, angle);
//            // draw image lines 
//        }
//    }

}


void Pfilter::update() {
    // TODO
    // 1) transition particles uniformly random
    transition();
    // 2) Reweight (fire rays and check intersection)
    
    // 3) resample
}























