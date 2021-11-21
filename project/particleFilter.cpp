
#include "particleFilter.h"
#include "grid.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


float rand_num() {
    return (float)rand()/RAND_MAX;
}

// converts an image pixel to a grid location
int convert(int width, int i, int scale) {

    int iw = width * scale;

    int x = i % iw;
    int y = i / iw;

    int gx = x / scale; 
    int gy = y / scale;

    return gx + gy*(width);
}

Pfilter::Pfilter(Grid* g, int n, int s) {

    numParticles = n;
    particleLocations = new int[n];
    grid = g;
    scale = s;

    // 1) randomly assign locations to particles in the grid
    
    grid->calc_num_open();
    int* open = grid->open;
    int num_open = grid->num_open;
    int imgW = g->width * scale;

    srand(time(NULL));

    for (int i=0; i<n; i++) {
        int random = rand_num() * num_open;
        int loc = open[random]; // uniformly random

        int j = g->to_img(loc, scale);


        int offsetX = rand_num() * (scale-2) + 1;
        int offsetY = rand_num() * (scale-2) + 1;


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

Grid* Pfilter::getGrid() {
    return grid;
}

int Pfilter::get_numParticles() {
    return numParticles;
}

int* Pfilter::get_particleLocations() {
    return particleLocations;
}


void Pfilter::transition() {
    int width = grid->width;
    for (int i=0; i<numParticles; i++) {
        float dir = rand_num();
        int loc = particleLocations[i];

        if (dir < .25) { // up
            loc += width * scale;
            int j = convert(width, loc, scale);
            if (grid->is_wall_at(j)) {
                loc -= width * scale;
            }
        } else if (dir < .5) { // right
            loc += 1;
            int j = convert(width, loc, scale);
            if (grid->is_wall_at(j)) {
                loc -= 1;
            }
        } else if (dir < .75) { // down 
            loc -= width * scale;
            int j = convert(width, loc, scale);
            if (grid->is_wall_at(j)) {
                loc += width * scale;
            }
        } else { // left
            loc -= 1;
            int j = convert(width, loc, scale);
            if (grid->is_wall_at(j)) {
                loc += 1;
            }
        }

        particleLocations[i] = loc;
    }
}


void Pfilter::update() {
    // TODO
    // 1) transition particles uniformly random
    transition();
    // 2) Reweight (fire rays and check intersection)
    // 3) resample
}























