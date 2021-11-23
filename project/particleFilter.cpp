
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <algorithm>    // std::min

#include "particleFilter.h"
#include "grid.h"
#include "util.h"

#define PI 3.14159265


Pfilter::Pfilter(Grid* g, int n, int s1, int s2) : numRays (10) {

    numParticles = n;
    particleLocations = new int[n];
    rays = new int[numRays * n]; // each particle gets numRays;
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
    }
    

}

Pfilter::~Pfilter() {
    if (particleLocations) {
        delete particleLocations;
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


int Pfilter::getIntersection(int ploc, double angle) {

    int gw = grid->width;
    int gh = grid->height;
    int iw = gw * gridScale;

    int num_closed = gw * gh - grid->num_open;
    int gscale = gridScale;
    int* closed = grid->closed;

    double m = tan(angle);
    int closestX = INT_MAX;
    int closestY = INT_MAX; 
    double closestDist = INT_MAX;


    int px = ploc % (iw);
    int py = ploc / (iw);

    double dir_x = cos(angle);
    double dir_y = sin(angle);

    for (int i=0; i<num_closed; i++) {

        int wloc = closed[i];

        int left_x = wloc % gw * gscale;
        int bot_y = wloc / gw * gscale;
        int right_x = left_x + gscale;
        int top_y = bot_y + gscale;

        double b = (double)py - m*(double)px;
            
        double left_int = m * left_x + b; // NOTE: this returns a y value on the left side of the wall
        double bot_int = (bot_y - b) / m;
        double right_int = m * right_x + b;
        double top_int = (top_y - b) / m;


        double dist_bot = (bot_int-px)*(bot_int-px) + (bot_y-py)*(bot_y-py);
        double dist_top = (top_int-px)*(top_int-px) + (top_y-py)*(top_y-py);
        double dist_left = (left_x-px)*(left_x-px) + (left_int-py)*(left_int-py);
        double dist_right = (right_x-px)*(right_x-px) + (right_int-py)*(right_int-py);

        if (dist_bot < closestDist) {
            if (dir_y*(bot_y-py) >= 0) {
                closestX = round(bot_int);
                closestY = round(bot_y);
                closestDist = dist_bot;
            }
        }
        if (dist_top < closestDist) {
            if (dir_y*(top_y-py) >= 0) {
                closestX = round(top_int);
                closestY = round(top_y);
                closestDist = dist_top;
            }
        }
        if (dist_left < closestDist) {
            if (dir_x*(left_x-px) >= 0) {
                closestX = round(left_x);
                closestY = round(left_int);
                closestDist = dist_left;
            }
        }
        if (dist_right < closestDist) {
            if (dir_x*(right_x-px) >= 0) {
                closestX = round(right_x);
                closestY = round(right_int);
                closestDist = dist_right;
            }
        }
    }

    return closestY * iw + closestX;
}



void Pfilter::firerays(int loc, int* ray) {

    const double dtheta = 2 * PI / (double)numRays;
    for (int i=0; i<numRays; i++) {
        double angle = ((double)i) * dtheta + .01f;
        int coord = getIntersection(loc, angle);
        *ray = coord;
        ray++;
    }
     
}


void Pfilter::reweight() {
    
    for (int i=0; i<numParticles; i++) {
        int loc = particleLocations[i];
        firerays(loc, rays + i*numRays);
    }

}


void Pfilter::update() {
    // TODO
    // 1) transition particles uniformly random
    transition();
    // 2) Reweight (fire rays and check intersection)
    reweight(); 
    // 3) resample
}























