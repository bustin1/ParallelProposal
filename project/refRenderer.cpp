
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <vector>

#include "include/refRenderer.h"



RefRenderer::RefRenderer(Pfilter* f) {
    filter = f;
    Grid* grid = filter->get_grid();
    int w = grid->width;
    int h = grid->height;
    int scale = filter->get_grid_scale();
    image = new Image(w * scale, h * scale);


}

RefRenderer::~RefRenderer() {
    if (image) {
        delete image;
    }
}

const Image* RefRenderer::get_image() {
    return image;
}

Pfilter* RefRenderer::get_filter() {
    return filter;
}


void RefRenderer::clearImage(int numThreads) {
    Grid* grid = filter->get_grid();
    int scale = filter->get_grid_scale();
    image->clear(grid, scale, to_grid, numThreads);
}

void RefRenderer::advanceAnimation() {
    filter->update();
}


void RefRenderer::render() {
    float* data = image->data;
    int* pLoc = filter->get_particle_locations();
    int numParticles = filter->get_num_particles();
    int particleScale = filter->get_particle_scale();
    int imageWidth = image->width;

    // 1) draws the particles
    for (int i=0; i<numParticles; i++) {
        int loc = 4 * pLoc[i];
        for (int y=0; y<particleScale; y++) {
            for (int x=0; x<particleScale; x++) {
                int pos = loc + 4 * (y * imageWidth) + 4 * x;
                data[pos] = 1;
                data[pos+1] = 0;
                data[pos+2] = 0;
                data[pos+3] = 1;
            }
        }
    }

    int goal = filter->get_goal() * 4;
    int goalScale = filter->get_goal_scale();

    // 2) set location of goal
    
    int goalSize = goalScale * particleScale;
    for (int y=-goalSize/2; y<goalSize/2; y++) {
        for (int x=-goalSize/2; x<goalSize/2; x++) {
            int pos = goal + 4 * (y * imageWidth) + 4 * x;
            data[pos] = 1;
            data[pos+1] = 0;
            data[pos+2] = 0;
            data[pos+3] = 1;
        }
    }

    // 3) draw robot
    Robot* robot = filter->get_robot();
    int robotScale = robot->get_scale()*particleScale;
    int robopos = robot->get_pos() * 4;
    for (int y=-robotScale/2; y<robotScale/2; y++) {
        for (int x=-robotScale/2; x<robotScale/2; x++) {
            int pos = robopos + 4 * (y * imageWidth) + 4 * x;
            data[pos] = 0;
            data[pos+1] = 0;
            data[pos+2] = 1;
            data[pos+3] = 1;
        }
    }

}

void RefRenderer::dumpWalls(const char* filename) {
    FILE* output = fopen(filename, "w");
    
    Grid* grid = filter->get_grid();
    int w = grid->width;
    int h = grid->height;
    fprintf(output, "%d %d\n", w, h);

    for (int r=0; r<h; r++) {
        for (int c=0; c<w; c++) {
            fprintf(output, "%d", grid->walls[r*w+c]);
        }
        fprintf(output, "\n");
    }

}


