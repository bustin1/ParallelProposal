
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <vector>

#include "refRenderer.h"
#include "grid.h"
#include "util.h"


RefRenderer::RefRenderer(Pfilter* f) {
    filter = f;
    Grid* grid = filter->getGrid();
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

const Image* RefRenderer::getImage() {
    return image;
}


void RefRenderer::clearImage() {
    Grid* grid = filter->getGrid();
    int scale = filter->get_grid_scale();
    image->clear(grid, scale, to_grid);
}

void RefRenderer::advanceAnimation() {
    filter->update();
}

void RefRenderer::render() {
    float* data = image->data;
    int* pLoc = filter->get_particleLocations();
    int numParticles = filter->get_numParticles();
    int particleScale = filter->get_particle_scale();
    int imageWidth = image->width;

    // draws the particles
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

}

void RefRenderer::dumpWalls(const char* filename) {
    FILE* output = fopen(filename, "w");
    
    Grid* grid = filter->getGrid();
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


