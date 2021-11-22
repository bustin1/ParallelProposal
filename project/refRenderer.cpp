
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
    int n = filter->get_numParticles();

    // draws the particles
    for (int i=0; i<n; i++) {
        int loc = pLoc[i];
        data[4*loc] = 1;
        data[4*loc+1] = 0;
        data[4*loc+2] = 0;
        data[4*loc+3] = 1;
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


