
#ifndef  __IMAGE_H__
#define  __IMAGE_H__

#include "grid.h"

/*
 * The image struct should only know it's image surface and how to clear it
 * This is why it only contains, height, width and data
 */

struct Image {

    Image(int w, int h) {
        width = w;
        height = h;
        data = new float[4 * width * height];
    }

    // redraws the image with the maze
    void clear(Grid* grid, int gridScale, int (*to_grid)(int, int, int)) {

    }

    int width;
    int height;
    float* data;
};


#endif
