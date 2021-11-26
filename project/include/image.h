
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

        int numPixels = width * height;

        float* ptr = data;
        for (int i=0; i<numPixels; i++) {
            ptr[0] = 1;
            ptr[1] = 1;
            ptr[2] = 1;
            ptr[3] = 1;

            int g = to_grid(width, i, gridScale);
            if (grid->is_wall_at(g)) {
                ptr[0] = 0;
                ptr[1] = 0;
                ptr[2] = 0;
                ptr[3] = 0;
            }

            ptr += 4;
        }
    }

    int width;
    int height;
    float* data;
};


#endif