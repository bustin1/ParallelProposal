
#ifndef  __IMAGE_H__
#define  __IMAGE_H__

#include <omp.h>
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
        #pragma omp parallel for shared(ptr)
        for (int i=0; i<numPixels; i++) {
            ptr[i*4] = 1;
            ptr[i*4+1] = 1;
            ptr[i*4+2] = 1;
            ptr[i*4+3] = 1;

            int g = to_grid(width, i, gridScale);
            if (grid->is_wall_at(g)) {
                ptr[i*4] = 0;
                ptr[i*4+1] = 0;
                ptr[i*4+2] = 0;
                ptr[i*4+3] = 0;
            }

        }
    }

    int width;
    int height;
    float* data;
};


#endif
