
#ifndef  __IMAGE_H__
#define  __IMAGE_H__

#include "grid.h"

struct Image {

    Image(int w, int h) {
        width = w;
        height = h;
        data = new float[4 * width * height];
    }

    // converts an image pixel to a grid location
    int to_grid(Grid* grid, int scale, int i) {

        int x = i % width;
        int y = i / width;

        int gx = x / scale; 
        int gy = y / scale;

        return gx + gy*(grid->width);
    }

    // redraws the image with the maze
    void clear(Grid* grid, int scale) {

        int numPixels = width * height;

        float* ptr = data;
        for (int i=0; i<numPixels; i++) {
            ptr[0] = 1;
            ptr[1] = 1;
            ptr[2] = 1;
            ptr[3] = 1;

            int g = to_grid(grid, scale, i);
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
