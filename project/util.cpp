

#include <stdlib.h>
#include "util.h"


// converts an image pixel to a grid location
// i := index of location in image space
int to_grid(int imageWidth, int i, int gridScale) {

    int iw = imageWidth;
    int gw = iw / gridScale;

    int x = i % iw;
    int y = i / iw;

    int gx = x / gridScale; 
    int gy = y / gridScale;

    return gx + gy*(gw);
}


// given converts a grid location to a image location (bot left)
// i := index of location in grid space
int to_img(int gridWidth, int i, int gridScale) {

    int gw = gridWidth;
    int iw = gw * gridScale;

    int x = i % gw;
    int y = i / gw;

    int ix = x * gridScale;
    int iy = y * gridScale;

    int j = iy*iw + ix;

    return j;
}


// TODO: put in helper functions
float rand_num() {
    return (float)rand()/RAND_MAX;
}
