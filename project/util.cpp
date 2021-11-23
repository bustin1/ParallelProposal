

#include <stdlib.h>
#include <math.h>
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



// samples a random 2d point with variance and mean
int gaussian(int mean, int imgWidth, int variance) {

    int x = mean % imgWidth;
    int y = mean / imgWidth;
    
    double v1 = rand_num();
    double v2 = rand_num();

    double newx = cos(2*PI*v2) * sqrt(-2*log(v1));
    double newy = sin(2*PI*v2) * sqrt(-2*log(v1));

    newx = newx * variance + x;
    newy = newy * variance + y;

    return round(newx + newy * imgWidth);
}



















