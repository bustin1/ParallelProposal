


#ifndef _UTIL_H
#define _UTIL_H

#define PI 3.14159265
#define E  2.71828182 

int to_grid(int imageWidth, int i, int gridScale);
int to_img(int gridWidth, int i, int gridScale);
float rand_num();
int gaussian(int mean, int imgWidth, int variance);

#endif
