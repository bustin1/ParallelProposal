


#ifndef _UTIL_H
#define _UTIL_H

#include <bits/stdc++.h>
#include "grid.h"

#define PI 3.14159265
#define E  2.71828182 

int to_grid(int imageWidth, int i, int gridScale);
int to_img(int gridWidth, int i, int gridScale);
double rand_num();
double gaussian1d(double mean, double variance);
double gaussian2d(int mean, int imgWidth, double variance);
std::vector<int> bfs(int start, int goal, Grid* grid);
double clamp(double value, double threshold);
double double_mod(double value, double threshold);
void print_stats(int pos, int imgWidth, double angle, const char* name);
double dist2d(int p1, int p2, int imgWidth);

#endif
