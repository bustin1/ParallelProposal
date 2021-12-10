


#ifndef _UTIL_H
#define _UTIL_H

#include <bits/stdc++.h>
#include "grid.h"

#define PI 3.14159265
#define E  2.71828182 

int to_grid(int imgWidth, int i, int gridScale);
int to_img(int gridWidth, int i, int gridScale);

int get_x(int pos, int imgWidth);
int get_y(int pos, int imgWidth);
int interpolate(int start, int end, int imgWidth, double t);

float rand_num();
float rand_angle();
double gaussian1d(double mean, double variance);
double gaussian2d(int mean, int imgWidth, double variance);
double clamp(double value, double threshold);
double double_mod(double value, double threshold);
double dist2d(int p1, int p2, int imgWidth);

std::vector<int> bfs(int start, int goal, Grid* grid);

void print_stats(int pos, int imgWidth, int gridScale, double angle, const char* name);
void print_loc(char* function_name, int pos, int width);

#endif
