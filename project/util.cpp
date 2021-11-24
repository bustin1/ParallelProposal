

#include <stdlib.h>
#include <math.h>
#include <bits/stdc++.h>
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
double rand_num() {
    return (double) rand()/RAND_MAX;
}


double gaussian1d(double mean, double variance) {

    double v1 = rand_num();
    double v2 = rand_num();

    double newx = cos(2*PI*v2) * sqrt(-2*log(v1));

    newx = newx * variance + mean;

    return newx;
}

// samples a random 2d point with variance and mean
double gaussian2d(int mean, int imgWidth, double variance) {

    int x = mean % imgWidth;
    int y = mean / imgWidth;
    
    double newx = gaussian1d(x, variance);
    double newy = gaussian1d(y, variance);

    return newx + round(newy) * imgWidth;
}


// start and goal and image locations, NOT grid locations
std::vector<int> bfs(int start, int goal, Grid* grid)
{
    int gridSize = grid->width * grid->height;
    std::queue<std::vector<int> > q;
    std::vector<bool> visited(gridSize, false);
    
    std::vector<int> startpath(1, start);
    q.push(startpath);

    while (!q.empty()) {
        std::vector<int> nextpath = q.front();
        int lastNode = nextpath.back(); // only check last node in path
        if (lastNode == goal) {
            return nextpath; // return the best path
        }

        q.pop();
        visited[lastNode] = true;

        // find all neighbors
        for (int r=-1; r<=1; r++) {
            for (int c=-1; c<=1; c++) {

                if ((r + c + 2) % 2 == 1) {
                    int x = (lastNode) % grid->width + c; 
                    int y = (lastNode) / grid->width + r;
                    int neighbor = y*grid->width + x;

                    if (!visited[neighbor]) {
                        if (!grid->is_wall_at(neighbor)) {
                            std::vector<int> copy = nextpath; // create a copy of the original path
                            copy.push_back(neighbor);
                            q.push(copy);
                        }
                    }
                }
            }
        }
    }

    return startpath;

}


double clamp(double value, double threshold) {
    if (value > threshold) return threshold;
    if (value < -threshold) return -threshold;
    return value;
}

double double_mod(double value, double threshold) {
    if (value > threshold) value -= threshold;
    if (value < -threshold) value += threshold;
    return value;
}

void print_stats(int pos, int imgWidth, double angle, const char* name) {
    printf("%s is @ pos %d,%d facing %f degrees\n", name, pos%imgWidth, pos/imgWidth, angle);
}














