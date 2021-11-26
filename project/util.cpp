

#include <stdlib.h>
#include <math.h>

#include "include/util.h"


// converts an image pixel to a grid location
// i := index of location in image space
int to_grid(int imgWidth, int i, int gridScale) {

    int gridWidth = imgWidth / gridScale;

    int x = i % imgWidth;
    int y = i / imgWidth;

    int gx = x / gridScale; 
    int gy = y / gridScale;

    return gx + gy*gridWidth;

}


// given converts a grid location to a image location (bot left)
// i := index of location in grid space
int to_img(int gridWidth, int i, int gridScale) {

    int imgWidth = gridWidth * gridScale;

    int x = i % gridWidth;
    int y = i / gridWidth;

    int ix = x * gridScale;
    int iy = y * gridScale;

    return iy*imgWidth + ix;

}


// returns a random number between 0 and 1
double rand_num() {
    return (double) rand()/RAND_MAX;
}


// returns a gaussian 1d random number
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


// TODO: parallelisze this?
// start and goal and image are grid locations
std::vector<int> bfs(int start, int goal, Grid* grid) {

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

                    if (neighbor >= 0 && neighbor < gridSize && !visited[neighbor]) {
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


// value is clamped from [-threshold, threshold]
double clamp(double value, double threshold) {
    if (value > threshold) return threshold;
    if (value < -threshold) return -threshold;
    return value;
}

// mods a double 
double double_mod(double value, double threshold) {
    if (value > threshold) value -= threshold;
    if (value < -threshold) value += threshold;
    return value;
}

// print stats about the robot or particle
void print_stats(int pos, int imgWidth, double angle, const char* name) {
    printf("%s is @ pos %d,%d facing %f degrees\n", name, pos%imgWidth, pos/imgWidth, angle * 180 / PI);
}


// distance between two points in image space
double dist2d(int p1, int p2, int imgWidth) {

    int x1 = p1 % imgWidth;
    int y1 = p1 / imgWidth;
    
    int x2 = p2 % imgWidth;
    int y2 = p2 / imgWidth;

    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));

}

// returns a random angle
double rand_angle() {
    return 2.0f * PI * rand_num();
}

int get_x(int i, int imgWidth) {
    return i % imgWidth;
}

int get_y(int i, int imgWidth) {
    return i / imgWidth;
}

int interpolate(int start, int end, int imgWidth, double t) {

    int sx = get_x(start, imgWidth);
    int sy = get_y(start, imgWidth);

    int ex = get_x(end, imgWidth);
    int ey = get_y(end, imgWidth);

    int newX = sx+(ex-sx)*t;
    int newY = sy+(ey-sy)*t;

    return newX + newY*imgWidth;

}


// print of location if width=gridWidth then grid location
void print_loc(char* function_name, int pos, int width) {
    printf("In function %s: pos=%d,%d\n", function_name, pos%width, pos/width);
}




