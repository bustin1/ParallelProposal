
#ifndef _GRID_H
#define _GRID_H

#include <stdio.h>

typedef int Wall;

struct Grid {

    Grid(int w, int h) {
        width = w;
        height = h;
        walls = new int[w*h]; // 1 means wall, 0 other wise
        num_open = w*h;
    }

    // check if a wall is at a location
    bool is_wall_at(int i) {
        return walls[i] == 1;
    }

    // reset all walls
    void clear() {
        int numWalls = width * height;
        for (int i=0; i<numWalls; i++) {
            walls[i] = 0;
        }
    }

    // call these functions to initlize the grid;
    void create_checkered_layout() {

        for (int r=0; r<height; r++) {
            for (int c=0; c<width; c++) {
                if ((r+c) % 2 == 1) {
                    walls[r*width + c] = 1;
                    num_open--;
                }
            }
        }

        open = new int[num_open];
        // calc_num_open();
    }

    void create_simple_layout() {

        for (int c=0; c<width; c++) {
            walls[c] = 1;
            walls[(height-1)*width + c] = 1;
            num_open -= 2;
        }
        for (int r=0; r<height; r++) {
            walls[r * width] = 1;
            walls[r * width + width-1] = 1;
            num_open -= 2;
        }
        walls[width+1] = 1;

        open = new int[num_open-1];
        // calc_num_open();
    }

    void calc_num_open() {
        int gridSize = width*height;
        int j = 0;
        for (int i=0; i<gridSize; i++) {
            if (walls[i] == 0){
                open[j++] = i;
            }
        }
    }

    int to_img(int i, int scale) {
        int x = i % width;
        int y = i / width;
        printf("x: %d\n", x);
        printf("scale: %d\n", scale);
        int ix = x * scale;
        int iy = y * scale;
        int j = iy*width*scale + ix;
        return j;
    }

    Wall* walls;
    int* open;
    int width;
    int height;
    int num_open;

};


#endif
