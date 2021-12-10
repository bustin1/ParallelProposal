
#ifndef _GRID_H
#define _GRID_H

#include <stdio.h>

typedef int Wall;

/*
 * The grid struct should only know it's grid map. It should be able to check
 * whether a wall exists, as well as creating walls, and calculating open
 * space in the grid
 */

struct Grid {

    Grid() {}

    void set_dim(int w, int h) {
        width = w;
        height = h;
        if (walls) delete walls;
        walls = new int[w*h]; // 1 means wall, 0 other wise
        num_open = w*h;
        num_closed = 0;
    }

    // check if a wall is at a location
    bool is_wall_at(int i) {
        if (i < 0 or i >= width * height) return true;
        return walls[i] == 1;
    }

    // reset all walls
    void clear() {
        int numWalls = width * height;
        for (int i=0; i<numWalls; i++) {
            walls[i] = 0;
        }
    }

    void create_maze_from_file(int* grid) {

        for (int r=0; r<height; r++) {
            for (int c=0; c<width; c++) {
                int wall = grid[r*width + c];
                walls[(height-1-r)*width + c] = wall;
                num_open -= wall;
            }
        }
        open = new int[num_open];
        num_closed = width*height-num_open;
        closed = new int[num_closed];

    }

    // this must be called before random sampling
    void init() {
        calc_num_open();
        calc_num_closed();
    }

    void calc_num_open() {
        int gridSize = width*height;
        int j = 0;
        for (int i=0; i<gridSize; i++) {
            if (walls[i] == 0){
                open[j++] = i;
            }
        }
        num_open = j;
    }

    void calc_num_closed() {
        int gridSize = width*height;
        int j = 0;
        for (int i=0; i<gridSize; i++) {
            if (walls[i] == 1){
                closed[j++] = i;
            }
        }
        num_closed = j;
    }


    Wall* walls;
    int* open;
    int* closed;
    int width;
    int height;
    int num_open;
    int num_closed;

};


#endif
