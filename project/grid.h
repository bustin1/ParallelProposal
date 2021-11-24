
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

    Grid() {
    }

    Grid(int w, int h) {
        width = w;
        height = h;
        walls = new int[w*h]; // 1 means wall, 0 other wise
        num_open = w*h;
    }

    void set_dim(int w, int h) {
        width = w;
        height = h;
        if (walls) delete walls;
        walls = new int[w*h]; // 1 means wall, 0 other wise
        num_open = w*h;
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
        closed = new int[width*height-num_open];
    }

    void create_simple_layout() {

        for (int c=0; c<width; c++) {
            walls[c] = 1;
            walls[(height-1)*width + c] = 1;
            num_open -= 2;
        }
        for (int r=1; r<height-1; r++) {
            walls[r * width] = 1;
            walls[r * width + width-1] = 1;
            num_open -= 2;
        }
        walls[width+1] = 1;
        num_open --;

        open = new int[num_open];
        closed = new int[width*height-num_open];
    }

    void create_bullseye_layout(int x, int y) {

        for (int r=0; r<height; r++) {
            for (int c=0; c<width; c++) {
                walls[r*width + c] = 1;
                num_open--;
            }
        }
        num_open++;
        walls[y*width+x] = 0;

        open = new int[num_open];
        closed = new int[width*height-num_open];
    }

    void create_simple_maze_layout() {

        for (int r=0; r<height; r++) {
            for (int c=0; c<width; c++) {
                if (r != height/2 || c == 0 || c == width-1) {
                    walls[r*width + c] = 1;
                    num_open--;
                }
            }
        }
        num_open++;
        walls[width + width/2] = 0;

        open = new int[num_open];
        closed = new int[width*height-num_open];
    }

    void create_maze_from_file(int* grid) {

        for (int r=0; r<height; r++) {
            for (int c=0; c<width; c++) {
                int wall = grid[r*width + c];
                walls[r*width + c] = wall;
                num_open -= wall;
            }
        }
        open = new int[num_open];
        closed = new int[width*height-num_open];

    }

    // this must be called before random sampling
    void calc_num_open() {
        int gridSize = width*height;
        int j = 0;
        for (int i=0; i<gridSize; i++) {
            if (walls[i] == 0){
                open[j++] = i;
            }
        }
    }

    // calc num of walls
    void calc_num_closed() {
        int gridSize = width*height;
        int j = 0;
        for (int i=0; i<gridSize; i++) {
            if (walls[i] == 1){
                closed[j++] = i;
            }
        }
    }


    Wall* walls;
    int* open;
    int* closed;
    int width;
    int height;
    int num_open;

};


#endif
