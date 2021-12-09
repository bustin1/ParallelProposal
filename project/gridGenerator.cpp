
#include <iostream>
#include <vector>
#include <random>
#include "include/gridGenerator.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


using namespace std;

// // struct position {
// //     int x;
// //     int y;
// // };

std::mt19937 rng_seed;

void add_nearby_wall_to_list(int height, int width, position removed_wall, vector<position>& nearby_wall_list, std::vector<std::vector<int>>& newgrid, std::vector<std::vector<int>>& visitedgrid) {

    int x = removed_wall.x;
    int y = removed_wall.y;

    if (x > 0) {
        position x1;
        x1.x = x-1;
        x1.y = y;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0) {
            nearby_wall_list.push_back(x1);
        }
    }

    if (x < height - 1) {
        position x1;
        x1.x = x+1;
        x1.y = y;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0) {
            nearby_wall_list.push_back(x1);
        }
    }

    if (y > 0) {
        position x1;
        x1.x = x;
        x1.y = y-1;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0) {
            nearby_wall_list.push_back(x1);
        }
    }

    if (y < width - 1) {
        position x1;
        x1.x = x;
        x1.y = y+1;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0) {
            nearby_wall_list.push_back(x1);
        }
    }
}

void printgrid(int height, int width, std::vector<std::vector<int>>& newgrid) {

    std::cout << "\n";

    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            std::cout << newgrid[i][j] << " ";
        }
        std::cout << "\n";
    }

}

bool canEvictWall(position wall_selection, int height, int width, std::vector<std::vector<int>>& newgrid) {

    int x = wall_selection.x;
    int y = wall_selection.y;

    int nearby_wall_counters = 0;

    // Check up
    if (x-1 >= 0) {
        if (newgrid[x-1][y] == 0) {
            nearby_wall_counters++;
        }
    }

    // Check down
    if (x+1 <= height-1) {
        if (newgrid[x+1][y] == 0) {
            nearby_wall_counters++;
        }
    }

    // Check left
    if (y-1 >= 0) {
        if (newgrid[x][y-1] == 0) {
            nearby_wall_counters++;
        }
    }

    // Check right
    if (y+1 <= width-1) {
        if (newgrid[x][y+1] == 0) {
            nearby_wall_counters++;
        }
    }

    if (nearby_wall_counters > 1){
        return false;
    }

    return true;
}

int generateRandomNumberInRange(int end) {

    if (end == 0) {
        return 0;
    }

    int iSecret = rand() % end + 1;
    return iSecret;
}

void initRandomizer() {
    // std::random_device dev;
    // std::mt19937 rng(dev());

    // return rng;

    // rng_seed.seed(1000);
    srand (time(NULL));
}

void generateGrid(int height, int width) {

    initRandomizer();
    // generateRandomNumberInRange(rng_seed);

    width = 4000;
    height = 4000;

    
    // int initial_point_h = 0, initial_point_w=0;

    // struct position wall_list[height*width];    // Every possible wall
    // int wall_count = 0;

    // for (int i=0; i<height; i++) {
    //     for (int j=0; j<width; j++) {
    //         newgrid[i][j] == 1;
    //     }
    // }

    // int newgrid[height][width];
    std::vector<std::vector<int>> newgrid(height, std::vector<int>(width));
    std::vector<std::vector<int>> visitedgrid(height, std::vector<int>(width));
    vector<position> nearby_wall_list;


    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            newgrid[i][j] = 1;
            visitedgrid[i][j] = 0;
        }
    }

    position init_position;
    init_position.x = 0;
    init_position.y = 0;

    newgrid[init_position.x][init_position.y] = 0;
    visitedgrid[init_position.x][init_position.y] = 1;

    add_nearby_wall_to_list(height, width, init_position, nearby_wall_list, newgrid, visitedgrid);

    std::cout << "vector size " << nearby_wall_list.size() << " \n";
    for (uint i=0; i<nearby_wall_list.size(); i++) {
        std::cout << "vector " << nearby_wall_list[i].x << ", " << nearby_wall_list[i].y << " \n";
    }

    int num_of_mistakes = 0;

    while (nearby_wall_list.size() > 0) {

        int random_wall_idx = generateRandomNumberInRange(nearby_wall_list.size()-1);
        // std::cout << "wall_eviction: " << random_wall_idx << "\n";

        // printgrid(height, width, newgrid);

        position selected_wall = nearby_wall_list.at(random_wall_idx);
        if (visitedgrid[selected_wall.x][selected_wall.y] == 1) {
            nearby_wall_list.erase(nearby_wall_list.begin() + random_wall_idx);
            num_of_mistakes++;
            continue;
        }

        bool canEvict = canEvictWall(selected_wall, height, width, newgrid);

        // std::cout << "canEvict: " << canEvict << "\n";

        if (canEvict) {
            newgrid[selected_wall.x][selected_wall.y] = 0;
            add_nearby_wall_to_list(height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid);
        }

        visitedgrid[selected_wall.x][selected_wall.y] = 1;
        nearby_wall_list.erase(nearby_wall_list.begin() + random_wall_idx);
    }
    printf("\nNum mistakes: %d\n", num_of_mistakes);
}