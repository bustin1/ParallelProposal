
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <time.h>

#include "robot.h"

#include "refRenderer.h"
#include "grid.h"
#include "particleFilter.h"
#include "platformgl.h"


void startRendererWithDisplay(RefRenderer* renderer);


int main(int argc, char *argv[]) {

    char* inputFilename = NULL;
    bool help = false;
    int opt = 0;

    do {
        opt = getopt(argc, argv, "i:h:");
        switch(opt) {
        case 'i':
            inputFilename = optarg;
            break;
        case 'h':
            help = true;
        }
    } while (opt != -1);

    if (help) {
        printf("Usage: %s -i <filename>\n", argv[0]);
        return -1; 
    }

    FILE* fin = NULL;
    if (inputFilename != NULL) {
        printf("reading file: %s ...\n", inputFilename);
        fin = fopen(inputFilename, "r");
    }
    

    int w = 6;
    int h = 4;

    Grid* grid = new Grid();

    if (fin == NULL) {
        if (inputFilename != NULL) {
            printf("WARNING: Unable to read file: %s ... moving on\n", inputFilename);
            return -1;
        }
        printf("No file specified :(\n");
        grid->set_dim(w, h);
//    grid->create_simple_layout();
//    grid->create_checkered_layout();
//    grid->create_bullseye_layout(1,1);
        grid->create_simple_maze_layout();
    } else {
        fscanf(fin, "%d %d\n", &w, &h);
        int* tmp_grid = new int[w * h];
        for (int r=0; r<h; r++) {
            for (int c=0; c<w; c++) {
                int v;
                fscanf(fin, "%d", &v);
                tmp_grid[r*w+c] = v;
            }
        }
        grid->set_dim(w, h);
        grid->create_maze_from_file(tmp_grid);
        delete[] tmp_grid;
        fclose(fin);
    }



    const int gridScale = 25; // 20 pixels per grid element
    const int particleScale= 3; // width x height of particle
    const int numParticles = 10000; // 50 particles

    srand(time(NULL));

    Robot* robot = new Robot(grid, gridScale, 0, 0);
    Pfilter* filter = new Pfilter(robot, grid, numParticles, gridScale, particleScale);

    RefRenderer* renderer = new RefRenderer(filter);

    glutInit(&argc, argv);
    startRendererWithDisplay(renderer);



    return 0;
}



