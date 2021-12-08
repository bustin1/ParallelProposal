
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <time.h>


#include "include/robot.h"
#include "include/refRenderer.h"
#include "include/grid.h"
#include "include/particleFilter.h"
#include "include/platformgl.h"
#include "include/gridGenerator.h"


// in display.cpp
void startRendererWithDisplay(RefRenderer* renderer, int DEBUG, bool printStats);


int main(int argc, char *argv[]) {

    char* inputFilename = NULL;
    // char* outputFilename = NULL;
    bool help = false;
    int opt = 0;
    int numParticles = -1;
    int numRays = -1;
    int gridScale = -1;
    int debug = 0;
    bool printStats = false;

    do {
        opt = getopt(argc, argv, "i:h:n:d:g:r:p:");
        switch(opt) {
        case 'i':
            inputFilename = optarg;
            break;
        // case 'o':
        //     outputFilename = optarg;
        //     break;
        case 'h':
            help = true;
            break;
        case 'n':
            numParticles = atoi(optarg);
            break;
        case 'r':
            numRays = atoi(optarg);
            break;
        case 'g':
            gridScale = atoi(optarg);
            break;
        case 'd':
            debug = atoi(optarg);
            break;
        case 'p':
            printStats = true;
            break;
        }
    } while (opt != -1);

    help = (gridScale < 0) || (numRays < 0) || (numParticles < 0) || (inputFilename == NULL);
    if (help) {
        printf("Usage: %s\n", argv[0]);
        printf("-h: print this message\n");
        // printf("-o: <outputGridGenerator>");
        printf("-i: <inputFilename> File of 1s and 0s. 1 means wall, 0 means no wall. Example: tests/easy.txt\n");
        printf("-n: <numParticles> Number of particles to simulate. Example: 1000\n");
        printf("-r: <numRays> Number of rays out of each particle. Example: 20\n");
        printf("-g: <gridScale> Width and Height (px) of one wall. Example: 20\n");
        printf("-d: <debug>. \n\t0=no debug(default)\n\t1=debug with all ray particles\n\t2=debug with best particle\n");
        return 0; 
    }

    FILE* fin = NULL;
    if (inputFilename != NULL) {
        printf("Reading file: %s ... ", inputFilename);
        fin = fopen(inputFilename, "r");
        printf("done\n");
    }

    // FILE* grid_out = NULL;
    // if (outputFilename != NULL) {
    //     printf("Writing grid to file: %s ... ", outputFilename);
    //     grid_out = fopen(outputFilename, "w");
    //     printf("done\n");
    // }

    int w = 5;
    int h = 5; 

    Grid* grid = new Grid();
    const clock_t begin_time = clock();
    generateGrid(w, h);
    std::cout << "Grid generation time: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << "\n";

    if (fin == NULL) {
        printf("ERROR: Unable to read file: %s\n", inputFilename);
        return -1;
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


    const int particleScale= 3; // width x height of particle

    srand(time(NULL));

    Robot* robot = new Robot();
    Pfilter* filter = new Pfilter(robot, grid, numParticles, gridScale, 
                                    particleScale, numRays, debug);

    RefRenderer* renderer = new RefRenderer(filter);

    glutInit(&argc, argv);
    startRendererWithDisplay(renderer, debug, printStats);



    return 0;
}



