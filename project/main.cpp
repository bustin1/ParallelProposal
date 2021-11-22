
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <time.h>

#include "refRenderer.h"
#include "grid.h"
#include "particleFilter.h"
#include "platformgl.h"


void startRendererWithDisplay(RefRenderer* renderer);


int main(int argc, char *argv[]) {

    char* outputFilename = NULL;
    int opt = 0;

    do {
        opt = getopt(argc, argv, "f:");
        switch(opt) {
        case 'f':
            outputFilename = optarg;
            break;
        }
    } while (opt != -1);

    if (outputFilename == NULL) {
        printf("Usage: %s -f <filename>\n", argv[0]);
        return -1; 
    }

    const int w = 3;
    const int h = 3;
    const int gridScale = 100; // 20 pixels per grid element
    const int particleScale= 2; // width x height of particle
    const int numParticles = 10; // 50 particles

    srand(time(NULL));

    Grid* grid = new Grid(w, h);
    grid->clear();
//    grid->create_simple_layout();
//    grid->create_checkered_layout();
    grid->create_bullseye_layout(1,1);

    Pfilter* filter = new Pfilter(grid, numParticles, gridScale, particleScale);

    RefRenderer* renderer = new RefRenderer(filter);

    renderer->dumpWalls(outputFilename);
    glutInit(&argc, argv);
    startRendererWithDisplay(renderer);


    return 0;
}



