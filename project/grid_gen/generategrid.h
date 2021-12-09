#ifndef GEN_GRID_PARALLEL_H
#define GEN_GRID_PARALLEL_H

#include "mpi.h"

typedef struct  {
    int x;
    int y;
} position;

typedef struct ringNode {
    int procID;
    int nproc;
    bool firstRecv;
    MPI_Request send_request;
    MPI_Request recv_request;
    MPI_Datatype dtype;
} node_t;

void generateGridParallel(int procID, int nproc, int height, int width);

#endif