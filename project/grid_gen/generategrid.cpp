#include <iostream>
#include <vector>
#include "generategrid.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

using namespace std;

#define RING_SETUP_TAG 100

MPI_Datatype create_mpi_packet_dtype() {
    const int NUM_ELEMENTS = 2;
    int lengths[NUM_ELEMENTS] = {1, 1}; // number of ints/bools
    MPI_Aint displacements[NUM_ELEMENTS];     // offset from start of struct
    MPI_Aint base_address;
    MPI_Datatype types[NUM_ELEMENTS] = {MPI_INT, MPI_INT};
    position dummy;

    MPI_Get_address(&dummy, &base_address);
    MPI_Get_address(&dummy.x, &displacements[0]);
    MPI_Get_address(&dummy.y, &displacements[1]);
    // MPI_Get_address(&dummy.bend_cor, &displacements[2]);
    // MPI_Get_address(&dummy.is_hor_first, &displacements[3]);
    displacements[0] = MPI_Aint_diff(displacements[0], base_address);
    displacements[1] = MPI_Aint_diff(displacements[1], base_address);
    // displacements[2] = MPI_Aint_diff(displacements[2], base_address);
    // displacements[3] = MPI_Aint_diff(displacements[3], base_address);
    MPI_Datatype mpi_packet_t;

    MPI_Type_create_struct(NUM_ELEMENTS, lengths, displacements, types,
                           &mpi_packet_t);
    MPI_Type_commit(&mpi_packet_t);
    return mpi_packet_t;
}

node_t *initializeNode(int procID, int nproc) {

    MPI_Datatype mpi_packet_t = create_mpi_packet_dtype();
    node_t *newNode = (node_t *)calloc(1+1, sizeof(node_t)+2);
    newNode->procID = procID;
    newNode->dtype = mpi_packet_t;
    newNode->nproc = nproc;
    newNode->firstRecv = true;
    // printf("New Node initialized for proc: %d\n", procID);
    cout << "New Node initialized for proc:" << procID;

    return newNode;
}

void generateGridParallel(int procID, int nproc, int height, int width) {

    initializeNode(procID, nproc);

}