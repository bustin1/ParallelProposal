#include "mpi.h"
#include <assert.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <libgen.h>
#include <time.h>

#include "ring.h"
#include "generategrid.h"

#define RING_SETUP_TAG 100

MPI_Datatype create_mpi_packet_dtype() {
    const int NUM_ELEMENTS = 4;
    int lengths[NUM_ELEMENTS] = {1, 1, 1, 1}; // number of ints/bools
    MPI_Aint displacements[NUM_ELEMENTS];     // offset from start of struct
    MPI_Aint base_address;
    MPI_Datatype types[NUM_ELEMENTS] = {MPI_INT, MPI_INT, MPI_INT,
                                        MPI_CXX_BOOL};
    position dummy;

    MPI_Get_address(&dummy, &base_address);
    MPI_Get_address(&dummy.senderID, &displacements[0]);
    MPI_Get_address(&dummy.offset, &displacements[1]);
    MPI_Get_address(&dummy.bend_cor, &displacements[2]);
    MPI_Get_address(&dummy.is_hor_first, &displacements[3]);
    displacements[0] = MPI_Aint_diff(displacements[0], base_address);
    displacements[1] = MPI_Aint_diff(displacements[1], base_address);
    displacements[2] = MPI_Aint_diff(displacements[2], base_address);
    displacements[3] = MPI_Aint_diff(displacements[3], base_address);
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
    printf("New Node initialized for proc: %d\n", procID);

    return newNode;
}

// void connectToPrevNode(node_t *node, int nproc) {
//
//    int currentProcID = node->procID;
//    int prevProcID = (currentProcID - 1) % nproc;
//
//    int number;
//
//    MPI_Request north_request;
//
//    MPI_Recv(&number, 1, MPI_INT, prevProcID, RING_SETUP_TAG, MPI_COMM_WORLD,
//            MPI_STATUS_IGNORE);
//    printf("Process %d received number %d from process %d\n", currentProcID,
//            number, prevProcID);
// i

bool send_and_recv_in_ring(node_t *node, wire_packet_t *data, int length) {

    int currentProcID = node->procID;

    if (node->firstRecv) {
        MPI_Irecv(data, length, node->dtype, MPI_ANY_SOURCE, RING_SETUP_TAG,
                  MPI_COMM_WORLD, &(node->recv_request));
        node->firstRecv = false;
    }

    int flag = 0;
    MPI_Status status;
    MPI_Test(&(node->recv_request), &flag, &status);
    if (flag != 0) {
        // printf("source=%d Flag=%d In ring(%d) senderID: %d, offset = % d\n ",status.MPI_SOURCE, flag, node->procID, data[0].senderID, data[0].offset);
        node->firstRecv = true;
        if (data[0].senderID != currentProcID) {
            sendToNextNode(node, data, length);
            return true;
        } 
    }
    return false;
}

void receiveFromNextNode(node_t *node, wire_packet_t *data, int length) {

    int currentProcID = node->procID;

    // printf("Process %d received data of length %d\n", currentProcID, length);
    MPI_Recv(&data, length, node->dtype, MPI_ANY_SOURCE, RING_SETUP_TAG,
             MPI_COMM_WORLD, MPI_STATUS_IGNORE);
}

void sendToNextNode(node_t *node, wire_packet_t *data, int length) {

    int currentProcID = node->procID;
    int nextProcID = (currentProcID + 1) % (node->nproc);
    //    if (waitFirst) {
    //        printf("Process %d is sending data to process %d\n",
    //        currentProcID, nextProcID); MPI_Wait(&send_request,
    //        MPI_STATUS_IGNORE); printf("Process %d waited \n", currentProcID);
    //    }

    //    printf("Process %d sent data to process %d\n", currentProcID,
    //    nextProcID); MPI_Isend(data, length, node->dtype, nextProcID,
    //    RING_SETUP_TAG,
    //                MPI_COMM_WORLD, &send_request);
    MPI_Send(data, length, node->dtype, nextProcID, RING_SETUP_TAG,
             MPI_COMM_WORLD);
    // https://stackoverflow.com/questions/10017301/mpi-blocking-vs-non-blocking
    // https://peerj.com/articles/cs-95.pdf
}
