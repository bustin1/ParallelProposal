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
    cout << "New Node initialized for proc:" << procID << "\n";

    return newNode;
}

void add_nearby_wall_to_list(int height, int width, position removed_wall, vector<position>& nearby_wall_list, std::vector<std::vector<int> >& newgrid, std::vector<std::vector<int> >& visitedgrid) {

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

    if (y < height - 1) {
        position x1;
        x1.x = x;
        x1.y = y+1;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0) {
            nearby_wall_list.push_back(x1);
        }
    }
}

void printgrid(int height, int width, std::vector<std::vector<int> >& newgrid) {

    std::cout << "\n";

    for (int i=0; i<newgrid.size(); i++) {
        for (int j=0; j<width; j++) {
            std::cout << newgrid[i][j] << " ";
        }
        std::cout << "\n";
    }
}

// Helper function for validation
void validate_ghost_row(node_t *node) {
    for (int i = 0; i<length; i++) {
        position curPos = data[i];
        if (curPos.x != -1) {
            cout << "Pos is 0: " << curPos.y << "\n";
        }
    }
}

void send_ghost_row(node_t *node, position *data, int length) {

    int currentProcID = node->procID;
    int nextProcID = (currentProcID - 1);

    if (currentProcID == 0) {
        return;
    }

    MPI_Isend(data, length, node->dtype, nextProcID, RING_SETUP_TAG,
             MPI_COMM_WORLD, &(node->send_request));
    // https://stackoverflow.com/questions/10017301/mpi-blocking-vs-non-blocking
    // https://peerj.com/articles/cs-95.pdf
}

bool recieve_ghost_row(node_t *node, position *data, int length, int nproc) {

    int currentProcID = node->procID;

    // Last row won't receive any data
    if (currentProcID == nproc - 1) {
        return true;
    }

    MPI_Recv(data, length, node->dtype, MPI_ANY_SOURCE, RING_SETUP_TAG,
                MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    cout << "Node " << currentProcID << " recieved data!!" << "\n";

    return true;
}

void generateGridParallel(int procID, int nproc, int height, int width) {

    srand (time(NULL) + procID);

    if (height % nproc != 0) {
        cout << "height not divisible by nproc" << "\n";
        return;
    }

    node_t *currentNode = initializeNode(procID, nproc);

    // Start of the grid
    int start_height = (height / nproc) * procID;
    int end_height = ((height / nproc) * (procID + 1)) + 1;   // add +1 for ghost row

    cout << "proc: " << procID << ", start_height" << start_height << ", end_height" << end_height << "\n";

    std::vector<std::vector<int> > newgrid(end_height-start_height, std::vector<int>(width));
    std::vector<std::vector<int> > visitedgrid(end_height-start_height, std::vector<int>(width));
    vector<position> nearby_wall_list;

    cout << "vectors size: " << newgrid[0].size() << "\n";

    for (int i=0; i<newgrid.size(); i++) {
        for (int j=0; j<width; j++) {
            newgrid[i][j] = 1;
            visitedgrid[i][j] = 0;
        }
    }
    
    position *send_packets =
        (position *)calloc(width+1, sizeof(position));
    position *recv_packets =
        (position *)calloc(width+1, sizeof(position));

    int num_of_packets = 0;

    // Process ghost row
    for (int i=0; i<width; i++) {
        int val = rand() % 2;

        if (val == 0) {
            position init_position;
            init_position.x = 0;
            init_position.y = i;

            newgrid[init_position.x][init_position.y] = 0;
            visitedgrid[init_position.x][init_position.y] = 1;

            add_nearby_wall_to_list(height, width, init_position, nearby_wall_list, newgrid, visitedgrid);

            send_packets[i] = init_position;
        } else {
            // Position is -1
            position init_position;
            init_position.x = -1;
            init_position.y = -1;

            send_packets[i] = init_position;
        }
    }

    printgrid(end_height - start_height, width, newgrid);

    send_ghost_row(currentNode, send_packets, width);

    recieve_ghost_row(currentNode, recv_packets, width, nproc);

}