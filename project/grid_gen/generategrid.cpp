#include <iostream>
#include <vector>
#include "generategrid.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <stdio.h>

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
    cout << "New Data Node initialized for proc:" << procID << "\n";

    return newNode;
}

MPI_Datatype create_mpi_packet_status_dtype() {
    const int NUM_ELEMENTS = 1;
    int lengths[NUM_ELEMENTS] = {1}; // number of ints/bools
    MPI_Aint displacements[NUM_ELEMENTS];     // offset from start of struct
    MPI_Aint base_address;
    MPI_Datatype types[NUM_ELEMENTS] = {MPI_INT};
    grid_status dummy;

    MPI_Get_address(&dummy, &base_address);
    MPI_Get_address(&dummy.status, &displacements[0]);
    // MPI_Get_address(&dummy.bend_cor, &displacements[2]);
    // MPI_Get_address(&dummy.is_hor_first, &displacements[3]);
    displacements[0] = MPI_Aint_diff(displacements[0], base_address);
    // displacements[1] = MPI_Aint_diff(displacements[1], base_address);
    // displacements[2] = MPI_Aint_diff(displacements[2], base_address);
    // displacements[3] = MPI_Aint_diff(displacements[3], base_address);
    MPI_Datatype mpi_packet_t;

    MPI_Type_create_struct(NUM_ELEMENTS, lengths, displacements, types,
                           &mpi_packet_t);
    MPI_Type_commit(&mpi_packet_t);
    return mpi_packet_t;
}

node_t *initializeNodeWithCompletion(int procID, int nproc) {

    MPI_Datatype mpi_packet_t = create_mpi_packet_dtype();
    node_t *newNode = (node_t *)calloc(1+1, sizeof(node_t)+2);
    newNode->procID = procID;
    newNode->dtype = mpi_packet_t;
    newNode->nproc = nproc;
    newNode->firstRecv = true;
    cout << "New Status Node initialized for proc:" << procID << "\n";

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

    if (y < width - 1) {
        position x1;
        x1.x = x;
        x1.y = y+1;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0) {
            nearby_wall_list.push_back(x1);
        }
    }
}

bool isSkip(position cur_pos, int skip_x, int skip_y) {
    if (cur_pos.x == skip_x && cur_pos.y == skip_y) {
        return true;
    }
    return false;
}

void add_nearby_wall_to_list_with_skip(int height, int width, position removed_wall, vector<position>& nearby_wall_list, std::vector<std::vector<int> >& newgrid, std::vector<std::vector<int> >& visitedgrid, int skip_x, int skip_y) {

    int x = removed_wall.x;
    int y = removed_wall.y;

    if (x > 0) {
        position x1;
        x1.x = x-1;
        x1.y = y;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0 && !isSkip(x1, skip_x, skip_y)) {
            nearby_wall_list.push_back(x1);
        }
    }

    if (x < height - 1) {
        position x1;
        x1.x = x+1;
        x1.y = y;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0 && !isSkip(x1, skip_x, skip_y)) {
            nearby_wall_list.push_back(x1);
        }
    }

    if (y > 0) {
        position x1;
        x1.x = x;
        x1.y = y-1;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0 && !isSkip(x1, skip_x, skip_y)) {
            nearby_wall_list.push_back(x1);
        }
    }

    if (y < width - 1) {
        position x1;
        x1.x = x;
        x1.y = y+1;
        if (newgrid[x1.x][x1.y] == 1 && visitedgrid[x1.x][x1.y] == 0 && !isSkip(x1, skip_x, skip_y)) {
            nearby_wall_list.push_back(x1);
        }
    }
}

bool canEvictWall(position wall_selection, int height, int width, std::vector<std::vector<int> >& newgrid) {

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

// Helper function
void printgrid(int height, int width, std::vector<std::vector<int> >& newgrid) {

    std::cout << "\n";

    for (int i=0; i<newgrid.size(); i++) {
        for (int j=0; j<width; j++) {
            std::cout << newgrid[i][j] << " ";
        }
        std::cout << "\n";
    }
}

// Helper function
void printgrid_with_proc(int height, int width, std::vector<std::vector<int> >& newgrid, int procID) {

    std::cout << "\n";

    for (int i=0; i<newgrid.size(); i++) {
        std::cout << "procID " << procID << ":  ";
        for (int j=0; j<width; j++) {
            std::cout << newgrid[i][j] << " ";
        }
        std::cout << "\n";
    }
}

// Helper function for validation
void validate_ghost_row(position *data, int length) {
    for (int i = 0; i<length; i++) {
        position curPos = data[i];
        if (curPos.x != -1) {
            cout << "Pos is 0: " << curPos.y << "\n";
        }
    }
}

int generateRandomNumberInRange(int end) {

    if (end == 0) {
        return 0;
    }

    int iSecret = rand() % end + 1;
    return iSecret;
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

    // cout << "Node " << currentProcID << " recieved data!!" << "\n";

    return true;
}

void send_status_signal(node_t *node, grid_status *data, int length, int nproc) {

    int currentProcID = node->procID;
    int nextProcID = (currentProcID + 1);

    if (currentProcID == nproc - 1) {
        return;
    }

    MPI_Isend(data, length, node->dtype, nextProcID, RING_SETUP_TAG,
             MPI_COMM_WORLD, &(node->send_request));
    // https://stackoverflow.com/questions/10017301/mpi-blocking-vs-non-blocking
    // https://peerj.com/articles/cs-95.pdf
}

void recv_status_signal(node_t *node, grid_status *data, int length) {

    int currentProcID = node->procID;

    // Last row won't receive any data
    if (currentProcID == 0) {
        return;
    }

    MPI_Recv(data, length, node->dtype, MPI_ANY_SOURCE, RING_SETUP_TAG,
                MPI_COMM_WORLD, MPI_STATUS_IGNORE);

    cout << "Node " << currentProcID << " recieved status data!!" << "\n";

    return;
}

void update_ghost_row_in_local_grid(position *ghost_data, int local_height, int width, vector<position>& nearby_wall_list, std::vector<std::vector<int> >& newgrid, std::vector<std::vector<int> >& visitedgrid) {

    for (int i = 0; i<width; i++) {
        position curPos = ghost_data[i];
        if (curPos.x != -1) {
            newgrid[local_height-1][curPos.y] = 0;
            visitedgrid[local_height-1][curPos.y] = 1;
        }
    }

    for (int i = 0; i<width; i++) {
        position curPos = ghost_data[i];
        if (curPos.x != -1) {
            curPos.x = local_height-1;
            // add_nearby_wall_to_list(local_height, width, curPos, nearby_wall_list, newgrid, visitedgrid);
        }
    }
}

int find_y_of_below_grid(int local_height, int width, std::vector<std::vector<int> >& newgrid) {

    int pos_y = 0;
    for (int i=0; i<width; i++) {
        if (newgrid[local_height-1][i] == 0) {
            pos_y = i;
            break;
        }
    }
    return pos_y;
}

int connect_grid_to_ghost_row(int cur_x, int cur_y, int local_height, int width, int first_y, std::vector<std::vector<int> >& newgrid, std::vector<std::vector<int> >& visitedgrid, vector<position>& nearby_wall_list, int procID) {

    position selected_wall;
    selected_wall.x = cur_x+1;
    selected_wall.y = cur_y;

    newgrid[selected_wall.x][selected_wall.y] = 0;
    if (cur_y < first_y) {
        add_nearby_wall_to_list_with_skip(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid, selected_wall.x, selected_wall.y+1);
    } else if (cur_y > first_y) {
        add_nearby_wall_to_list_with_skip(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid, selected_wall.x, selected_wall.y-1);
    } else {
        add_nearby_wall_to_list(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid);
    }
    visitedgrid[selected_wall.x][selected_wall.y] = 1;

    // Move left
    if (cur_y < first_y) {
        for (int i=cur_y+1; i<=first_y; i++) {
            selected_wall.y=i;
            newgrid[selected_wall.x][selected_wall.y] = 0;
            add_nearby_wall_to_list_with_skip(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid, selected_wall.x, selected_wall.y+1);
            visitedgrid[selected_wall.x][selected_wall.y] = 1;
        }
    } else if (cur_y > first_y) {
        for (int i=cur_y-1; i>=first_y; i--) {
            selected_wall.y=i;
            newgrid[selected_wall.x][selected_wall.y] = 0;
            add_nearby_wall_to_list_with_skip(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid, selected_wall.x, selected_wall.y+1);
            visitedgrid[selected_wall.x][selected_wall.y] = 1;
        }
    } else {
        // Already connected in the previous step
    }
} 

void delete_input_file(const char* input_file_name) {
    if( remove(input_file_name) != 0)
      printf("Error deleting file!!1\n");  
    else
        printf("File deleted\n");
}

void write_to_file(char *output_file_name, int procID, int total_height, int width, int local_height, std::vector<std::vector<int> >& newgrid) {
    FILE *fp;
    
    fp = fopen(output_file_name, "a");

    if (procID == 0) {
        delete_input_file(output_file_name);
        fp = fopen(output_file_name, "a");
        fprintf(fp, "%d %d\n", total_height, width);
    }

    for (int i=0; i<local_height-1; i++) {
        for (int j=0; j<width; j++) {
            fprintf(fp, "%d ", newgrid[i][j]);
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
}

void generateGridParallel(int procID, int nproc, int height, int width, char *output_file_name) {

    srand (time(NULL) + procID);

    if (height % nproc != 0) {
        cout << "height not divisible by nproc" << "\n";
        return;
    }

    node_t *currentNode = initializeNode(procID, nproc);

    // Start of the grid
    int start_height = (height / nproc) * procID;
    int end_height = ((height / nproc) * (procID + 1)) + 1;   // add +1 for ghost row
    int local_height = end_height-start_height;

    cout << "proc: " << procID << ", start_height" << start_height << ", end_height" << end_height << "\n";

    std::vector<std::vector<int> > newgrid(end_height-start_height, std::vector<int>(width));
    std::vector<std::vector<int> > visitedgrid(end_height-start_height, std::vector<int>(width));
    vector<position> nearby_wall_list;

    vector<position> ghost_wall_list;

    // Initialize the local grids
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
        visitedgrid[0][i] = 1;

        if (val == 0) {
            position init_position;
            init_position.x = 0;
            init_position.y = i;

            newgrid[init_position.x][init_position.y] = 0;
            
            send_packets[i] = init_position;
        } else {
            // Position is -1
            position init_position;
            init_position.x = -1;
            init_position.y = -1;

            send_packets[i] = init_position;
        }
    }

    for (int i=0; i<width; i++) {
        if (send_packets[i].x != -1) {
            add_nearby_wall_to_list(local_height, width, send_packets[i], nearby_wall_list, newgrid, visitedgrid);
            
            break;
        }
    }

    // Communicate the ghost rows
    send_ghost_row(currentNode, send_packets, width);

    recieve_ghost_row(currentNode, recv_packets, width, nproc);

    update_ghost_row_in_local_grid(recv_packets, local_height, width, nearby_wall_list, newgrid, visitedgrid);

    int num_of_mistakes = 0;
    bool is_grid_connected = false;

    // Iterate while there exist walls which are potential targets.
    while (nearby_wall_list.size() > 0) {

        int random_wall_idx = generateRandomNumberInRange(nearby_wall_list.size()-1);

        position selected_wall = nearby_wall_list.at(random_wall_idx);

        if (visitedgrid[selected_wall.x][selected_wall.y] == 1) {
            nearby_wall_list.erase(nearby_wall_list.begin() + random_wall_idx);
            num_of_mistakes++;
            continue;
        }

        // If it's the current or next ghost wall, skip
        if (selected_wall.x == local_height - 1 || selected_wall.x == 0) {
            nearby_wall_list.erase(nearby_wall_list.begin() + random_wall_idx);
            continue;
        }

        bool canEvict = canEvictWall(selected_wall, local_height, width, newgrid);

        if (canEvict) {
            newgrid[selected_wall.x][selected_wall.y] = 0;
            // Connect the current grid block to the next grid block 
            if (selected_wall.x == local_height - 3 && 
                is_grid_connected == false) {    
                    int first_y = find_y_of_below_grid(local_height, width, newgrid);
                    
                    // Connect the current node but don't connect the second last row
                    add_nearby_wall_to_list_with_skip(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid, selected_wall.x + 1, selected_wall.y);
                    visitedgrid[selected_wall.x][selected_wall.y] = 1;
                    nearby_wall_list.erase(nearby_wall_list.begin() + random_wall_idx);

                    connect_grid_to_ghost_row(selected_wall.x, selected_wall.y, local_height, width, first_y, newgrid, visitedgrid, nearby_wall_list, procID);
                    is_grid_connected = true;
                    continue;
            }
            add_nearby_wall_to_list(local_height, width, selected_wall, nearby_wall_list, newgrid, visitedgrid);
        }

        visitedgrid[selected_wall.x][selected_wall.y] = 1;
        nearby_wall_list.erase(nearby_wall_list.begin() + random_wall_idx);
    }

    // Create the status packet
    grid_status *status_packets =
        (grid_status *)calloc(2, sizeof(grid_status));
        status_packets[0].status = 1;

    node_t *statusNode = initializeNodeWithCompletion(procID, nproc);

    // Send the status signal to start writing to file.
    if (procID == 0) {
        write_to_file(output_file_name, procID, height, width, local_height, newgrid);
        send_status_signal(statusNode, status_packets, 1, nproc);
        return;
    }

    recv_status_signal(statusNode, status_packets, 1);
    write_to_file(output_file_name, procID, height, width, local_height, newgrid);
    send_status_signal(statusNode, status_packets, 1, nproc);
}