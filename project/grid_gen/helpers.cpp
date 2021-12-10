#include "mpi.h"
#include <assert.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <libgen.h>
#include <time.h>

#include "helpers.h"



bool should_send_data(const int SEND_FREQ, int num_of_wires, int wire_idx,
                      int wire_start) {
    return (wire_idx - wire_start) % SEND_FREQ == SEND_FREQ - 1;
}

bool should_recv_data(const int RECV_FREQ, int num_of_wires, int j,
                      int wire_start) {
    return ((j - wire_start) % RECV_FREQ == RECV_FREQ - 1 ||
            j == num_of_wires - 1);
}

MPI_Datatype create_mpi_wire_dtype() {
    const int NUM_ELEMENTS = 6;
    int lengths[NUM_ELEMENTS] = {1, 1, 1, 1, 1, 1}; // number of ints/bools
    MPI_Aint displacements[NUM_ELEMENTS]; // offset from start of struct
    MPI_Aint base_address;
    MPI_Datatype types[NUM_ELEMENTS] = {MPI_INT, MPI_INT, MPI_INT,
                                        MPI_INT, MPI_INT, MPI_CXX_BOOL};
    wire_t dummy;

    MPI_Get_address(&dummy, &base_address);
    MPI_Get_address(&dummy.bend_cor, &displacements[0]);
    MPI_Get_address(&dummy.is_hor_first, &displacements[1]);
    displacements[0] = MPI_Aint_diff(displacements[0], base_address);
    displacements[1] = MPI_Aint_diff(displacements[1], base_address);
    displacements[2] = MPI_Aint_diff(displacements[2], base_address);
    displacements[3] = MPI_Aint_diff(displacements[3], base_address);
    displacements[4] = MPI_Aint_diff(displacements[4], base_address);
    MPI_Datatype mpi_wire_t;

    MPI_Type_create_struct(NUM_ELEMENTS, lengths, displacements, types,
                           &mpi_wire_t);
    MPI_Type_commit(&mpi_wire_t);
    return mpi_wire_t;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// PLOTTING WIRE //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void put_wire_on_cost_graph_vertical(int wire_idx, cost_t *costs, wire_t *wires, int dim_x, int dim_y) {

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;
    bool is_hor_first = wire->is_hor_first;
    int bend_cor = wire->bend_cor;

    if (is_hor_first) {
        // Do vertical calculations

        // Move from top to bottom
        if (y2 > y1) {
            for (int i = y1; i < bend_cor; i++) {
                costs[i * dim_x + x1] += 1;
            }
            for (int i = bend_cor; i < y2; i++) {
                costs[i * dim_x + x2] += 1;
            }
        } else { // Move from bottom to top
            for (int i = y1; i > bend_cor; i--) {
                costs[i * dim_x + x1] += 1;
            }
            for (int i = bend_cor; i > y2; i--) {
                costs[i * dim_x + x2] += 1;
            }
        }

    } else {
        // Move from top to bottom
        if (y2 > y1) {
            for (int i = y1; i < y2; i++) {
                costs[i * dim_x + bend_cor] += 1;
            }
        } else { // Move from bottom to top
            for (int i = y1; i > y2; i--) {
                costs[i * dim_x + bend_cor] += 1;
            }
        }
    }
}

void put_wire_on_cost_graph_horizontal(int wire_idx, cost_t *costs,
                                       wire_t *wires, int dim_x, int dim_y) {

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;
    int is_hor_first = wire->is_hor_first;
    int bend_cor = wire->bend_cor;

    if (is_hor_first) {
        // Do horizontal calculations

        if (x2 > x1) { // Move from left to right
            for (int i = x1; i < x2; i++) {
                costs[bend_cor * dim_x + i] += 1;
            }
        } else { // Move from right to left
            for (int i = x1; i > x2; i--) {
                costs[bend_cor * dim_x + i] += 1;
            }
        }

    } else {
        if (x2 > x1) { // Move from left to right
            for (int i = x1; i < bend_cor; i++) {
                costs[y1 * dim_x + i] += 1;
            }
            for (int i = bend_cor; i < x2; i++) {
                costs[y2 * dim_x + i] += 1;
            }
        } else { // Move from right to left
            for (int i = x1; i > bend_cor; i--) {
                costs[y1 * dim_x + i] += 1;
            }
            for (int i = bend_cor; i > x2; i--) {
                costs[y2 * dim_x + i] += 1;
            }
        }
    }
}

void plot_on_cost_graph(int wire_idx, cost_t *costs, wire_t *wires, int dim_x,
                        int dim_y) {

    wire_t *wire = &(wires[wire_idx]);
    int x2 = wire->x2;
    int y2 = wire->y2;

    put_wire_on_cost_graph_vertical(wire_idx, costs, wires, dim_x, dim_y);

    put_wire_on_cost_graph_horizontal(wire_idx, costs, wires, dim_x, dim_y);

    // Added end points separately to prevent double-counting or zero-counting
    // of endpoints
    costs[y2 * dim_x + x2] += 1;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// REMOVING WIRE //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void remove_wire_from_cost_graph_vertical(int wire_idx, cost_t *costs,
                                          wire_t *wires, int dim_x, int dim_y) {

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;
    int bend_cor = wire->bend_cor;
    bool is_hor_first = wire->is_hor_first;

    if (is_hor_first) {
        // Do vertical calculations

        // Move from top to bottom
        if (y2 > y1) {
            for (int i = y1; i < bend_cor; i++) {
                costs[i * dim_x + x1] -= 1;
            }
            for (int i = bend_cor; i < y2; i++) {
                costs[i * dim_x + x2] -= 1;
            }
        } else { // Move from bottom to top
            for (int i = y1; i > bend_cor; i--) {
                costs[i * dim_x + x1] -= 1;
            }
            for (int i = bend_cor; i > y2; i--) {
                costs[i * dim_x + x2] -= 1;
            }
        }

    } else {
        // Move from top to bottom
        if (y2 > y1) {
            for (int i = y1; i < y2; i++) {
                costs[i * dim_x + bend_cor] -= 1;
            }
        } else { // Move from bottom to top
            for (int i = y1; i > y2; i--) {
                costs[i * dim_x + bend_cor] -= 1;
            }
        }
    }
}

void remove_wire_from_cost_graph_horizontal(int wire_idx, cost_t *costs,
                                            wire_t *wires, int dim_x,
                                            int dim_y) {

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;
    int bend_cor = wire->bend_cor;
    bool is_hor_first = wire->is_hor_first;

    if (is_hor_first) {
        // Do horizontal calculations

        if (x2 > x1) { // Move from left to right
            for (int i = x1; i < x2; i++) {
                costs[bend_cor * dim_x + i] -= 1;
            }
        } else { // Move from right to left
            for (int i = x1; i > x2; i--) {
                costs[bend_cor * dim_x + i] -= 1;
            }
        }

    } else {
        if (x2 > x1) { // Move from left to right
            for (int i = x1; i < bend_cor; i++) {
                costs[y1 * dim_x + i] -= 1;
            }

            for (int i = bend_cor; i < x2; i++) {
                costs[y2 * dim_x + i] -= 1;
            }
        } else { // Move from right to left
            for (int i = x1; i > bend_cor; i--) {
                costs[y1 * dim_x + i] -= 1;
            }

            for (int i = bend_cor; i > x2; i--) {
                costs[y2 * dim_x + i] -= 1;
            }
        }
    }
}

void remove_wire_from_cost_graph(int wire_idx, cost_t *costs, wire_t *wires,
                                 int dim_x, int dim_y) {

    wire_t *wire = &(wires[wire_idx]);
    int x2 = wire->x2;
    int y2 = wire->y2;

    remove_wire_from_cost_graph_vertical(wire_idx, costs, wires, dim_x, dim_y);
    remove_wire_from_cost_graph_horizontal(wire_idx, costs, wires, dim_x,
                                           dim_y);

    // Added end points separately to prevent double-counting or zero-counting
    // of endpoints
    costs[y2 * dim_x + x2] -= 1;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// CALC WIRE COST /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int path_cost_vertical(int wire_idx, cost_t *costs, wire_t *wires, int dim_x,
                       int dim_y, int bend_cor, bool is_hor_first,
                       int &max_value) {

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;

    int total_ver_cost = 0;
    int bottleneck = 0;

    if (is_hor_first) {
        // Do vertical calculations

        // Move from top to bottom
        if (y2 > y1) {

            for (int i = y1; i < bend_cor; i++) {
                total_ver_cost += costs[i * dim_x + x1];
                bottleneck = MAX(bottleneck, costs[i * dim_x + x1]);
            }

            for (int i = bend_cor; i < y2; i++) {
                total_ver_cost += costs[i * dim_x + x2];
                bottleneck = MAX(bottleneck, costs[i * dim_x + x2]);
            }
        } else { // Move from bottom to top

            for (int i = y1; i > bend_cor; i--) {
                total_ver_cost += costs[i * dim_x + x1];
                bottleneck = MAX(bottleneck, costs[i * dim_x + x1]);
            }

            for (int i = bend_cor; i > y2; i--) {
                total_ver_cost += costs[i * dim_x + x2];
                bottleneck = MAX(bottleneck, costs[i * dim_x + x2]);
            }
        }

    } else {
        // Move from top to bottom
        if (y2 > y1) {

            for (int i = y1; i < y2; i++) {
                total_ver_cost += costs[i * dim_x + bend_cor];
                bottleneck = MAX(bottleneck, costs[i * dim_x + bend_cor]);
            }
        } else { // Move from bottom to top

            for (int i = y1; i > y2; i--) {
                total_ver_cost += costs[i * dim_x + bend_cor];
                bottleneck = MAX(bottleneck, costs[i * dim_x + bend_cor]);
            }
        }
    }

    max_value = bottleneck;

    return total_ver_cost;
}

int path_cost_horizontal(int wire_idx, cost_t *costs, wire_t *wires, int dim_x,
                         int dim_y, int bend_cor, bool is_hor_first,
                         int &max_value) {

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;

    int total_hor_cost = 0;
    int bottleneck = 0;

    if (is_hor_first) {
        // Do horizontal calculations

        if (x2 > x1) { // Move from left to right

            for (int i = x1; i < x2; i++) {
                total_hor_cost += costs[bend_cor * dim_x + i];
                bottleneck = MAX(bottleneck, costs[bend_cor * dim_x + i]);
            }
        } else { // Move from right to left

            for (int i = x1; i > x2; i--) {
                total_hor_cost += costs[bend_cor * dim_x + i];
                bottleneck = MAX(bottleneck, costs[bend_cor * dim_x + i]);
            }
        }

    } else {
        if (x2 > x1) { // Move from left to right

            for (int i = x1; i < bend_cor; i++) {
                total_hor_cost += costs[y1 * dim_x + i];
                bottleneck = MAX(bottleneck, costs[y1 * dim_x + i]);
            }

            for (int i = bend_cor; i < x2; i++) {
                total_hor_cost += costs[y2 * dim_x + i];
                bottleneck = MAX(bottleneck, costs[y2 * dim_x + i]);
            }
        } else { // Move from right to left

            for (int i = x1; i > bend_cor; i--) {
                total_hor_cost += costs[y1 * dim_x + i];
                bottleneck = MAX(bottleneck, costs[y1 * dim_x + i]);
            }

            for (int i = bend_cor; i > x2; i--) {
                total_hor_cost += costs[y2 * dim_x + i];
                bottleneck = MAX(bottleneck, costs[y2 * dim_x + i]);
            }
        }
    }

    max_value = bottleneck;

    return total_hor_cost;
}

int path_cost(int wire_idx, cost_t *costs, wire_t *wires, int dim_x, int dim_y,
              int bend_cor, bool is_hor_first, int &bottleneck) {

    wire_t *wire = &(wires[wire_idx]);
    int x2 = wire->x2;
    int y2 = wire->y2;

    int bottleneck_vertical = 0;
    int bottleneck_horizontal = 0;
    int total_cost =
        path_cost_vertical(wire_idx, costs, wires, dim_x, dim_y, bend_cor,
                           is_hor_first, bottleneck_vertical) +
        path_cost_horizontal(wire_idx, costs, wires, dim_x, dim_y, bend_cor,
                             is_hor_first, bottleneck_horizontal);
    bottleneck = MAX(bottleneck_vertical, bottleneck_horizontal);

    // Added separately to prevent double counting or zero counting of last
    // endpoint
    total_cost += costs[y2 * dim_x + x2];

    return total_cost;
}

void update_cost_graph(cost_t *costs, wire_t *wires, wire_packet_t *packets,
                       int wireBS, int length, int procID, int nproc, int dim_x, int dim_y) {

    int wire_start = packets[0].senderID * wireBS + packets[0].offset;
    int wire_end = wire_start + length;
    // printf("(process: %d): %d %d\n", procID, wire_start, wire_end);

    for (int j = wire_start; j < wire_end; j++) {
        remove_wire_from_cost_graph(j, costs, wires, dim_x, dim_y);
        // update
        wire_packet_t packet = packets[j - wire_start];
        wire_t* wire = &(wires[j]);
        wire->is_hor_first = packet.is_hor_first;
        wire->bend_cor = packet.bend_cor;

        int x1 = wire->x1;
        int y1 = wire->y1;
        int x2 = wire->x2;
        int y2 = wire->y2;

        // printf("Update cost graph Wire: PROCID: %d %d, %d // %d, %d // %d %d\n", procID, x1, y1, x2, y2, wire->bend_cor, wire->is_hor_first);

        plot_on_cost_graph(j, costs, wires, dim_x, dim_y);
        // wire_t *wire = wires + i;
//        printf("procId: %d, %d %d, %d %d\n", procID, wire->x1, wire->y1, wire->x2, wire->y2);
        // printf("%d\n", i);
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////// NEW ROUTES!!!! /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

bool select_new_route_with_prob(double prob) {
    double random_variable = (((double)std::rand()) / RAND_MAX);

    if (random_variable <= prob) {
        return true;
    }

    return false;
}

void get_new_random_route_path(int wire_idx, cost_t *costs, wire_t *wires,
                               int dim_x, int dim_y, wire_packet_t *packets,
                               int packet_idx) {

    wire_t *wire = &(wires[wire_idx]);
    wire_packet_t *packet = &(packets[packet_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;

    // Delete the old path
    remove_wire_from_cost_graph(wire_idx, costs, wires, dim_x, dim_y);


    // uniformly chooses which bend (veritcal or horizontal)
    double dx = std::abs(wire->x1 - wire->x2);
    double dy = std::abs(wire->y1 - wire->y2);
    double h_prob = dy / (dx + dy);           // horizontal prob first
    if (select_new_route_with_prob(h_prob)) { // horizontal
        double random_variable = ((double)std::rand()) / RAND_MAX;
        wire->bend_cor = (int)(random_variable * dx + MIN(wire->x1, wire->x2));
        packet->bend_cor = wire->bend_cor;
        if (wire->bend_cor == MAX(wire->x1, wire->x2)) {
            wire->bend_cor -= 1;
            packet->bend_cor -= 1;
        }
        wire->is_hor_first = false;
        packet->is_hor_first = false;
        //        printf("random!\n");
    } else { // vertical
        double random_variable = ((double)std::rand()) / RAND_MAX;
        wire->bend_cor = (int)(random_variable * dy + MIN(wire->y1, wire->y2));
        packet->bend_cor = wire->bend_cor;
        if (wire->bend_cor == MAX(wire->y1, wire->y2)) {
            wire->bend_cor -= 1;
            packet->bend_cor -= 1;
        }
        wire->is_hor_first = true;
        packet->is_hor_first = true;
        //        printf("min route!\n");
    }

    // printf("(x1, y1): (%d, %d)  and (x2, y2): (%d, %d) , bend_cor: %d,
    // is_hor_first: %d\n", wire->x1, wire->y1, wire->x2, wire->y2,
    // wire->bend_cor, wire->is_hor_first);

    // printf("Random Wire: %d, %d // %d, %d // %d %d\n", x1, y1, x2, y2, wire->bend_cor, wire->is_hor_first);

    // Write to the cost graph
    plot_on_cost_graph(wire_idx, costs, wires, dim_x, dim_y);
}

int get_new_min_route(int wire_idx, cost_t *costs, wire_t *wires, int dim_x,
                      int dim_y, int path_dir, wire_packet_t *packets, int packet_idx) {

    wire_packet_t *packet = &(packets[packet_idx]);

    wire_t *wire = &(wires[wire_idx]);
    int x1 = wire->x1;
    int y1 = wire->y1;
    int x2 = wire->x2;
    int y2 = wire->y2;

    // Delete the old path
    remove_wire_from_cost_graph(wire_idx, costs, wires, dim_x, dim_y);

    int smaller_x;
    int smaller_y;
    int min_path_cost = __INT_MAX__;
    int min_bend_cor = 0;
    bool min_is_hor_first = false;

    if (x1 < x2) {
        smaller_x = x1;
    } else {
        smaller_x = x2;
    }

    if (y1 < y2) {
        smaller_y = y1;
    } else {
        smaller_y = y2;
    }

    int current_bottleneck = __INT_MAX__;
    int total_path_cost = std::abs(x2 - x1) + std::abs(y2 - y1);

    int current_path_cost = 0;

    int bottleneck = 0;
    // If value greater than x2, then shift to vertical
    if (path_dir < std::abs(x1 - x2)) {
        current_path_cost = path_cost(wire_idx, costs, wires, dim_x, dim_y,
                                      smaller_x + path_dir, false, bottleneck) 
                                      + total_path_cost;
    } else {
        current_path_cost = path_cost(wire_idx, costs, wires, dim_x, dim_y,
                                      smaller_y + total_path_cost - path_dir,
                                      true, bottleneck) + total_path_cost;
    }

    if (bottleneck <= current_bottleneck && current_path_cost < min_path_cost) {
        if (path_dir < std::abs(x1 - x2)) {
            min_bend_cor = smaller_x + path_dir;
            min_is_hor_first = false;
        } else {
            min_bend_cor = smaller_y + total_path_cost - path_dir;
            min_is_hor_first = true;
        }
        min_path_cost = current_path_cost;
        current_bottleneck = bottleneck;
    }

    // After finding the min path, update the wire and cost graph
    wire->bend_cor = min_bend_cor;
    wire->is_hor_first = min_is_hor_first;
    packet->bend_cor = min_bend_cor;
    packet->is_hor_first = min_is_hor_first;

    // printf("New Path Wire: %d, %d // %d, %d // %d %d\n", x1, y1, x2, y2, wire->bend_cor, wire->is_hor_first);

    plot_on_cost_graph(wire_idx, costs, wires, dim_x, dim_y);

    // printf("Wire NEW MIN: %d, %d // %d, %d\n", x1, y1, x2, y2);

    return min_path_cost;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void print_all_packets(wire_packet_t *packets, int size) {
    for (int i = 0; i < size; i++) {
        wire_packet_t packet = packets[i];
        printf("bend_cor:%d is_hor_first:%d\n", packet.bend_cor,
               (int)packet.is_hor_first);
    }
}

void print_cost_matrix(cost_t *costs, int dim_x, int dim_y) {
    for (int row = 0; row < dim_y; row++) {
        for (int col = 0; col < dim_x; col++) {
            printf("%d ", costs[dim_y * row + col]);
        }
        printf("\n");
    }
}

void delete_input_file(const char* input_file_name) {
    if( remove(input_file_name) != 0)
        // perror( "Error deleting file" );
      printf("Error deleting file!!1\n");  
    else
        printf("File deleted\n");
        // puts( "File successfully deleted" );
}

void write_cost_matrix(cost_t *costs, int dim_x, int dim_y,
                       const char *input_filename, int num_of_threads) {
    // parse file name (do we even need this?)
    char wire_output_filename[100] = "costs_";
    strcat(wire_output_filename, input_filename);
    strcat(wire_output_filename, "_");

    char num_threads[5];
    sprintf(num_threads, "%d", num_threads);
    strcat(wire_output_filename, num_threads);
    strcat(wire_output_filename, ".txt");

    //    printf("out name: %s\n", num_threads);
    //    printf("out name: %s\n", wire_output_filename);

    FILE *fp;
    // TODO: wire_output_filename is wrong
    // fp = fopen(wire_output_filename, "w");
    const char *output_file_name = "costs.txt";
    delete_input_file(output_file_name);
    fp = fopen(output_file_name, "w");

    fprintf(fp, "%d %d\n", dim_x, dim_y);

    for (int row = 0; row < dim_y; row++) {
        for (int col = 0; col < dim_x; col++) {
            fprintf(fp, "%d ", costs[dim_y * row + col]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
}

void write_wiring(wire_t *wires, int dim_x, int dim_y, int num_of_wires,
                  const char *input_filename, int num_of_threads) {
    // conver filename and num threads to strings
    char wire_output_filename[100] = "output_";
    strcat(wire_output_filename, input_filename);
    strcat(wire_output_filename, "_");

    char num_threads[10];
    sprintf(num_threads, "%d", num_threads);
    strcat(wire_output_filename, num_threads);
    strcat(wire_output_filename, ".txt");

    FILE *fp;
    // TODO: wire_output_filename is wrong
    // fp = fopen(wire_output_filename, "w");
    const char *output_file_name = "output.txt";
    delete_input_file(output_file_name);
    fp = fopen(output_file_name, "w");

    fprintf(fp, "%d %d\n", dim_x, dim_y);
    fprintf(fp, "%d\n", num_of_wires);

    for (int j = 0; j < num_of_wires; j++) {
        wire_t wire = wires[j];
        int x1 = wire.x1;
        int y1 = wire.y1;
        int x2 = wire.x2;
        int y2 = wire.y2;
        int bend_cor = wire.bend_cor;
        bool is_hor_first = wire.is_hor_first;

        // printf("Wire: %d, %d // %d, %d // %d %d\n", x1, y1, x2, y2, bend_cor, is_hor_first);

        if (is_hor_first == false) {
            /**
             * Move diagonally along x-axis from x1 to bend_cor
             * Move vertically along y-axis from y1 to y2
             * Move diagonally along x-axis from bend_cor to x2
             * */

            if (x2 > x1) {
                for (int i = x1; i < bend_cor; i++) {
                    fprintf(fp, "%d %d ", i, y1);
                }

                // Move vertically
                if (y2 > y1) {
                    for (int i = y1; i < y2; i++) {
                        fprintf(fp, "%d %d ", bend_cor, i);
                    }
                } else {
                    for (int i = y1; i > y2; i--) {
                        fprintf(fp, "%d %d ", bend_cor, i);
                    }
                }

                for (int i = bend_cor; i < x2; i++) {
                    fprintf(fp, "%d %d ", i, y2);
                }
            } else {
                for (int i = x1; i > bend_cor; i--) {
                    fprintf(fp, "%d %d ", i, y1);
                }

                // Move vertically
                if (y2 > y1) {
                    for (int i = y1; i < y2; i++) {
                        fprintf(fp, "%d %d ", bend_cor, i);
                    }

                } else {
                    for (int i = y1; i > y2; i--) {
                        fprintf(fp, "%d %d ", bend_cor, i);
                    }
                }

                for (int i = bend_cor; i > x2; i--) {
                    fprintf(fp, "%d %d ", i, y2);
                }
            }
        } else {
            /**
             * Move vertically along y-axis from y1 to bend_cor
             * Move diagonally along x-axis from x1 to x2
             * Move vertically along y-axis from bend_cor to y2
             * */

            if (y2 > y1) {

                // Move vertically first
                for (int i = y1; i < bend_cor; i++) {
                    fprintf(fp, "%d %d ", x1, i);
                }

                // Move diagonally
                if (x2 > x1) {
                    for (int i = x1; i < x2; i++) {
                        fprintf(fp, "%d %d ", i, bend_cor);
                    }
                } else {
                    for (int i = x1; i > x2; i--) {
                        fprintf(fp, "%d %d ", i, bend_cor);
                    }
                }

                // Move vertically again
                for (int i = bend_cor; i < y2; i++) {
                    fprintf(fp, "%d %d ", x2, i);
                }
            } else {

                // Move vertically first
                for (int i = y1; i > bend_cor; i--) {
                    fprintf(fp, "%d %d ", x1, i);
                }

                // Move diagonally
                if (x2 > x1) {
                    for (int i = x1; i < x2; i++) {
                        fprintf(fp, "%d %d ", i, bend_cor);
                    }
                } else {
                    for (int i = x1; i > x2; i--) {
                        fprintf(fp, "%d %d ", i, bend_cor);
                    }
                }

                // Move vertically again
                for (int i = bend_cor; i > y2; i--) {
                    fprintf(fp, "%d %d ", x2, i);
                }
            }
        }
        fprintf(fp, "%d %d\n", x2, y2);
    }
    fclose(fp);
}

int get_local_sum_of_squares(cost_t* costs, int nproc, int procID, int dim_x, int dim_y) {
    int block_width = dim_x;
    int block_height = (dim_y + nproc - 1) / nproc;
    int block_start = block_height * procID; 
    int block_end = MIN(block_start + block_height, dim_y);
    int local_sum = 0;
    for (int r=0; r<block_width; r++) {
        for (int c=0; c<block_height; c++) {
            int tmp = costs[r * block_width + c];
            local_sum += tmp * tmp;
        }
    }
    return local_sum;
}

int get_sum_of_squares(cost_t* costs, const int root, int procID, int nproc, int dim_x, int dim_y) {

    int local_sum = get_local_sum_of_squares(costs, nproc, procID, dim_x, dim_y);
    int global_sum = 0;
    MPI_Reduce(&local_sum, &global_sum, 1, MPI_INT, MPI_SUM, root, MPI_COMM_WORLD);
    return global_sum;

}

int get_local_max(cost_t* costs, int nproc, int procID, int dim_x, int dim_y) {
    int block_width = dim_x;
    int block_height = (dim_y + nproc - 1) / nproc;
    int block_start = block_height * procID; 
    int block_end = MIN(block_start + block_height, dim_y);
    int local_max = 0;
    for (int r=0; r<block_width; r++) {
        for (int c=0; c<block_height; c++) {
            int tmp = costs[r * block_width + c];
            local_max = MAX(tmp, local_max);
        }
    }
    return local_max;
}

int get_max(cost_t* costs, const int root, int procID, int nproc, int dim_x, int dim_y) {

    int local_max = get_local_max(costs, nproc, procID, dim_x, dim_y);
    int global_max = 0;
    MPI_Reduce(&local_max, &global_max, 1, MPI_INT, MPI_MAX, root, MPI_COMM_WORLD);
    return global_max;

}



// #define PRINTF(fmt, args...) printf(fmt, ## args)
// #else
// #define PRINTF(fmt, args...)

