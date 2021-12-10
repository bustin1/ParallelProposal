#ifndef HELPER
#define HELPER

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) > (b)) ? (b) : (a))

bool should_send_data(const int SEND_FREQ, int num_of_wires, int wire_idx,
                      int wire_start);

bool should_recv_data(const int RECV_FREQ, int num_of_wires, int j,
                      int wire_start);

MPI_Datatype create_mpi_wire_dtype();

#include "ring.h"
#include "wireroute.h"

void plot_on_cost_graph(int wire_idx, cost_t *costs, wire_t *wires, int dim_x,
                        int dim_y);

bool select_new_route_with_prob(double prob);

int get_new_min_route(int wire_idx, cost_t *costs, wire_t *wires, int dim_x,
                      int dim_y, int path_dir, wire_packet_t *send_packets, int packet_idx);

void get_new_random_route_path(int wire_idx, cost_t *costs, wire_t *wires,
                               int dim_x, int dim_y, wire_packet_t *packets,
                               int packet_idx);

void update_cost_graph(cost_t *costs, wire_t *wires, wire_packet_t *packets,
                       int wireBS, int length, int procID, int nproc, int dim_x, int dim_y);

void write_wiring(wire_t *wires, int dim_x, int dim_y, int num_of_wires,
                  const char *input_filename, int num_of_threads);

void write_cost_matrix(cost_t *costs, int dim_x, int dim_y,
                       const char *input_filename, int num_of_threads);

int get_sum_of_squares(cost_t* costs, const int root, int procID, int nproc, 
                            int dim_x, int dim_y);

int get_max(cost_t* costs, const int root, int procID, int nproc, int dim_x, int dim_y);

#endif
