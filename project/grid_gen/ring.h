#ifndef _RING_PATH_H
#define _RING_PATH_H


typedef struct ringNode {
    int procID;
    int nproc;
    bool firstRecv;
    MPI_Request send_request;
    MPI_Request recv_request;
    MPI_Datatype dtype;
    // struct ringNode *next; // Removed because sending pointers inside struct is complicated. Can reintroduce if need multiple rings. 
} node_t;

// node_t *rootNode;
//
// typedef struct {
//     int senderID;
//     int offset;
//     int bend_cor;
//     bool is_hor_first;
// } wire_packet_t;

node_t* initializeNode(int procID, int nproc);

//void connectToNeighbours(node_t *node, int nproc, wire_packet_t, int length);
void sendToNextNode(node_t *node, wire_packet_t* data, int length);
void receiveFromNextNode(node_t *node, wire_packet_t* data, int length);
bool send_and_recv_in_ring(node_t* node, wire_packet_t* data, int length);

#endif
