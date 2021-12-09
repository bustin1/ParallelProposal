// No need to modify this file.

#include "generategrid.h"
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include "mpi.h"

int main(int argc, char *argv[]) {
  int procID;
  int nproc;
  double startTime;
  double endTime;
  char *outputFilename = NULL;
  int opt = 0;
  int height, width;

  // Initialize MPI
  MPI_Init(&argc, &argv);

  // Read command line arguments
  do {
    opt = getopt(argc, argv, "f:h:w:");
    switch (opt) {
    case 'f':
      outputFilename = optarg;
      break;

    case 'h':
      height = atoi(optarg);
      break;

    case 'w':
      width = atoi(optarg);
      break;

    case -1:
      break;

    default:
      break;
    }
  } while (opt != -1);

  if (outputFilename == NULL) {
    printf("Usage: %s -f <output_filename> [-h height] [-w width]\n", argv[0]);
    MPI_Finalize();
    return -1;
  }

  // Get process rank
  MPI_Comm_rank(MPI_COMM_WORLD, &procID);

  // Get total number of processes specificed at start of run
  MPI_Comm_size(MPI_COMM_WORLD, &nproc);

  // Run computation
  // startTime = MPI_Wtime();
  // compute(procID, nproc, outputFilename, prob, numIterations);
  generateGridParallel(procID, nproc, height, width);
  printf("proc %d ended.\n", procID);
  // endTime = MPI_Wtime();

  // Cleanup
  MPI_Finalize();
  // printf("Elapsed time for proc %d: %f\n", procID, endTime - startTime);
  return 0;
}
