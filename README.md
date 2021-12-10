# ParallelProposal

## Description

We plan to parallelize random maze generation and particle filtering. Maze generation has a nice dual interpretation with graph search algorithms. Through openMP and MPI interface, we want to efficently implement maze generation using sequential and parallel graph algorithms. After creating a maze, particle filtering, a method to determine the location of a robot, will be used to navigate through this maze. The aim of this project is to identify axis of parallelism learned through tools developed from 15418/15618.

## Approach

See report

## Target Machines

GHC machines with 8 intel core -i7 processors and an Nvidia RTX 2080.  

## Results

For maze generation we achieved higher-than-linear speedup  
For particle filtering we achieved 40x in CUDA vs. the single-threaded CPU.

## Commands to run
Maze Generation
```
cd grid_gen
mpirun -np <proc_count> ./generategrid -f <output_file> -h <height> -w <width>
```
NOTE: This will only work if the height and width are divisible by the number of processors.

Particle Filtering (CUDA)
```
./render -i <input> -n <# of particles> -r <# of rays> -g <size of one wall widith=height> -d <debug mode>
./render -i tests/hard.txt -n 1000 -r 16 -g 20 -d 2
```
For more help simply run
```
./render
```
If you want to look at the openMP and sequential versions, take a look at the branches in this repo https://github.com/bustin1/ParallelProposal
