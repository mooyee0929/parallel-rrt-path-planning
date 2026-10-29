# Parallel RRT Path Planning

A C++ implementation of the RRT (Rapidly-exploring Random Tree) and RRT* sampling-based path planners on 2D maps with rectangular obstacles, parallelized four different ways: OpenMP, Pthreads, MPI and CUDA. Each version targets the same hot spots of the planning loop (the nearest-node search over the whole tree and the RRT* choose-parent / rewire passes), keeps the same command-line interface and output format, and prints a per-phase timing breakdown so the back ends can be compared. A Python script renders the resulting tree and path as an image.

The OpenMP version is the original OMPRRT ("Open-MP based rapidly exploring random trees", pronounced "Ompert") code base; the Pthreads, MPI and CUDA versions extend it.

![RRT tree and path on map 3](images/map_3.png)

*Tree (blue), obstacles (grey) and the resulting path (pink) on `inputs/map_3.txt`, rendered by `visualize.py`.*

## How the planner works

Each iteration of the main loop:

1. Samples a random point on the map (the seed is fixed with `srand(0)`, so runs are reproducible).
2. Finds the nearest existing tree node by squared Euclidean distance.
3. Grows a new node from that nearest node toward the sample by a fixed step distance (`-d`).
4. Rejects the new edge if it hits an obstacle. Obstacles are padded by one unit and tested with segment/rectangle-edge intersection.
5. With RRT* enabled (`-r 1`), picks the lowest-cost parent among nearby nodes (radius `2 * d`) and rewires neighbors that become cheaper through the new node.
6. Records the node as a goal candidate if it lands within the step distance of the goal, keeping the lowest-cost one.

The program accumulates wall-clock time (in microseconds) for six phases: generate random point, find nearest node, grow from node, collision check, RRT*, and goal check. It prints those totals and the overall time.

## Parallel back ends

| Source | Model | What is parallelized |
| --- | --- | --- |
| `omp.cpp` | OpenMP | Nearest-node search with `#pragma omp parallel for`, keeping a per-thread minimum that is reduced afterwards. The RRT* choose-parent and rewire loops use `#pragma omp for schedule(static, 32)`. |
| `pthread.cpp` | Pthreads | The node array is split into equal chunks, one per thread. The global minimum is updated under a mutex. RRT* runs as two threaded phases (`find_min_cost`, then `change_cost`). While the tree has fewer nodes than threads, the thread count is reduced to match. |
| `mpi.cpp` | MPI | Each rank scans a strided slice of the node list. Local minima are collected on rank 0 with `MPI_Gather`, and the winning index is sent back to every rank with `MPI_Bcast`. Every rank uses the same seed, so all ranks grow identical trees. |
| `cuda.cpp` + `cuda.cu` | CUDA | `findNearestNodeKernel` runs nearest-node search on the GPU, with one thread per node, 1024 threads per block and `atomicMin` for the reduction. The node list is copied to the device on each query. RRT* refinement is commented out in this version, so it runs plain RRT. |
| `main.cpp` | Pthreads (draft) | An earlier Pthreads version that uses per-thread minimum arrays. The Makefile builds this file. |

Shared code lives in `dataStructs.h` (point, rectangle, tree-node and thread-argument structs) and `util.h` (distance helpers).

## Repository structure

```
.
├── omp.cpp, pthread.cpp, mpi.cpp      # Planner variants (one main() each)
├── cuda.cpp, cuda.cu                  # CUDA host program and kernel
├── main.cpp                           # Makefile build target
├── dataStructs.h, util.h, main.h      # Shared types and helpers
├── Makefile                           # Builds main.cpp into ./master
├── mpi_run.txt                        # MPI build and launch commands
├── hosts, host.txt, config            # MPI hostfiles and SSH host aliases for a cluster
├── visualize.py                       # Renders map + tree + path to images/
├── inputs/                            # Maps (map_1..map_3) and map_template.txt
├── outputs/                           # Sample node/path outputs and format templates
├── images/                            # Rendered maps
└── main, master, pthread, cuda, main.o  # Prebuilt Linux x86-64 binaries
```

## Requirements

- A C++11 compiler with OpenMP support (the Makefile uses `g++ -m64 -std=c++11 -O3 -fopenmp`)
- POSIX threads for the Pthreads version
- An MPI implementation that provides `mpicxx` and `mpirun` for the MPI version
- The CUDA toolkit and an NVIDIA GPU for the CUDA version
- Python 3 with OpenCV (`cv2`) and NumPy for the visualizer

## Build

The Makefile's default target compiles `main.cpp` into `master`:

```
make
make clean
```

Note: in its current state `main.cpp` reads `min_d2_array` and `min_d2_index_array` from the thread-argument struct, but `dataStructs.h` does not declare those fields. `make` will fail until the two files are brought back in sync.

The MPI version is built and launched with the commands recorded in `mpi_run.txt`:

```
mpicxx mpi.cpp -o mpi
mpirun -np 4 --hostfile hosts mpi -f inputs/map_1.txt -n 4 -r 0 -d 5
```

`omp.cpp`, `pthread.cpp` and `cuda.cpp`/`cuda.cu` have no build script in the repository. Compile them with the same flags as the Makefile, plus the Pthreads or CUDA toolchain as needed.

## Usage

Run from the repository root, because results are written to `outputs/`:

```
./master -f inputs/map_1.txt -n 4 -r 0 -d 5
```

| Flag | Meaning | Default |
| --- | --- | --- |
| `-f` | Input map file (required) | none |
| `-n` | Number of threads. It is also used in the output file names. | 1 |
| `-r` | `0` runs RRT, `1` runs RRT* | 0 |
| `-d` | Step distance used when growing a new node | 5 |

Each run writes two files:

- `outputs/<map>_<n>_nodes.txt`: the node count, then one `x y parent_index` line per node
- `outputs/<map>_<n>_path.txt`: the path length and cost, then the path nodes from goal back to start

Format templates are in `outputs/map_1_nodes_template.txt` and `outputs/map_1_path_template.txt`.

### Visualization

```
python visualize.py -i inputs/map_3.txt -n outputs/map_3_4_nodes.txt -p outputs/map_3_4_path.txt
```

The script draws obstacles, tree edges, the path, and the start (green) and goal (red) points on a 4000x4000 canvas. It saves the result to `images/<map>.png`.

## Map format

Maps are plain text (see `inputs/map_template.txt`):

```
dim_x dim_y
num_max_nodes
start_point_x start_point_y
goal_point_x goal_point_y
num_obstacles
x1 y1 x2 y2
...
```

| Map | Size | Max nodes | Obstacles |
| --- | --- | --- | --- |
| `inputs/map_1.txt` | 100 x 100 | 30000 | 1 |
| `inputs/map_2.txt` | 1000 x 1000 | 40000 | 1 |
| `inputs/map_3.txt` | 500 x 500 | 10000 | 8 |

## Acknowledgements

This was developed collaboratively; see the commit history for contributors. The command-line option helpers come from a starter framework. The segment/rectangle collision test is based on a Stack Overflow answer referenced in the source, and the main loop follows the BuildRRT pseudocode from Wikipedia, also quoted in the source.
