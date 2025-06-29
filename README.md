# Sensor Processing Simulation

This repository contains a small C++17 program that models a simplified spacecraft sensor chain. Multiple sensors run concurrently, their data is processed at a fixed rate and a basic FDIR (Fault Detection, Isolation and Recovery) component monitors the overall health.

The code has no external dependencies beyond the C++ standard library.

For a detailed architecture description and available scenarios see [SIMULATION.md](SIMULATION.md).

## Quick Start

```bash
# build the simulator
g++ -std=c++17 -pthread src/main.cpp -o sim

# run one of the scenarios
./sim nominal    # all sensors healthy
./sim imu_fail   # IMUs fail sequentially
./sim gnss_fail  # GNSS dropout
```

Two log files are created for each run: `<scenario>_proc.log` with the filtered data and `<scenario>_fdir.log` containing alarm messages.

## Running on a Fresh Machine

1. Install a modern C++ compiler, e.g. on Ubuntu:
   ```bash
   sudo apt-get update && sudo apt-get install g++
   ```
2. Clone this repository and change into its directory.
3. Compile and execute the desired scenario as shown above.

The program should work on any Linux system with POSIX threads and a C++17 compiler.
