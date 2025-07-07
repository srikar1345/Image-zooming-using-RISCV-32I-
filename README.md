# RISC-V CPU 2x Image Zooming Project

This project implements a 2x image zooming algorithm using a custom RISC-V CPU. The CPU reads a 50x50 grayscale image from memory and creates a 100x100 zoomed version using nearest neighbor interpolation.

## Files

- `proc.v` - Main CPU implementation with all modules
- `test.v` - Testbench to run the simulation
- `run_simulation.v` - Combined file for simulation
- `README.md` - This file

## How it works

1. **Source Image**: 50x50 grayscale pixels stored in memory starting at address 0
2. **Destination Image**: 100x100 grayscale pixels stored in memory starting at address 8192
3. **Algorithm**: Nearest neighbor interpolation - each source pixel is mapped to a 2x2 block in the destination

## Algorithm Details

For each destination pixel (x, y):
- Source pixel = (x/2, y/2)
- Load source pixel value
- Store to destination address = dest_base + y * dest_width + x

## Running the Simulation

To run the simulation using a Verilog simulator (like ModelSim, Icarus Verilog, or Verilator):

```bash
# Using Icarus Verilog
iverilog -o zoom_sim run_simulation.v
vvp zoom_sim

# Using Verilator
verilator --cc run_simulation.v
make -C obj_dir -f Vrun_simulation.mk
./obj_dir/Vrun_simulation
```

## Expected Output

The simulation will:
1. Load the source image from memory addresses 0-2499
2. Process each pixel using the 2x zooming algorithm
3. Store the zoomed image to memory addresses 8192-16191
4. Generate a waveform file `zoom_simulation.vcd` for analysis

## Memory Layout

- **Source Image**: Addresses 0-2499 (50x50 pixels)
- **Destination Image**: Addresses 8192-16191 (100x100 pixels)
- **Instruction Memory**: Addresses 0-16383 (4KB)

## CPU Features

- RISC-V RV32I instruction set
- 32 general-purpose registers
- ALU with arithmetic, logical, and multiplication operations
- Memory interface for 8-bit grayscale pixel data
- Branch and jump instructions for control flow

## Fixed Issues

1. **Instruction Encoding**: Corrected all RISC-V instruction encodings
2. **Register Usage**: Fixed register conflicts and usage patterns
3. **Memory Addressing**: Proper source and destination address calculations
4. **Loop Control**: Corrected branch instruction offsets and conditions
5. **ALU Support**: Added multiplication support for address calculations
6. **Testbench**: Created proper simulation environment 