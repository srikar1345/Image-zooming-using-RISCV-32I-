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

## Single Cycle Processor Architecture

A **single cycle processor** is a CPU design where each instruction is executed in exactly one clock cycle. This means that all stages of instruction execution (fetch, decode, execute, memory access, and write-back) are completed within a single cycle. The main characteristics are:

- **Simplicity**: All instructions take the same amount of time, making control logic straightforward.
- **No pipelining**: There is no overlap between instructions; each instruction completes before the next begins.
- **Performance**: The clock period must be long enough to accommodate the slowest instruction, which can limit maximum speed.
- **Components**:
  - **Instruction Memory**: Stores program instructions.
  - **Register File**: 32 general-purpose registers for computation.
  - **ALU (Arithmetic Logic Unit)**: Performs arithmetic and logical operations.
  - **Data Memory**: For load/store instructions.
  - **Control Unit**: Generates control signals based on the instruction opcode.

In this project, the single cycle processor implements the RISC-V RV32I instruction set and is responsible for reading the image, performing the zoom operation, and writing the result back to memory.

## RISC-V Instruction Set Overview

The **RISC-V RV32I** is a 32-bit integer instruction set architecture (ISA) that is open and simple. Key features include:

- **Register-Register Operations**: ADD, SUB, AND, OR, XOR, SLT, etc.
- **Immediate Operations**: ADDI, ANDI, ORI, etc.
- **Load/Store**: LB, LH, LW, SB, SH, SW for memory access.
- **Branching**: BEQ, BNE, BLT, BGE, etc. for conditional execution.
- **Jumps**: JAL, JALR for function calls and returns.
- **Shift Operations**: SLL, SRL, SRA for bitwise shifts.
- **Simple Encoding**: Fixed 32-bit instruction width, with fields for opcode, registers, and immediates.

This project uses these instructions to implement loops, memory access, and arithmetic needed for the image zooming algorithm.

## Nearest Neighbour Algorithm (2x Image Zoom) - Detailed Explanation

The **nearest neighbour interpolation** algorithm is a simple method for image scaling. For 2x zoom:

- Each pixel in the source image is mapped to a 2x2 block in the destination image.
- The value of each destination pixel is copied from the nearest source pixel (no averaging or interpolation).

### Mathematical Formulation

Let:
- Source image size: \( W_s \times H_s \) (here, 50x50)
- Destination image size: \( W_d \times H_d \) (here, 100x100)
- For each destination pixel at coordinates \( (x_d, y_d) \):

\[
  x_s = \left\lfloor \frac{x_d}{2} \right\rfloor \\
  y_s = \left\lfloor \frac{y_d}{2} \right\rfloor
\]

- The value of the destination pixel is:

\[
  \text{dest}[y_d][x_d] = \text{src}[y_s][x_s]
\]

- In memory (flattened 2D array):

\[
  \text{src\_addr} = y_s \times W_s + x_s \\
  \text{dest\_addr} = y_d \times W_d + x_d
\]

- For 2x zoom, each source pixel at (x, y) is written to four destination pixels:
  - (2x, 2y), (2x+1, 2y), (2x, 2y+1), (2x+1, 2y+1)

### Example
- Source pixel at (10, 20) maps to destination pixels (20, 40), (21, 40), (20, 41), (21, 41).

### Why Nearest Neighbour?
- **Fast**: Only requires integer arithmetic (no division or floating point).
- **Simple**: Easy to implement in hardware and software.
- **Drawback**: Can produce blocky or pixelated images, but is ideal for simple zooming tasks.

## Input vs Output Comparison

| Input Image (50x50) | Output Image (100x100, 2x Zoomed) |
|---------------------|------------------------------------|
| ![Input](Input.jpg) | ![Output](zoomed_output.png) |

The left image is the original 50x50 grayscale input. The right image is the 2x zoomed output generated by the RISC-V CPU using nearest neighbour interpolation. 
