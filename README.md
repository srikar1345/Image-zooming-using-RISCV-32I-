RISC-V CPU 2x Image Zooming Project
This project implements a 2x image zooming algorithm using a custom-built single-cycle RISC-V CPU. The CPU reads a 50x50 grayscale image from memory and generates a 100x100 zoomed version using nearest neighbor interpolation.

📁 Files
File	Description
proc.v	Verilog implementation of the RISC-V single-cycle processor
test.v	Testbench to simulate the CPU and verify output
run_simulation.v	Combined wrapper file for simulation
README.md	This documentation file

🧠 How It Works
Source Image: 50×50 grayscale pixels (2500 bytes) starting at memory address 0

Destination Image: 100×100 grayscale pixels (10,000 bytes) starting at address 8192

Zoom Algorithm: Nearest neighbor interpolation — each source pixel is expanded into a 2×2 block in the destination

🔁 Zooming Algorithm - Nearest Neighbor
Concept: Each pixel in the zoomed (100×100) image corresponds to the nearest pixel in the original (50×50) image.

🔬 Formula
For destination coordinates (x_d, y_d):

Compute source coordinates:
x_s = x_d / 2, y_s = y_d / 2 (integer division)

Fetch pixel at (x_s, y_s) from source image

Store it at (x_d, y_d) in the destination image

🧮 Math:
Let:

dest_base = 8192

dest_width = 100

src_width = 50

Then:

text
Copy
Edit
src_addr = y_s * src_width + x_s
dst_addr = dest_base + y_d * dest_width + x_d
For example:

text
Copy
Edit
For x_d = 4, y_d = 6
  x_s = 2, y_s = 3
  src_addr = 3 * 50 + 2 = 152
  dst_addr = 8192 + 6 * 100 + 4 = 8796
⚙️ CPU Architecture - Single-Cycle Processor
A single-cycle processor executes each instruction in one clock cycle. This is simple and fast for small programs.

🔧 Architecture Diagram
rust
Copy
Edit
[PC] --> [Instruction Memory] --> [Control Unit]
         |                          |
         v                          v
     [Register File] <--> [ALU] <--> [Data Memory]
Key Features:
Instruction fetch, decode, execute, memory, write-back in one cycle

Easy to design, test, and simulate

Ideal for embedded and algorithm-specific tasks

📜 RISC-V Instruction Set (RV32I)
The CPU implements a subset of the RV32I instruction set:

Type	Instructions
Arithmetic	add, sub, addi
Logical	and, or, xor
Memory	lw, sw, lb, sb
Control	beq, bne, jal, jalr
Shift	sll, srl
Multiply	Custom support: mul

The CPU uses 32 general-purpose registers (x0 to x31), each 32 bits wide.

🧪 Running the Simulation
To simulate the CPU using Icarus Verilog:

bash
Copy
Edit
# Compile
iverilog -o zoom_sim run_simulation.v

# Run
vvp zoom_sim
With Verilator:

bash
Copy
Edit
verilator --cc run_simulation.v --exe sim_main.cpp
make -C obj_dir -f Vrun_simulation.mk
./obj_dir/Vrun_simulation
✅ Expected Output
Input Memory: Contains 50×50 grayscale image (addresses 0–2499)

Output Memory: Filled with 100×100 zoomed image (addresses 8192–16191)

Waveform: A zoom_simulation.vcd file is generated for debugging

🗂️ Memory Layout
Region	Description	Address Range
Source Image	50×50 = 2500 pixels	0 to 2499
Zoomed Image	100×100 = 10000 pixels	8192 to 18191
Instruction Memory	Up to 4KB of program	ROM[0] to ROM[1023]

🛠️ CPU Features
✅ Single-cycle RISC-V RV32I

✅ ALU with add, sub, mul, and, or, xor

✅ 32 general-purpose registers

✅ Byte-level memory interface for grayscale pixels
