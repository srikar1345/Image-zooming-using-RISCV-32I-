module cpu (
    input clk, 
    input reset,
    output reg [31:0] iaddr,
    input [31:0] idata,
    output [31:0] daddr,
    input [7:0] drdata,         // 8-bit for grayscale pixel
    output [7:0] dwdata,        // 8-bit for grayscale pixel
    output write_en,            // Single write enable bit
    output [31:0] debug_x5,     // Debug signals for algorithm tracking
    output [31:0] debug_x6,
    output [31:0] debug_x7,
    output [31:0] debug_x8,
    output [31:0] debug_x9,
    output [31:0] debug_x10,
    output [31:0] debug_x11,
    output [31:0] debug_x15,
    output [31:0] debug_x13
);

    reg [4:0] regAddr1;
    reg [4:0] regAddr2;
    reg [4:0] regAddr3;
    wire [31:0] regVal1;
    wire [31:0] regVal2;
    wire [31:0] dataIn;
    reg wr;

    wire [4:0] w_regAddr1;
    wire [4:0] w_regAddr2;
    wire [4:0] w_regAddr3;
    wire w_wr;

    assign w_regAddr1 = regAddr1;
    assign w_regAddr2 = regAddr2;
    assign w_regAddr3 = regAddr3;
    assign w_wr = wr;

    wire [31:0] regs [0:31];
    regFile rf(clk, reset, w_regAddr1, w_regAddr2, w_regAddr3, dataIn, w_wr, regVal1, regVal2, debug_x5, debug_x6, debug_x7, debug_x8, debug_x9, debug_x10, debug_x11, debug_x15);

    reg [31:0] r1;
    reg [31:0] r2;
    reg [4:0] forcedAluOpcode;
    wire [31:0] res;

    wire [31:0] w_r1;
    wire [31:0] w_r2;
    wire [4:0] alu_opcode;
    wire eqFlag;

    assign w_r1 = regVal1;
    assign w_r2 = (idata[6:0] == 7'b0010011 || idata[6:0] == 7'b0110111 || idata[6:0] == 7'b1100111 || idata[6:0] == 7'b1101111 || idata[6:0] == 7'b0010111)? r2: regVal2;
    assign alu_opcode = (idata[6:0] == 7'b1100011 || idata[6:0] == 7'b0110111 || idata[6:0] == 7'b1100111 || idata[6:0] == 7'b1101111)? forcedAluOpcode : {idata[5], idata[30], idata[14:12]};

    alu myALU(w_r1, w_r2, alu_opcode, res, eqFlag);

    wire [31:0] mem_offset;
    assign mem_offset = (idata[6:0] == 7'b0000011)? {{20{idata[31]}}, idata[31:20]} : {{20{idata[31]}}, idata[31:25], idata[11:7]};
    assign daddr = regVal1 + mem_offset;

    // Simplified masker for 8-bit data
    simple_masker muxForRegfileDataIn(
        .opcode(idata[6:0]), 
        .drdata(drdata), 
        .ALU_res(res), 
        .current_pc(iaddr),
        .funct3(idata[14:12]),
        .maskedDataIn(dataIn)
    );
    
    // Simplified store masker for 8-bit data
    simple_store_masker muxForDataInDMEM(
        .reset(reset), 
        .opcode(idata[6:0]), 
        .dataIn(regVal2),
        .funct3(idata[14:12]),
        .write_en(write_en), 
        .dwdata(dwdata)
    );

    reg shouldIBranch = 1'b0;
    reg [31:0] pc_branch;
    wire [31:0] pc_next;
    assign pc_next = (shouldIBranch == 1'b1)? pc_branch : (iaddr + 4);

    //changing pc
    always @(*) begin
        if (idata[6:0] == 7'b1100011) begin //Branch instructions control logic
            pc_branch = iaddr + {{20{idata[31]}}, idata[31], idata[7], idata[30:25], idata[11:8], 1'b0};
            case (idata[14:12])
                3'b000: if (eqFlag == 1'b1) shouldIBranch = 1'b1; else shouldIBranch = 1'b0;  //BEQ
                3'b001: if (eqFlag != 1'b1) shouldIBranch = 1'b1; else shouldIBranch = 1'b0; //BNE
                3'b100: if (eqFlag != 1'b1 && res == 1) shouldIBranch = 1'b1; else shouldIBranch = 1'b0; //BLT
                3'b101: if (eqFlag == 1'b1 || res == 0) shouldIBranch = 1'b1; else shouldIBranch = 1'b0; //BGE
                3'b110: if (eqFlag != 1'b1 && res == 1) shouldIBranch = 1'b1; else shouldIBranch = 1'b0; //BLTU
                3'b111: if (eqFlag == 1'b1 || res == 0) shouldIBranch = 1'b1; else shouldIBranch = 1'b0; //BGEU
                default: shouldIBranch = 1'b0;
            endcase
            $display("BRANCH: PC=%h, opcode=%b, funct3=%b, eqFlag=%b, res=%d, shouldBranch=%b, target=%h", 
                     iaddr, idata[6:0], idata[14:12], eqFlag, res, shouldIBranch, pc_branch);
        end
        else if (idata[6:0] == 7'b1101111) begin //JAL
            $display("JAL: PC=%h, target=%h", iaddr, iaddr + {{11{idata[31]}}, idata[31], idata[19:12], idata[20], idata[30:21], 1'b0});
            shouldIBranch = 1'b1;
            pc_branch = iaddr + {{11{idata[31]}}, idata[31], idata[19:12], idata[20], idata[30:21], 1'b0};
        end
        else if (idata[6:0] == 7'b1100111) begin //JALR
            $display("JALR: PC=%h, rs1=%h, offset=%h, target=%h", iaddr, regVal1, {{20{idata[31]}}, idata[31:20]}, (regVal1 + {{20{idata[31]}}, idata[31:20]}) & ~32'b1);
            shouldIBranch = 1'b1;
            pc_branch = (regVal1 + {{20{idata[31]}}, idata[31:20]}) & ~32'b1; // Clear LSB after addition
        end
        else  begin
            shouldIBranch = 1'b0;
            pc_branch = 0;
        end
    end
    
    // Add detailed instruction execution tracking
    always @(posedge clk) begin
        if (!reset && idata != 0) begin
            // Commented out to reduce debug output
            /*
            case (idata[6:0])
                7'b0010011: $display("EXEC: ADDI at PC=%h, rd=%d, rs1=%d, imm=%h", iaddr, idata[11:7], idata[19:15], {{20{idata[31]}}, idata[31:20]});
                7'b0110011: $display("EXEC: ALU at PC=%h, rd=%d, rs1=%d, rs2=%d, funct3=%b", iaddr, idata[11:7], idata[19:15], idata[24:20], idata[14:12]);
                7'b0000011: $display("EXEC: LOAD at PC=%h, rd=%d, rs1=%d, offset=%h", iaddr, idata[11:7], idata[19:15], {{20{idata[31]}}, idata[31:20]});
                7'b0100011: $display("EXEC: STORE at PC=%h, rs1=%d, rs2=%d, offset=%h", iaddr, idata[19:15], idata[24:20], {{20{idata[31]}}, idata[31:25], idata[11:7]});
                7'b1100011: $display("EXEC: BRANCH at PC=%h, rs1=%d, rs2=%d, funct3=%b", iaddr, idata[19:15], idata[24:20], idata[14:12]);
                7'b1101111: $display("EXEC: JAL at PC=%h, rd=%d", iaddr, idata[11:7]);
                7'b1100111: $display("EXEC: JALR at PC=%h, rd=%d, rs1=%d", iaddr, idata[11:7], idata[19:15]);
                7'b0110111: $display("EXEC: LUI at PC=%h, rd=%d, imm=%h", iaddr, idata[11:7], {idata[31:12], 12'b0});
                7'b0010111: $display("EXEC: AUIPC at PC=%h, rd=%d, imm=%h", iaddr, idata[11:7], {idata[31:12], 12'b0});
                default: $display("EXEC: UNKNOWN at PC=%h, instruction=%h", iaddr, idata);
            endcase
            */
        end
    end
    
    //regfile control signals
    always @(*) begin
        regAddr1 = 0;
        regAddr2 = 0;
        regAddr3 = 0;
        wr = 0;
        forcedAluOpcode = 0;
        r1 = 0;
        r2 = 0;
        if (idata[6:0] == 7'b0010011) begin //Immediate mode ALU instructions ALU inputs and regfile control
            // $display("ADDI: PC=%h, rd=%d, rs1=%d, imm=%h", iaddr, idata[11:7], idata[19:15], {{20{idata[31]}}, idata[31:20]});
            regAddr3 = idata[11:7];
            regAddr1 = idata[19:15];
            wr = 1'b1;
            r2 = {{20{idata[31]}}, idata[31:20]};
        end
        else if (idata[6:0] == 7'b0110011) begin //Non immediate mode ALU instructions ALU inputs and regfile control
            // $display("ALU: PC=%h, rd=%d, rs1=%d, rs2=%d, funct3=%b", iaddr, idata[11:7], idata[19:15], idata[24:20], idata[14:12]);
            regAddr3 = idata[11:7];
            regAddr1 = idata[19:15];
            regAddr2 = idata[24:20];
            wr = 1'b1;
            r2 = regVal2;
        end
        else if (idata[6:0] == 7'b0000011) begin //Load instructions regfile control
            // $display("LOAD: PC=%h, rd=%d, rs1=%d, offset=%h", iaddr, idata[11:7], idata[19:15], {{20{idata[31]}}, idata[31:20]});
            regAddr3 = idata[11:7];
            regAddr1 = idata[19:15];
            wr = 1'b1;
        end
        else if (idata[6:0] == 7'b0100011) begin //Store instructions regfile control
            // $display("STORE: PC=%h, rs1=%d, rs2=%d, offset=%h", iaddr, idata[19:15], idata[24:20], {{20{idata[31]}}, idata[31:25], idata[11:7]});
            wr = 1'b0;
            regAddr1 = idata[19:15];
            regAddr2 = idata[24:20];
        end
        else if (idata[6:0] == 7'b1100011) begin //branch instructions ALU inputs
            $display("BRANCH_CTRL: PC=%h, rs1=%d, rs2=%d, funct3=%b", iaddr, idata[19:15], idata[24:20], idata[14:12]);
            wr = 1'b0;
            regAddr1 = idata[19:15];
            regAddr2 = idata[24:20];
            if (idata[13] == 1'b1) forcedAluOpcode = 5'b10011;
            else forcedAluOpcode = 5'b10010;
        end
        else if (idata[6:0] == 7'b1101111) begin //JAL regfile control
            $display("JAL_CTRL: PC=%h, rd=%d", iaddr, idata[11:7]);
            regAddr3 = idata[11:7];
            regAddr2 = 0;
            regAddr1 = 0;  //x0 = 0
            wr = 1'b1;
            r2 = iaddr + 4;
            forcedAluOpcode = 5'b00000; //ADDI this so we take advantage of our prebuilt routing via alu to update the reg
        end
        else if (idata[6:0] == 7'b1100111) begin //JALR regfile control
            $display("JALR_CTRL: PC=%h, rd=%d, rs1=%d", iaddr, idata[11:7], idata[19:15]);
            regAddr3 = idata[11:7];
            regAddr2 = 0;
            regAddr1 = idata[19:15];
            wr = 1'b1;
            r2 = iaddr + 4;
            forcedAluOpcode = 5'b00000;
        end
        else if (idata[6:0] == 7'b0110111) begin //LUI
            $display("LUI: PC=%h, rd=%d, imm=%h", iaddr, idata[11:7], {idata[31:12], 12'b0});
            regAddr3 = idata[11:7];
            regAddr1 = 0;  //x0 = 0
            r2 = {idata[31:12], 12'b0};  //immideate mode value as specified in ISA
            wr = 1'b1;
            forcedAluOpcode = 5'b00000; //ADDI this so we take advantage of our prebuilt routing via alu to update the reg
        end
        else if (idata[6:0] == 7'b0010111) begin //AUIPC
            // $display("AUIPC: PC=%h, rd=%d, imm=%h, result=%h", iaddr, idata[11:7], {idata[31:12], 12'b0}, iaddr + {idata[31:12], 12'b0});
            regAddr3 = idata[11:7];
            regAddr1 = 0;  //x0 = 0
            r2 = iaddr + {idata[31:12], 12'b0};  //immideate mode value as specified in ISA
            wr = 1'b1;
            forcedAluOpcode = 5'b00000; //ADDI this so we take advantage of our prebuilt routing via alu to update the reg
        end
        else begin
            wr = 1'b0;
        end
    end

    // In the cpu module, modify the PC update logic:
always @(posedge clk) begin
    if (reset) begin
        iaddr <= 0;
        $display("reset state");
    end else begin
            if (idata != 0) begin
                // $display("INSTRUCTION : %h at PC = %h", idata, iaddr);
                // $display("REGS: x5(src)=%h x6(dcol)=%h x7(swidth)=%h x8(dwidth)=%h x9(drow)=%h x10(daddr)=%h x11(tmp)=%h", debug_x5, debug_x6, debug_x7, debug_x8, debug_x9, debug_x10, debug_x11);
                // $display("PC Update: current=%h, next=%h, shouldBranch=%b", iaddr, pc_next, shouldIBranch);
                // $display("FETCH_DEBUG: Next instruction at PC=%h is %h", pc_next, imem_inst.mem[pc_next[31:2]]);
                // if (iaddr == pc_next) $display("WARNING: PC did not change! Possible infinite loop or halt.");
            end
        iaddr <= pc_next;
    end
end

    // Add debug output for register writes
    always @(posedge clk) begin
        if (!reset && wr && idata[11:7] != 0) begin
            // $display("REG_WRITE: PC=%h, rd=%d, value=%h", iaddr, idata[11:7], dataIn);
        end
    end

    // Add debug output for ALU operations
    always @(posedge clk) begin
        if (!reset && (idata[6:0] == 7'b0110011 || idata[6:0] == 7'b0010011 || idata[6:0] == 7'b1100011)) begin
            // $display("ALU_OP: PC=%h, opcode=%b, funct3=%b, rs1_val=%h, rs2_val=%h, result=%h", 
            //          iaddr, idata[6:0], idata[14:12], regVal1, r2, res);
        end
    end

    // Add debug output for memory operations
    always @(posedge clk) begin
        if (!reset && idata[6:0] == 7'b0000011) begin // LOAD (lb)
            $display("CPU_MEM_READ: daddr=%0d, x10=%0d, drdata=%0h", daddr, debug_x10, drdata);
        end
        if (!reset && idata[6:0] == 7'b0100011) begin // STORE (sb)
            $display("CPU_MEM_WRITE: daddr=%0d, x10=%0d, dwdata=%0h", daddr, debug_x10, dwdata);
        end
    end

    // Add specific debug output for branch decisions
    always @(posedge clk) begin
        if (!reset && idata[6:0] == 7'b1100011) begin
            $display("BRANCH_DEBUG: PC=%h, x6(dcol)=%d, x9(drow)=%d, x8(dwidth)=%d", iaddr, debug_x6, debug_x9, debug_x8);
        end
    end

    always @(posedge clk) begin
        if (!reset && iaddr == 32'h18) begin
            $display("LOOP_ENTRY_DEBUG: Entered main zooming loop at PC=0x18");
        end
    end

    // Local cycle counter for tracing
    reg [31:0] cpu_cycle_count = 0;
    always @(posedge clk) begin
        if (reset) begin
            cpu_cycle_count <= 0;
        end else begin
            cpu_cycle_count <= cpu_cycle_count + 1;
        end
    end

    always @(posedge clk) begin
        if (!reset) begin
            $display("TRACE_PC: Cycle=%0d, PC=%h, idata=%h", cpu_cycle_count, iaddr, idata);
        end
    end

    always @(posedge clk) begin
        if (!reset && iaddr >= 32'h18 && iaddr <= 32'h54) begin
            $display("LOOP_DEBUG: Cycle=%0d, PC=%h, x6(dcol)=%d, x9(drow)=%d", cpu_cycle_count, iaddr, debug_x6, debug_x9);
        end
    end

    always @(posedge clk) begin
        if (!reset && idata == 32'h0000006f) begin // jal x0, 0 (halt)
            $display("HALT instruction executed at PC=%h, cycle=%0d", iaddr, cpu_cycle_count);
        end
    end

    integer debug_cycle_count = 0;
    always @(posedge clk) begin
        if (reset) debug_cycle_count = 0;
        else debug_cycle_count = debug_cycle_count + 1;
        // Print debug info for each store to destination region (limit to first 2000 cycles)
        if (!reset && write_en && daddr >= 8192 && debug_cycle_count <= 2000) begin
            $display("ZOOM_DEBUG: cycle=%0d, src_addr=%0d, src_val=%h, dest_addr=%0d, stored_val=%h, x11(src_addr)=%0d, x13(pixel)=%h", debug_cycle_count, debug_x11, debug_x13, daddr, dwdata, debug_x11, debug_x13);
        end
    end

    reg [31:0] x13_pixel; // Register to hold pixel value (x13)
    assign debug_x13 = x13_pixel;

    // Print simulation start and register values at the start of the program
    initial begin
        $display("SIM STARTED");
        $display("INIT_REGS: x5(src_base)=%0d, x7(src_width)=%0d, x8(dest_width)=%0d, x9(dest_row)=%0d, x10(dest_base)=%0d", debug_x5, debug_x7, debug_x8, debug_x9, debug_x10);
    end
    // Print register values and addresses before every memory access
    always @(posedge clk) begin
        if (!reset && (idata[6:0] == 7'b0000011 || idata[6:0] == 7'b0100011)) begin // LOAD or STORE
            $display("MEM_ACCESS: x5=%0d, x7=%0d, x8=%0d, x9=%0d, x10=%0d, src_addr=%0d, dest_addr=%0d, daddr=%0d", debug_x5, debug_x7, debug_x8, debug_x9, debug_x10, debug_x15, debug_x10, daddr);
        end
    end

endmodule

module alu(
    input [31:0] rs1,
    input [31:0] rs2,
    input [4:0] alu_opcode, // {opcode[5], funct7[5], funct3[2:0]}
    output reg [31:0] res,
    output eqFlag
);
    always @(rs1, rs2, alu_opcode) begin
        casex(alu_opcode)
            5'b0x000: res = rs1 + rs2; //ADDI
            5'b10000: res = rs1 + rs2; //ADD
            5'b11000: res = rs1 - rs2; //SUB
            5'b0x111: res = rs1 & rs2; //AND / ANDI
            5'b0x110: res = rs1 | rs2; //OR / ORI
            5'b0x100: res = rs1 ^ rs2; //XOR / XORI
            5'b00101: res = rs1 >> rs2[4:0]; //SRL / SRLI
            5'b00001: res = rs1 << rs2[4:0]; //SLL / SLLI
            5'b01101: res = $signed(rs1) >>> rs2[4:0]; //SRA / SRAI
            5'b0x010: if ($signed(rs1) < $signed(rs2)) res = 1; else res = 0; //SLT / SLTI
            5'b0x011: if (rs1 < rs2) res = 1; else res = 0; //SLTU / SLTIU
            5'b10001: res = rs1 * rs2; //MUL
            default: res = 0;
        endcase
        // $display("ALU_inputs %d, %d, %d, %d", $signed(rs1), $signed(rs2), alu_opcode, $signed(res));
    end

    assign eqFlag = (rs1 == rs2)? 1 : 0;
endmodule

`timescale 1ns/1ps
`define DMEM_N_FILE(x,y) {x,y,".mem"}
module image_mem (
    input clk,
    input [31:0] addr,
    input [7:0] write_data,
    input write_en,
    output [7:0] read_data
);
    // Define memory for image pixels (grayscale - 8 bits per pixel)
    reg [7:0] mem [0:16383]; // 16K pixels (enough for source and destination)
    
    // Use only the lower bits of the address
    wire [13:0] addr_masked;
    assign addr_masked = addr[13:0];
    
    integer i;
    integer non_zero_count;
    
    initial begin
        $display("=== IMAGE MEMORY INITIALIZATION ===");
        $display("Starting image memory initialization...");
        
        // Initialize memory to zeros
        $display("Initializing memory to zeros...");
        for (i = 0; i < 16384; i = i + 1) begin
            mem[i] = 8'h00;
        end
        $display("Memory zeroed. First 10 values after zeroing:");
        for (i = 0; i < 10; i = i + 1) begin
            $display("mem[%0d] = %h", i, mem[i]);
        end
        
        // Load image data from file
        $display("Loading image data from imagek.mem...");
        $readmemh("imagek.mem", mem); // Load without range specification
        $display("Image loading completed. First 10 values after loading:");
        for (i = 0; i < 10; i = i + 1) begin
            $display("mem[%0d] = %h", i, mem[i]);
        end
        
        // Check if file was loaded properly by looking for non-zero values
        $display("Checking for non-zero values in first 100 memory locations:");
        non_zero_count = 0;
        for (i = 0; i < 100; i = i + 1) begin
            if (mem[i] != 8'h00) begin
                $display("Found non-zero value: mem[%0d] = %h", i, mem[i]);
                non_zero_count = non_zero_count + 1;
            end
        end
        $display("Total non-zero values in first 100 locations: %0d", non_zero_count);
        $display("=== IMAGE MEMORY INITIALIZATION COMPLETE ===");
    end
    
    // Read operation (combinational) with address masking
    assign read_data = mem[addr_masked];
    
    // image_mem debug: print on every read from 0-2499
    always @(*) begin
        if (!write_en && addr_masked < 2500) begin
            $display("MEM_READ_DEBUG: src_addr=%0d, src_val=%h", addr_masked, mem[addr_masked]);
        end
    end
    
    // Write operation (sequential) with address masking
    always @(posedge clk) begin
        if (write_en && addr_masked >= 8192) begin
            $display("MEM_WRITE_DEBUG: dest_addr=%0d, stored_val=%h", addr_masked, write_data);
        end
    end
endmodule


`timescale 1ns/1ps
module imem (
    input [31:0] iaddr,
    output [31:0] idata
);
    // Ignores LSB 2 bits, so will not generate alignment exception
    reg [31:0] mem[0:4095]; // Define a 4-K location memory (16KB)
    
    // Explicitly initialize all memory to NOPs
    integer i;
    initial begin
        // Initialize all memory to NOPs first
        for (i = 0; i < 4096; i = i + 1) begin
            mem[i] = 32'h00000013; // addi x0, x0, 0 (NOP)
        end
        // 2x Zooming algorithm - Nearest Neighbor Interpolation (CORRECTED)
        // Source image: 50x50 pixels at address 0
        // Destination image: 100x100 pixels at address 8192
        // Registers: x5=src_base, x6=dest_col, x7=src_width, x8=dest_width, x9=dest_row, x10=dest_base, x13=pixel, x14=src_row, x15=src_col, x16=src_addr, x17=dest_addr
        mem[0] = 32'h00000293; // addi x5, x0, 0      # x5 = src_base = 0
        mem[1] = 32'h7ff00513; // addi x10, x0, 2047  # x10 = 2047
        mem[2] = 32'h7ff50513; // addi x10, x10, 2047 # x10 = 4094
        mem[3] = 32'h7ff50513; // addi x10, x10, 2047 # x10 = 6141
        mem[4] = 32'h7ff50513; // addi x10, x10, 2047 # x10 = 8188
        mem[5] = 32'h00450513; // addi x10, x10, 4    # x10 = 8192
        mem[6] = 32'h03200393; // addi x7, x0, 50      # x7 = src_width = 50
        mem[7] = 32'h06400413; // addi x8, x0, 100     # x8 = dest_width = 100
        mem[8] = 32'h00000493; // addi x9, x0, 0       # x9 = dest_row = 0
        // row_loop:
        mem[9] = 32'h00000313; // addi x6, x0, 0       # x6 = dest_col = 0
        // col_loop:
        mem[10] = 32'h4014a713; // srai x14, x9, 1      # x14 = drow >> 1 (src_row)
        mem[11] = 32'h40132793; // srai x15, x6, 1      # x15 = dcol >> 1 (src_col)
        mem[12] = 32'h00770733; // mul x14, x14, x7     # x14 = src_row * src_width
        mem[13] = 32'h00f707b3; // add x15, x14, x15    # x15 = src_row*src_width + src_col
        mem[14] = 32'h00578733; // add x14, x15, x5     # x14 = src_addr = src_base + ...
        mem[15] = 32'h0007a283; // lb x13, 0(x14)       # x13 = mem[src_addr]
        mem[16] = 32'h009487b3; // mul x15, x9, x8      # x15 = dest_row * dest_width
        mem[17] = 32'h006787b3; // add x15, x15, x6     # x15 = dest_row*dest_width + dest_col
        mem[18] = 32'h00a787b3; // add x15, x15, x10    # x15 = dest_addr
        mem[19] = 32'h00d7a023; // sb x13, 0(x15)       # mem[dest_addr] = x13
        mem[20] = 32'h00130313; // addi x6, x6, 1       # x6++
        mem[21] = 32'hfe631ae3; // blt x6, x8, col_loop (PC=6)
        mem[22] = 32'h00148493; // addi x9, x9, 1       # x9++
        mem[23] = 32'hfe849ae3; // blt x9, x8, row_loop (PC=5)
        mem[24] = 32'h0000006f; // jal x0, 0            # halt
    end

    // Read operation with bounds checking
    assign idata = (iaddr[31:2] < 4096) ? mem[iaddr[31:2]] : 32'h00000013;
endmodule

// Updated masker for 8-bit data with funct3 support
module simple_masker(
    input [6:0] opcode,
    input [7:0] drdata,        // 8-bit pixel data
    input [31:0] ALU_res,
    input [31:0] current_pc,
    input [2:0] funct3,        // Added to handle different load types
    output [31:0] maskedDataIn
);
    reg [31:0] masked;
    assign maskedDataIn = masked;
    
    always @(*) begin
        if (opcode == 7'b0000011) begin  // Load instruction
            case (funct3)
                3'b000: masked = {{24{drdata[7]}}, drdata};      // LB - sign extend
                3'b001: masked = {{16{drdata[7]}}, drdata, 8'h00}; // LH - treat as 2 pixels (with sign extension)
                3'b010: masked = {drdata, drdata, drdata, drdata}; // LW - duplicate pixel to all bytes
                3'b100: masked = {24'b0, drdata};               // LBU - zero extend
                3'b101: masked = {16'b0, drdata, 8'h00};        // LHU - treat as 2 pixels (zero extended)
                default: masked = {24'b0, drdata};              // Default to byte load zero extended
            endcase
        end
        else if (opcode == 7'b1100111) begin  // JALR
            masked = current_pc + 4;
        end
        else masked = ALU_res;
    end
endmodule

// Updated store masker for 8-bit data with funct3 support
module simple_store_masker(
    input reset,
    input [6:0] opcode,
    input [31:0] dataIn,
    input [2:0] funct3,        // Added to handle different store types
    output reg write_en,
    output reg [7:0] dwdata
);
    always @(*) begin
        if (reset == 1'b1) begin
            write_en = 1'b0;
            dwdata = 8'b0;
        end
        else if (opcode == 7'b0100011) begin  // Store instruction
            case (funct3)
                3'b000: begin  // SB - store byte
                    dwdata = dataIn[7:0];
                    write_en = 1'b1;
                end
                3'b001: begin  // SH - store halfword (use LSB)
                    dwdata = dataIn[7:0];  // For grayscale, we just use the LSB
                    write_en = 1'b1;
                end
                3'b010: begin  // SW - store word (use LSB)
                    dwdata = dataIn[7:0];  // For grayscale, we just use the LSB
                    write_en = 1'b1;
                end
                default: begin
                    dwdata = 8'b0;
                    write_en = 1'b0;
                end
            endcase
        end
        else begin
            dwdata = 8'b0;
            write_en = 1'b0;
        end
    end
endmodule

module regFile(
    input clk,
    input rst,
    input [4:0] regAddr1, //read Addr line 1
    input [4:0] regAddr2, //read Addr line 2
    input [4:0] regAddr3, //write Addr line
    input [31:0] dataIn,
    input wr,
    output reg [31:0] dataOut1,
    output reg [31:0] dataOut2,
    output [31:0] debug_x5,
    output [31:0] debug_x6,
    output [31:0] debug_x7,
    output [31:0] debug_x8,
    output [31:0] debug_x9,
    output [31:0] debug_x10,
    output [31:0] debug_x11,
    output [31:0] debug_x15
);
    reg [31:0] r [31:0];
    assign debug_x5 = r[5];
    assign debug_x6 = r[6];
    assign debug_x7 = r[7];
    assign debug_x8 = r[8];
    assign debug_x9 = r[9];
    assign debug_x10 = r[10];
    assign debug_x11 = r[11];
    assign debug_x15 = r[15];

    always @(*) begin
        dataOut1 = r[regAddr1];
        dataOut2 = r[regAddr2];
    end

    always @(posedge clk) begin
        if (rst == 1'b1) begin
            r[0] <= 32'b0;
            r[1] <= 32'b0;
            r[2] <= 32'b0;
            r[3] <= 32'b0;
            r[4] <= 32'b0;
            r[5] <= 32'b0;
            r[6] <= 32'b0;
            r[7] <= 32'b0;
            r[8] <= 32'b0;
            r[9] <= 32'b0;
            r[10] <= 32'b0;
            r[11] <= 32'b0;
            r[12] <= 32'b0;
            r[13] <= 32'b0;
            r[14] <= 32'b0;
            r[15] <= 32'b0;
            r[16] <= 32'b0;
            r[17] <= 32'b0;
            r[18] <= 32'b0;
            r[19] <= 32'b0;
            r[20] <= 32'b0;
            r[21] <= 32'b0;
            r[22] <= 32'b0;
            r[23] <= 32'b0;
            r[24] <= 32'b0;
            r[25] <= 32'b0;
            r[26] <= 32'b0;
            r[27] <= 32'b0;
            r[28] <= 32'b0;
            r[29] <= 32'b0;
            r[30] <= 32'b0;
            r[31] <= 32'b0;
        end
        else if (wr == 1'b1 && regAddr3 != 0) begin
            $display($time, "Datawrite %d in reg %d", $signed(dataIn), regAddr3);
            r[regAddr3] <= dataIn;
        end
    end
endmodule

// Top-level module to connect CPU with memory
module system (
    input clk,
    input reset,
    output [31:0] debug_x15
);
    wire [31:0] iaddr;
    wire [31:0] idata;
    wire [31:0] daddr;
    wire [7:0] drdata;
    wire [7:0] dwdata;
    wire write_en;
    
    // Debug signals from CPU
    wire [31:0] debug_x5, debug_x6, debug_x7, debug_x8, debug_x9, debug_x10, debug_x11, debug_x13;
    
    // Add a cycle counter
    reg [7:0] cycle_count = 0;
    
    // Cycle counter for debugging
    always @(posedge clk) begin
        if (!reset) begin
            cycle_count <= cycle_count + 1;
            if (cycle_count % 100 == 0) begin
            $display("Cycle %d: PC = %h", cycle_count, iaddr);
            end
        end
    end
    
    // Debug output for algorithm tracking
    always @(posedge clk) begin
        if (!reset) begin
            // Track key registers for zooming algorithm
            if (iaddr >= 32'h0 && iaddr <= 32'h54) begin
                $display("ALGORITHM_DEBUG: Cycle %d, PC=%h, x5=%d, x6=%d, x7=%d, x8=%d, x9=%d, x10=%d, x11=%d", 
                         cycle_count, iaddr, debug_x5, debug_x6, debug_x7, debug_x8, debug_x9, debug_x10, debug_x11);
            end
        end
    end
    
    // Instantiate CPU
    cpu cpu_inst (
        .clk(clk),
        .reset(reset),
        .iaddr(iaddr),
        .idata(idata),
        .daddr(daddr),
        .drdata(drdata),
        .dwdata(dwdata),
        .write_en(write_en),
        .debug_x5(debug_x5),
        .debug_x6(debug_x6),
        .debug_x7(debug_x7),
        .debug_x8(debug_x8),
        .debug_x9(debug_x9),
        .debug_x10(debug_x10),
        .debug_x11(debug_x11),
        .debug_x15(debug_x15),
        .debug_x13(debug_x13)
    );
    
    // Instantiate instruction memory
    imem imem_inst (
        .iaddr(iaddr),
        .idata(idata)
    );
    
    // Instantiate data memory (image memory)
    image_mem dmem_inst (
        .clk(clk),
        .addr(daddr),
        .write_data(dwdata),
        .write_en(write_en),
        .read_data(drdata)
    );

    // Add debug output for register values during the loop
    always @(posedge clk) begin
        if (!reset && iaddr >= 32'h18 && iaddr <= 32'h54 && cycle_count <= 2000) begin
            $display("LOOP_REGS: Cycle=%0d, PC=%h, x10=%d, x9=%d, x6=%d, daddr=%h, write_en=%b, dwdata=%h", cycle_count, iaddr, debug_x10, debug_x9, debug_x6, daddr, write_en, dwdata);
        end
    end

    // Add debug output for memory writes in the CPU
    always @(posedge clk) begin
        if (!reset && write_en) begin
            $display("CPU_MEM_WRITE: daddr=%0h, x10=%0d, write_en=%0d, dwdata=%0h", daddr, debug_x10, write_en, dwdata);
        end
    end
endmodule


