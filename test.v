// Test file for 2x zooming simulation
`timescale 1ns/1ps

module test;
    reg clk;
    reg rst;
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period (100MHz)
    end
    
    // Test stimulus
    initial begin
        // Add debug output at the beginning
        $display("=== SIMULATION START ===");
        $display("SIM STARTED");
        $display("Testbench starting...");
        
        // Initialize waveform dump
        $dumpfile("zoom_simulation.vcd");
        $dumpvars(0, test);
        
        // Reset the system
        $display("Applying reset...");
        rst = 1;
        #20;
        rst = 0;
        $display("Reset released, starting simulation...");
        
        // Run for a longer time to ensure the loop is executed
        #1000000; // 100,000 cycles
        
        // Dump zoomed image data to file
        $display("Dumping zoomed image data to file...");
        dump_zoomed_image();
        // Dump zoomed image memory to .mem file
        dump_zoomed_image_mem();
        
        // Display some results
        $display("Simulation completed!");
        $display("=== MEMORY DEBUG ===");
        
        // Print first 20 values of source image memory (should match imagek.mem)
        // $display("Source image memory (mem[0] to mem[19]):");
        // for (integer i = 0; i < 20; i = i + 1) begin
        //     $display("mem[%0d] = %h", i, sys_inst.dmem_inst.mem[i]);
        // end
        
        // Print first 20 values of destination (zoomed) image memory (mem[8192] to mem[8211])
        $display("Zoomed image memory (mem[8192] to mem[8211]):");
        for (integer i = 8192; i < 8212; i = i + 1) begin
            $display("mem[%0d] = %h", i, sys_inst.dmem_inst.mem[i]);
        end
        
        // Print more values from source image memory (mem[30] to mem[60])
        // $display("Source image memory (mem[30] to mem[60]):");
        // for (integer i = 30; i <= 60; i = i + 1) begin
        //     $display("mem[%0d] = %h", i, sys_inst.dmem_inst.mem[i]);
        // end
        
        // Print more values from source image memory (mem[70] to mem[100])
        // $display("Source image memory (mem[70] to mem[100]):");
        // for (integer i = 70; i <= 100; i = i + 1) begin
        //     $display("mem[%0d] = %h", i, sys_inst.dmem_inst.mem[i]);
        // end
        
        // Print instruction memory contents to verify program loading
        // $display("=== INSTRUCTION MEMORY DEBUG ===");
        // $display("Instructions loaded in memory:");
        // for (integer i = 0; i < 22; i = i + 1) begin
        //     $display("imem[%0d] = %h", i, sys_inst.imem_inst.mem[i]);
        // end
        
        // Print CPU state information
        // $display("=== CPU STATE DEBUG ===");
        // $display("Final PC = %h", sys_inst.cpu_inst.iaddr);
        // $display("Final instruction = %h", sys_inst.cpu_inst.idata);
        
        $display("=== SIMULATION ENDED (timeout) ===");
        $finish;
    end
    
    // Simulation timeout to prevent infinite run
    initial begin
        #10000000; // 10 ms
        $display("SIMULATION TIMEOUT: Ending simulation after timeout.");
        $finish;
    end
    
    // Task to dump zoomed image data to file
    task dump_zoomed_image;
        integer file_handle;
        integer row, col;
        integer pixel_value;
        begin
            file_handle = $fopen("zoomed_image.txt", "w");
            if (file_handle == 0) begin
                $display("Error: Could not open file for writing");
            end
            else begin
            
                $fdisplay(file_handle, "Zoomed Image Data (100x100 pixels)");
                $fdisplay(file_handle, "Format: Each line contains 100 pixel values in hex");
                $fdisplay(file_handle, "Memory base address: 8192 (0x2000)");
                $fdisplay(file_handle, "");
                
                // Dump all 100x100 pixels
                for (row = 0; row < 100; row = row + 1) begin
                    for (col = 0; col < 100; col = col + 1) begin
                        pixel_value = sys_inst.dmem_inst.mem[8192 + row * 100 + col];
                        $fwrite(file_handle, "%02h ", pixel_value);
                    end
                    $fdisplay(file_handle, ""); // New line after each row
                end
                
                $fclose(file_handle);
                $display("Zoomed image data saved to 'zoomed_image.txt'");
                
                // Also display first 10x10 section in console
                $display("=== ZOOMED IMAGE PREVIEW (10x10) ===");
                for (row = 0; row < 10; row = row + 1) begin
                    $write("Row %2d: ", row);
                    for (col = 0; col < 10; col = col + 1) begin
                        $write("%02h ", sys_inst.dmem_inst.mem[8192 + row * 100 + col]);
                    end
                    $display(""); // New line
                end
            end
        end
    endtask
    
    // Task to dump zoomed image memory to a .mem file
    task dump_zoomed_image_mem;
        begin
            $display("Dumping zoomed image memory to zoomed_image.mem...");
            $writememh("zoomed_image.mem", sys_inst.dmem_inst.mem, 8192, 16191);
            $display("Zoomed image memory dumped to zoomed_image.mem");
        end
    endtask
    
    // Add wire for debug_x15
    wire [31:0] debug_x15;
    
    // Instantiate the system
    system sys_inst (
        .clk(clk),
        .reset(rst),
        .debug_x15(debug_x15)
    );
    
    // Add simulation start and register print at the very top
    initial begin
        $display("SIM STARTED");
        #5;
        $display("INIT_REGS: x5(src_base)=%0d, x7(src_width)=%0d, x8(dest_width)=%0d, x9(dest_row)=%0d, x10(dest_base)=%0d", sys_inst.debug_x5, sys_inst.debug_x7, sys_inst.debug_x8, sys_inst.debug_x9, sys_inst.debug_x10);
    end
    
endmodule

/*
// Minimal testbench for image_mem
module image_mem_test;
    reg clk;
    reg [31:0] addr;
    reg [7:0] write_data;
    reg write_en;
    wire [7:0] read_data;

    // Instantiate image_mem
    image_mem uut (
        .clk(clk),
        .addr(addr),
        .write_data(write_data),
        .write_en(write_en),
        .read_data(read_data)
    );

    integer i;
    initial begin
        $display("=== IMAGE_MEM MINIMAL TESTBENCH START ===");
        clk = 0;
        addr = 0;
        write_data = 0;
        write_en = 0;
        #10;
        
        $display("First 20 values of image_mem after loading imagek.mem:");
        for (i = 0; i < 20; i = i + 1) begin
            addr = i;
            #1;
            $display("mem[%0d] = %h", i, read_data);
        end
        
        $display("\n=== CHECKING MIDDLE OF IMAGE (should show pattern) ===");
        $display("Values from mem[1000] to mem[1020] (middle of 50x50 image):");
        for (i = 1000; i < 1020; i = i + 1) begin
            addr = i;
            #1;
            $display("mem[%0d] = %h", i, read_data);
        end
        
        $display("\n=== CHECKING END OF IMAGE ===");
        $display("Values from mem[2400] to mem[2420] (end of image):");
        for (i = 2400; i < 2420; i = i + 1) begin
            addr = i;
            #1;
            $display("mem[%0d] = %h", i, read_data);
        end
        
        $display("=== TEST COMPLETE ===");
        $finish;
    end
endmodule
*/

