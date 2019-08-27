`timescale 1ns / 1ps
`include "CDC_FIFO.sv"
module CDC_FIFO_TB ();

    // Inputs
    reg clk_DA;
    reg clk_DB;
    reg Push_DA;
    reg Deq_DB;
    reg [7:0] DataIn_DA;
    reg rst;

    // Outputs
    wire FifoFull_DA;
    wire DataValid_DB;
    wire [7:0] DataOut_DB;

    // Instantiate the Unit Under Test (UUT)
    CDC_FIFO #(
        .DataWidth(8),
        .FifoDepth(32)
    ) uut (
        .clk_DA      (clk_DA),
        .Push_DA     (Push_DA),
        .DataIn_DA   (DataIn_DA),
        .FifoFull_DA (FifoFull_DA),
        .rst         (rst),
        .clk_DB      (clk_DB),
        .Deq_DB      (Deq_DB),
        .DataValid_DB(DataValid_DB),
        .DataOut_DB  (DataOut_DB)
    );

    initial begin
        $dumpfile("sim.vcd");
        $dumpvars;
        // Initialize Inputs
        clk_DA  = 0;
        clk_DB  = 0;
        Deq_DB  = 1;
        Push_DA = 1;
        rst    = 1;
        DataIn_DA = 0;
        #100
        rst = 0;
        #1000;
        rst = 1;
        #47
        rst = 0;
        #200
        Deq_DB = 0;
        #1000
        Deq_DB = 1;
        #600
        Push_DA = 0;
        #1000
        $finish;
    end
    always @(posedge clk_DA) begin
        if(rst) begin
            DataIn_DA <= '0;
        end
        else if(Push_DA & !FifoFull_DA) begin
            DataIn_DA <= DataIn_DA+1;
        end
    end
    always begin
        #10
        clk_DA = ~clk_DA;
    end
    always begin
        #3
        clk_DB = ~clk_DB;
    end
endmodule

