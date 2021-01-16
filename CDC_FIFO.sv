/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * MIT License                                                                     *
 *                                                                                 *
 * Copyright (c) 2019 https://github.com/Mop-u                                     *
 *                                                                                 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy    *
 * of this software and associated documentation files (the "Software"), to deal   * 
 * in the Software without restriction, including without limitation the rights    *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell       *
 * copies of the Software, and to permit persons to whom the Software is           *
 * furnished to do so, subject to the following conditions:                        *
 *                                                                                 *
 * The above copyright notice and this permission notice shall be included in all  *   
 * copies or substantial portions of the Software.                                 *
 *                                                                                 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR      *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,        *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE     *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER          *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,   * 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE   * 
 * SOFTWARE.                                                                       *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

`timescale 1ns / 1ps
/*
 * Module `CDC_FIFO`
 * 
 * The receiver will always peek at the first available valid data word. 
 * 
 * '_DA' corresponds to signals valid for usage in the data source clock domain A
 * '_DB' corresponds to signals valid for usage in the data recipient clock domain B
 */
module CDC_FIFO #(
    parameter DataWidth = 32,
    parameter FifoDepth = 4  // should not be lower than 4
)(
    input                      clk_DA,       // Source clock domain A
    input                      Push_DA,      // Push new value to FIFO from A-Domain
    input      [DataWidth-1:0] DataIn_DA,    // A-Domain data input to the FIFO
    output reg                 FifoFull_DA,  // High when the FIFO won't accept new values.
    
    input                      rst,          // reset
    
    input                      clk_DB,       // Receiver domain B
    input                      Deq_DB,       // Read next FIFO value (DataValid_DB goes low if empty)
    output reg                 DataValid_DB, // High when valid data is held in DataOut_DB
    output reg [DataWidth-1:0] DataOut_DB    // B-Domain synchronized peek of the FIFO output
);

localparam Depth = (FifoDepth < 4) ? 4 : FifoDepth;

localparam AddressWidth = $clog2(Depth);
reg  [DataWidth-1:0] SharedFifo [0:Depth-1];

// Fifo pointers
reg  [AddressWidth-1:0] FifoHead_DA;
reg  [AddressWidth-1:0] FifoTail_DB;

// Wraparound increment logic to support addressing arbitrary fifo depths
wire [AddressWidth-1:0] FifoHeadNext_DA = (FifoHead_DA == Depth-1) ? '0 : (FifoHead_DA + 1);
wire [AddressWidth-1:0] FifoTailNext_DB = (FifoTail_DB == Depth-1) ? '0 : (FifoTail_DB + 1);

/*
 * Async handshake logic
 * 
 * The ack regs reset their respective valid signal.
 * 
 * The final flags are derived from the XOR-ing of the respective domain flags,
 * then the entire vector is stored to a respective domain register.
 * Before any flags are used, relevant bits are sync'd with at least one further register.
 */ 
reg  [Depth-1:0] DataPush_DA;
reg  [Depth-1:0] DataAck_DB;
wire [Depth-1:0] SharedDataValid = (DataPush_DA ^ DataAck_DB);
reg  [Depth-1:0] SharedDataValid_DA;
reg  [Depth-1:0] SharedDataValid_DB;

// A-Domain flag vector synchronizer
always_ff @(posedge clk_DA, posedge rst) begin
    if(rst) SharedDataValid_DA <= '0;
    else    SharedDataValid_DA <= SharedDataValid;
end
// B-Domain flag vector synchronizer
always_ff @(posedge clk_DB, posedge rst) begin
    if(rst) SharedDataValid_DB <= '0;
    else    SharedDataValid_DB <= SharedDataValid;
end
// A-Domain fifo full flag synchronizer
wire PushEnable_DA = (Push_DA & !FifoFull_DA);
always_ff @(posedge clk_DA, posedge rst) begin
    if(rst) FifoFull_DA <= '0;
    else    FifoFull_DA <= SharedDataValid_DA[FifoHeadNext_DA];
end
// B-Domain data ready flag synchronizer
reg  DataReady_DB;
always_ff @(posedge clk_DB, posedge rst) begin
    if(rst) DataReady_DB <= 0;
    else    DataReady_DB <= SharedDataValid_DB[FifoTailNext_DB];
end

// A-Domain fifo push
always_ff @(posedge clk_DA, posedge rst) begin
    if(rst) begin
        DataPush_DA <= '0;
        FifoHead_DA <= '0;
    end
    else if(PushEnable_DA) begin
        SharedFifo[FifoHead_DA] <= DataIn_DA;
        DataPush_DA[FifoHead_DA] <= ~DataPush_DA[FifoHead_DA];
        FifoHead_DA <= FifoHeadNext_DA;
    end
end

// B-Domain fifo dequeue
always_ff @(posedge clk_DB, posedge rst) begin
    if(rst) begin
        DataAck_DB <= '0;
        FifoTail_DB <= '0;
        DataValid_DB <= '0;
        DataOut_DB <= '0;
    end
    else if(DataReady_DB & (!DataValid_DB|Deq_DB)) begin
        DataOut_DB  <= SharedFifo[FifoTail_DB];
        DataAck_DB[FifoTail_DB] <= ~DataAck_DB[FifoTail_DB];
        FifoTail_DB <= FifoTailNext_DB;
        DataValid_DB <= 1'b1;
    end
    else if(~DataReady_DB & Deq_DB) begin
        DataValid_DB <= 1'b0;
    end
end
endmodule
