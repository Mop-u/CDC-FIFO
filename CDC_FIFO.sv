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
 * This module guarantees a hold time of at least 1/2 a source clock domain cycle
 * before the receiver domain is allowed to latch the data.
 * 
 * The receiver will always peek at the first available valid data word. 
 * 
 * '_DA' corresponds to signals valid for usage in the data source clock domain A
 * '_DB' corresponds to signals valid for usage in the data recipient clock domain B
 */
module CDC_FIFO #(
    parameter DataWidth = 32,
    parameter FifoDepth = 1
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

// Fifo parameterization with safe clog2 if the fifo is single entry
localparam AddressWidth = (FifoDepth == 1) ? 1 : $clog2(FifoDepth);
reg  [DataWidth-1:0] SharedFifo [0:FifoDepth-1];

// Fifo pointers
reg  [AddressWidth-1:0] FifoHead_DA;
reg  [AddressWidth-1:0] FifoTail_DB;

// Wraparound increment logic to support addressing arbitrary fifo depths
wire [AddressWidth-1:0] FifoHeadNext_DA = (FifoHead_DA == FifoDepth-1) ? '0 : (FifoHead_DA + 1);
wire [AddressWidth-1:0] FifoTailNext_DB = (FifoTail_DB == FifoDepth-1) ? '0 : (FifoTail_DB + 1);

/*
 * Async handshake logic
 * 
 * The push & push confirm regs are for two-phase writing to the fifo
 * to ensure a minimum hold timing of 1/2 an A-Domain clock cycle. This
 * resolves race conditions where the valid data flag is read before the
 * data itself can propogate to the B-Domain buffer.
 * 
 * The ack regs reset their respective valid signal.
 * 
 * The final flags are derived from the XOR-ing of the respective domain flags,
 * which are updated as toggle flip-flops. This is a relatively cheap way to 
 * unify the states accross domains.
 */ 
reg  [FifoDepth-1:0] DataPush_DA;
reg  [FifoDepth-1:0] DataPushConfirm_DA;
reg  [FifoDepth-1:0] DataAck_DB;
wire [FifoDepth-1:0] SharedDataValid = (DataPushConfirm_DA ^ DataAck_DB);

/*
 * A-Domain fifo update flags
 * 
 * The FifoFull_DA flag only cares if the next fifo write address is occupied.
 * A-Domain tail & size information is redundant & not needed as an overrun
 * can be inferred from the SharedDataValid flag vector.
 */
wire PushEnable_DA = (Push_DA & !FifoFull_DA);
always_ff @(negedge clk_DA, posedge rst) begin
    if(rst) begin
        FifoFull_DA <= '0;
    end
    else begin
        FifoFull_DA <= SharedDataValid[FifoHead_DA];
    end
end

// B-Domain data ready flag synchronizer
reg  DataReady_DB;
always_ff @(negedge clk_DB, posedge rst) begin
    if(rst) begin
        DataReady_DB <= 0;
    end
    else begin
        DataReady_DB <= SharedDataValid[FifoTail_DB];
    end
end

// A-Domain fifo push phase 1
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

// A-Domain fifo push phase 2
always_ff @(negedge clk_DA, posedge rst) begin
    if(rst) begin
        DataPushConfirm_DA <= '0;
    end
    else begin
        DataPushConfirm_DA <= DataPush_DA;
    end
end

// B-Domain fifo dequeue block
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
