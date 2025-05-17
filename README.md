# final-verilog
# final-verilog

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/16/2025 03:33:54 PM
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// /////////////////////


module instruction_memory( input [5:0] addr, 
 output reg [21:0] inst 

); 

reg [21:0] instruction_memory [0:45]; 

initial begin // [21:18]opcode [17:12]destination [11:6]A [5:0]B // Opcode: 0000-Load 0011-Mul 0010-Add 1100-Jump 1000-Store 1101-JGT 
instruction_memory[0] = 22'b0000_000001_000000_000000; // Load R1, n 
instruction_memory[1] = 22'b0000_000010_000000_000001; // Load R2, nA 
instruction_memory[2] = 22'b0000_000011_000000_000010; // Load R3, nB 
instruction_memory[3] = 22'b0011_000100_000001_000010; // mul R4, R1, R2 
instruction_memory[4] = 22'b0011_000101_000001_000011; // mul R5, R1, R3 
instruction_memory[5] = 22'b0010_000110_000100_000101; // add R6, R4, R5 
instruction_memory[6] = 22'b0000_000111_000000_000000; // Load R7, 0 
instruction_memory[7] = 22'b0000_001000_000000_000000; // Load R8, 0 
instruction_memory[8] = 22'b0000_001001_000000_000000; // Load R9, 0 
instruction_memory[9] = 22'b1101_000010_000111_010010; // JGT R2 > R7 ? continue : jump to 18 
instruction_memory[10] = 22'b1101_000011_001000_010000; // JGT R3 > R8 ? continue : jump to 16 
instruction_memory[11] = 22'b0000_001010_000000_000110; // Load R10, R6 
instruction_memory[12] = 22'b1000_001010_001001_000000; // Store M[R10], R9 
instruction_memory[13] = 22'b0010_001010_001010_000001; // add R10, R10, 1 
instruction_memory[14] = 22'b0010_001000_001000_000001; // add R8, R8, 1 
instruction_memory[15] = 22'b1100_000000_000000_001011; // Jump 11 
instruction_memory[16] = 22'b0010_000110_000110_000001; // add R6, R6, 1 
instruction_memory[17] = 22'b1100_000000_000000_001010; // Jump 10 
instruction_memory[18] = 22'b0000_001011_000000_000000; // Load R11, 0 
instruction_memory[19] = 22'b0000_001100_000000_000000; // Load R12, 0 
instruction_memory[20] = 22'b0000_001101_000000_000000; // Load R13, 0 
instruction_memory[21] = 22'b0000_001110_000000_000000; // Load R14, 0 
instruction_memory[22] = 22'b0000_001111_000000_000100; // Load R15, R4 
instruction_memory[23] = 22'b0000_010000_000000_000110; // Load R16, R6 
instruction_memory[24] = 22'b1101_000010_001011_101101; // JGT R2 > R11 ? continue : jump to 45 
instruction_memory[25] = 22'b1101_000011_001100_101011; // JGT R3 > R12 ? continue : jump to 43 
instruction_memory[26] = 22'b1101_000001_001101_100110; // JGT R1 > R13 ? continue : jump to 38 
instruction_memory[27] = 22'b0000_010001_001110_000000; // Load R17, M[R14] 
instruction_memory[28] = 22'b0000_010010_001111_000000; // Load R18, M[R15] 
instruction_memory[29] = 22'b0000_010100_010000_000000; // Load R19, M[R16] 
instruction_memory[30] = 22'b0011_010100_010001_010010; // mul R20, R17, R18 
instruction_memory[31] = 22'b0010_010011_010010_010100; // add R19, R19, R20 
instruction_memory[32] = 22'b1000_010000_010011_000000; // Store M[R16], R19 
instruction_memory[33] = 22'b0010_001110_001110_000001; // add R14, R14, 1 
instruction_memory[34] = 22'b0010_010000_010000_000001; // add R16, R16, 1 
instruction_memory[35] = 22'b0010_001111_001111_000001; // add R15, R15, 1 
instruction_memory[36] = 22'b0010_001101_001101_000001; // add R13, R13, 1 
instruction_memory[37] = 22'b1100_000000_000000_011011; // Jump 27 
instruction_memory[38] = 22'b0010_001100_001100_000001; // add R12, R12, 1
instruction_memory[39] = 22'b0000_001111_000000_000100; // Load R15, R4 
instruction_memory[40] = 22'b0010_001111_001100_001100; // add R15, R15, R12 
instruction_memory[41] = 22'b0011_001110_001011_000010; // mul R14, R11, R2 
instruction_memory[42] = 22'b1100_000000_000000_011010; // Jump 26 
instruction_memory[43] = 22'b0010_001011_001011_000001; // add R11, R11, 1 
instruction_memory[44] = 22'b1100_000000_000000_011000; // Jump 24 
instruction_memory[45] = 22'b0000_000000_000000_000000; // EXIT 
end 

always @(*) begin inst = instruction_memory[addr]; end 

endmodule 

module Decoder ( 
    input  wire [21:0] inst, 
    output wire [3:0]  opcode, 
    output wire [5:0]  D,   
    output wire [5:0]  A,   
    output wire [5:0]  B,   
    output reg  [5:0]  Branch_addr, 
    output reg         RegWrite, 
    output reg         MemRead, 
    output reg         MemWrite, 
    output reg         MemToReg, 
    output reg [1:0]   ALU_FS,   
    output reg         Branch, 
    output reg         Jump, 
    output reg         CondJump, 
    output reg         pc_sel, 
    output reg         M1,     // ALU or RAM 
    output reg         M2      // register or constant
); 
    assign opcode = inst[21:18]; 
    assign D = inst[17:12];    
    assign A = inst[11:6];     
    assign B = inst[5:0];      
    always @(*) begin 
        Branch_addr = 6'b000000; 
        RegWrite = 0; 
        MemRead = 0; 
        MemWrite = 0; 
        MemToReg = 0; 
        ALU_FS = 2'b00;  
        Branch = 0; 
        Jump = 0; 
        CondJump = 0; 
        pc_sel = 0; 
        M1 = 0; 
        M2 = 0; 
        case (opcode) 
            4'b0000: begin // Load constant to register 
                RegWrite = 1;  
                M2 = 0;         
            end 
            4'b0001: begin  
                RegWrite = 1; 
                M1 = 0;   
                M2 = 1;   
            end 
            4'b0011: begin // Multiply 
                RegWrite = 1; 
                M1 = 0;   
                M2 = 1;   
                ALU_FS = 2'b00;   
            end 
            4'b0010: begin // Add 
                RegWrite = 1; 
                M1 = 0;   
                M2 = 1;   
                ALU_FS = 2'b01;   
            end 
            4'b1100: begin // Jump  
                Jump = 1; 
                Branch_addr = inst[5:0];  
                pc_sel = 1; 
            end 
            4'b1000: begin // Store to memory 
                MemWrite = 1; 
            end 
            4'b1101: begin // Jump if greater than (JGT)
                CondJump = 1; 
                Branch = 1;
                Branch_addr = inst[5:0];   
                pc_sel = 1; 
            end 
            default: begin 
            end 
        endcase 
    end 
endmodule 

module RegisterFile (  
    input clk,  
    input reset,  
    input [5:0] AA,          
    input [5:0] BA,          
    input [5:0] DA,           
    input [5:0] D_data,      
    input reg_write,         
    output [5:0] Adata,      
    output [5:0] Bdata       
);  
    reg [5:0] registers [0:31];   // Registers 6-bit also 32 bits just incase we need more but 20 should be fine 0-20 we used 1-20 
  
    always @(posedge clk or posedge reset) begin  
        if (reset) begin  
            registers[0] <= 6'b0;  
            registers[1] <= 6'b0;  
            registers[2] <= 6'b0;  
            registers[3] <= 6'b0;  
            registers[4] <= 6'b0;  
            registers[5] <= 6'b0;  
            registers[6] <= 6'b0;  
            registers[7] <= 6'b0;  
            registers[8] <= 6'b0;  
            registers[9] <= 6'b0;  
            registers[10] <= 6'b0;  
            registers[11] <= 6'b0;  
            registers[12] <= 6'b0;  
            registers[13] <= 6'b0;  
            registers[14] <= 6'b0;  
            registers[15] <= 6'b0;  
            registers[16] <= 6'b0;  
            registers[17] <= 6'b0;  
            registers[18] <= 6'b0;  
            registers[19] <= 6'b0;  
            registers[20] <= 6'b0;  
            registers[21] <= 6'b0;  
            registers[22] <= 6'b0;  
            registers[23] <= 6'b0;  
            registers[24] <= 6'b0;  
            registers[25] <= 6'b0;  
            registers[26] <= 6'b0;  
            registers[27] <= 6'b0;  
            registers[28] <= 6'b0;  
            registers[29] <= 6'b0;  
            registers[30] <= 6'b0;  
            registers[31] <= 6'b0;  
        end 
        else if (reg_write) begin  
            registers[DA] <= D_data;  
        end  
    end  
    assign Adata = registers[AA];  
    assign Bdata = registers[BA];  
endmodule 

module RAM( 
 input clk, 
 input MW, //memory read or write
 input [6:0]address, //RAM size is 128bit //matrices take up 65bits 
 input [6:0]data_in,
 output reg [6:0]data_out ); 
 reg[1:0]ram_block[0:127]; 
 always @(posedge clk) begin 
 if(MW) 
    ram_block[address] <= data_in; //write to RAM 
 else 
    data_out <= ram_block[address]; //read from RAM end 
 end
endmodule  

module MUX( //got this from lab 4 
    input [6:0] in1, 
    input [6:0] in2, 
    input sel, 
    output reg [6:0] out ); 
    always @(in1 or in2 or sel) begin 
        if(sel == 0) out = in1; 
        else out = in2;
         end 
endmodule  

module ALU( input [1:0] FS, // bit depends on number of operations ALU performs in our assembly code 
 input [6:0] Adata, 
 input [6:0] Bdata, 
 output reg [6:0] Ddata ); 
always @(FS) begin 
 if (FS ==2'b00) // multiply 
 Ddata <= Adata * Bdata; 
 else if (FS ==2'b01) // add 
 Ddata <= Adata + Bdata; 
 else if (FS ==2'b10) // compare 
 Ddata <= Adata < Bdata; 
 else 
 Ddata <= Adata; 
end 
endmodule  

module top( input clk, input reset,
    input [5:0] addr,
    input [5:0]D_data, // alu out - reg in XX     
    output mux_out,
    output wire Branch, //decode out - ? in 
    output wire Jump, //decode out - ?in 
    output wire CondJump, //decode out - ?in 
    output wire pc_sel, //decode out - pc in --no pc
    output wire [3:0] opcode, //decode out - out
    output wire MemRead, //decode out - ram in??? --no space in ram input
    output wire MemToReg //decode out    --??    
    output wire [21:0] inst; // im out - decode in (22 bit instruction)  
    //data    
    output wire [5:0]Adata; //reg out - alu in xx
    output wire [5:0]Bdata; //reg out - alu in xx
    //addresses
    output wire [5:0]D; //decode out - reg in XX
    output wire [5:0]A; //decode out - reg in XX
    output wire [5:0]B; //decode out - reg in XX
    
    output wire RegWrite; //decode out - reg in xx
    output wire MemWrite; //decode out - ram in XX
    
    output wire [1:0]ALU_FS;  //decode out - alu in xx
    
    
    output wire M1; //mux xx
    output wire M2; //mux
    
     //ALU 
    output wire [6:0]alu_result; 
 //RAM 
    output wire ram_data_out; 
 //MUX wire [6:0] MUX1_out;   
    );
    
    
     
 
 InstructionMemory imem ( 
    .addr(addr), 
    .inst(inst) 
); 
 
Decoder decoder ( 
    .inst(inst), 
    .opcode(opcode), 
    .D(D), 
    .A(A), 
    .B(B), 
    .Branch_addr(Branch_addr), 
    .RegWrite(RegWrite), 
    .MemRead(MemRead), 
    .MemWrite(MemWrite), 
    .MemToReg(MemToReg), 
    .ALU_FS(ALU_FS), 
    .Branch(Branch), 
    .Jump(Jump), 
    .CondJump(CondJump), 
    .pc_sel(pc_sel), 
    .M1(M1), //mux select
    .M2(M2)  //mux select
); 
 
RegisterFile reg_file ( 
    .clk(clk), 
    .reset(reset), 
    .AA(A), 
    .BA(B), 
    .DA(D), 
    .D_data(D_data), 
    .reg_write(RegWrite), 
    .Adata(Adata), 
    .Bdata(Bdata) 
); 

 
RAM ram (   
.clk(clk),  
.MW(MemWrite),  
.address(Adata),  
.data_in(Bdata), 
.data_out(ram_data_out)  
); 
 
ALU alu (  
.FS(ALU_FS),  
.Adata(Adata),   
.Bdata(Bdata), 
.Ddata(alu_result) 
);  
 
MUX MUX1 (  
    .sel(M1),  
    .in1(alu_result), //sel = 0 
    .in2(ram_data_out), //sel = 1 
    .out(MUX1_out)  
);  
 
assign mux_out = MUX1_out; 
  

endmodule 

 
